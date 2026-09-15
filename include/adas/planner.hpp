#pragma once
// ============================================================================
//  DAY 7 — planner.hpp
//  "Where should I be in the next 3 seconds?" A lattice planner: generate a
//  handful of candidate paths parallel to the centerline, score each one
//  against *predicted* obstacle positions, keep the cheapest.
//
//  DETAILED EXPLANATION:
//  The planner is the "decision maker"—given the current fused picture of
//  the world, it computes a safe, efficient path for the next 3 seconds.
//  Rather than solving complex optimization, it uses a simple lattice:
//    • Generate 5 candidate lateral offsets (-3.5, -2.0, 0, +2.0, +3.5 m)
//    • For each, compute 20 waypoints along the track's centerline
//    • Blend from current position to target offset (smooth, drivable paths)
//    • Score each path against predicted obstacle locations
//    • Return the lowest-cost path
//
//  KEY COMPONENTS:
//    • Path: Output to controller
//        - points: std::vector<Vec2> waypoints in world frame
//        - target_speed: Scalar speed recommendation [m/s]
//        - lateral_offset, min_clearance, cost: Diagnostics for tuning
//    • Planner (abstract): Interface allowing pluggable planners
//    • LatticePlanner (concrete): The lattice implementation
//    • Config: Tunable parameters (horizon, blend distance, offsets, weights, etc.)
//
//  ALGORITHM: build_and_score()
//    1. Generate 20 waypoints along the centerline +/- target offset
//    2. Blend from current lateral position to target offset over blend_m
//    3. For each waypoint, predict where tracked obstacles will be at
//       the time ego reaches that point (time-parameterized collision check)
//    4. Measure minimum clearance to each track
//    5. Cost = w_offset·offset² + collision_penalty + safety_margin_penalty
//       + off-track penalty (1e6 if outside lane bounds)
//    6. Higher min_clearance → higher target_speed (closer to cruise_speed)
//
//  PREDICTION STRATEGY:
//    Two modes (controlled by road_aware_prediction):
//      • Straight-line: pos_future = pos_now + velocity * dt
//      • Road-aware: Slide along the track centerline maintaining lateral
//        offset (more realistic on curves—constant-velocity would cut the corner)
//
//  PARALLELIZATION:
//    One std::async thread per candidate offset (5 threads). Each thread
//    calls build_and_score(); main thread collects futures and picks the
//    minimum-cost path. Cheap fork/join without managing threads explicitly.
//
//  USAGE IN PIPELINE:
//    • Tracker publishes FusedPicture (ego pose, tracks with predictions)
//    • Planner reads FusedPicture and Course
//    • Planner publishes Path to Latest<> for controller
//    • Controller extracts target speed and waypoints from Path
//    • Renderer draws path as a fading green ribbon
//
//  What you learn today
//    • A second abstract interface (Planner) — same idea as Sensor, so you
//      can swap in your own planner without touching main()
//    • std::async + std::future: fork/join parallelism in three lines
//    • Time-parameterized collision checking (predict where tracks *will be*)
//    • std::ranges::min_element with a pointer-to-member projection
// ============================================================================
#include <algorithm>
#include <cmath>
#include <future>
#include <limits>
#include <string_view>
#include <vector>

#include "geometry.hpp"
#include "tracker.hpp"
#include "world.hpp"

namespace adas::plan {

using fuse::FusedPicture;
using geo::Vec2;
using sim::Course;

struct Path {
    std::vector<Vec2> points;
    double            target_speed{0.0};
    double            lateral_offset{0.0};
    double            min_clearance{0.0};
    double            cost{0.0};
};

class Planner {
public:
    virtual ~Planner() = default;
    [[nodiscard]] virtual Path             plan(const FusedPicture& pic, const Course& course) = 0;
    [[nodiscard]] virtual std::string_view name() const noexcept = 0;
};

class LatticePlanner final : public Planner {
public:
    struct Config {
        double              horizon_m{30.0};
        double              blend_m{12.0};        // distance over which to reach the target offset
        int                 n_points{20};
        std::vector<double> offsets{-3.5, -2.0, 0.0, 2.0, 3.5};
        double              w_offset{0.3};        // cost per m² of lateral deviation
        double              w_clearance{10.0};    // cost per m² inside the safety bubble
        double              safe_clearance{2.5};  // m
        double              obstacle_radius{1.0}; // assumed; tracks don't know their size
        double              margin{0.4};          // extra bubble for tracker error
        double              cruise_speed{12.0};
        double              crawl_speed{2.5};
        // Predict other cars *along the road* instead of in a straight line.
        // Set to false and watch ego T-bone the car ahead on the first tight
        // bend at t≈15s — constant-velocity extrapolation leaves the curve
        // tangentially and under-predicts how far the car swings inward.
        bool                road_aware_prediction{true};
    };

    explicit LatticePlanner(Config c) : cfg_(std::move(c)) {}
    LatticePlanner() : LatticePlanner(Config{}) {}

    [[nodiscard]] std::string_view name() const noexcept override { return "LatticePlanner"; }

    [[nodiscard]] Path plan(const FusedPicture& pic, const Course& course) override {
        const double s0        = static_cast<double>(course.nearest_index(pic.ego.pos));
        const double lat0      = course.lateral_offset(pic.ego.pos);   // where we are now
        const double ds        = cfg_.horizon_m / cfg_.n_points / course.spacing();  // index units
        const double v_assumed = std::max(pic.ego_speed, 3.0);

        // Fork: one async task per candidate. std::launch::async forces a real
        // thread (the default policy may run lazily on .get()).
        std::vector<std::future<Path>> futures;
        for (double off : cfg_.offsets) {
            futures.push_back(std::async(std::launch::async, [&, off] {
                return build_and_score(pic, course, s0, ds, lat0, off, v_assumed);
            }));
        }

        // Join.
        std::vector<Path> candidates;
        for (auto& f : futures) candidates.push_back(f.get());

        return *std::ranges::min_element(candidates, {}, &Path::cost);
    }

private:
    [[nodiscard]] Path build_and_score(const FusedPicture& pic, const Course& course,
                                       double s0, double ds, double lat0, double off, double v_assumed) const {
        Path p;
        p.lateral_offset = off;
        p.points.reserve(static_cast<std::size_t>(cfg_.n_points));
        for (int k = 1; k <= cfg_.n_points; ++k) {
            const double s = s0 + k * ds;
            // Blend from current lateral position to the target offset over
            // `blend_m` metres so the path is drivable, not a step change.
            const double blend = std::clamp(k * ds * course.spacing() / cfg_.blend_m, 0.0, 1.0);
            p.points.push_back(course.point_at(s) + course.normal_at(s) * std::lerp(lat0, off, blend));
        }

        double cost = cfg_.w_offset * off * off;
        if (std::abs(off) + sim::World::kEgoRadius > course.half_width()) cost += 1e6;  // off-road

        const double bubble  = cfg_.obstacle_radius + sim::World::kEgoRadius + cfg_.margin;
        double       min_clr = std::numeric_limits<double>::infinity();

        // Pre-compute each track's road coordinates once (s, lateral, speed
        // along the road). Prediction then becomes "slide s forward".
        struct RoadTrack { double s, lat, v_along; Vec2 pos, vel; };
        std::vector<RoadTrack> rts;
        rts.reserve(pic.tracks.size());
        for (const auto& tr : pic.tracks) {
            const double s = static_cast<double>(course.nearest_index(tr.pos()));
            rts.push_back({.s = s, .lat = course.lateral_offset(tr.pos()),
                           .v_along = tr.vel().dot(course.tangent_at(s)), .pos = tr.pos(), .vel = tr.vel()});
        }
        auto predict = [&](const RoadTrack& rt, double t) -> Vec2 {
            if (!cfg_.road_aware_prediction) return rt.pos + rt.vel * t;
            const double s = rt.s + rt.v_along * t / course.spacing();
            return course.point_at(s) + course.normal_at(s) * rt.lat;
        };

        for (std::size_t k = 0; k < p.points.size(); ++k) {
            // When will ego be at point k? Use that time to predict each track.
            // Then measure clearance to the *segment* ending at k, not just the
            // point: with 1.5 m spacing and a 1 m/step relative motion, a
            // point-only check can step straight over a collision (it did — that
            // bug is how this comment got written).
            const double t    = (static_cast<double>(k) + 1.0) * ds * course.spacing() / v_assumed;
            const Vec2   from = (k == 0) ? pic.ego.pos : p.points[k - 1];
            for (const auto& rt : rts) {
                const Vec2   predicted = predict(rt, t);
                const double clr       = geo::distance_to_segment(predicted, from, p.points[k]) - bubble;
                min_clr = std::min(min_clr, clr);
                if (clr < 0.0)                     cost += 500.0;
                else if (clr < cfg_.safe_clearance) cost += cfg_.w_clearance * (cfg_.safe_clearance - clr) * (cfg_.safe_clearance - clr);
            }
        }

        p.cost          = cost;
        p.min_clearance = min_clr;
        const double ratio = std::clamp(min_clr / cfg_.safe_clearance, 0.0, 1.0);
        p.target_speed  = std::lerp(cfg_.crawl_speed, cfg_.cruise_speed, ratio);
        return p;
    }

    Config cfg_;
};

}  // namespace adas::plan
