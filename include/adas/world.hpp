#pragma once
// ============================================================================
//  DAY 4 — world.hpp
//  "Ground truth." The simulator owns the only true state of the universe:
//  the course, the traffic, and the ego vehicle. Everybody else gets *copies*
//  of a Snapshot, never a reference — that is what makes it thread-safe.
//
//  DETAILED EXPLANATION:
//  This module is the "ground truth" simulator. It owns all true state:
//    - The closed-loop track (Course) with waypoints and lane width
//    - All traffic vehicles moving along the track
//    - The ego vehicle being controlled by the stack
//
//  The World runs on one thread, steps the simulation, and publishes snapshots
//  to other threads (sensor simulation, fusion, visualization). Each snapshot
//  is a complete copy, so readers never race with the writer.
//
//  KEY COMPONENTS:
//    • Course: Closed-loop track defined by waypoints. Supports:
//        - point_at(s): Get world position at arc-length parameter s
//        - Tangent/normal vectors for frenet-frame operations
//        - Lateral offset calculation (signed distance from centerline)
//    • Obstacle: A traffic vehicle (id, position, velocity, radius)
//    • TrafficSpec: Configuration for spawning traffic vehicles
//    • World: Main simulator
//        - step(control, dt): Apply ego control, update all actors, detect crashes
//        - snapshot(): Return current state as a thread-safe copy
//    • Snapshot: The publishable view (ego pose/speed, obstacles, crash status, laps)
//
//  SIMULATION DETAILS:
//    • Ego vehicle: Kinematic model from vehicle.hpp
//    • Traffic: Toy ACC (adaptive cruise control)—cars don't collide with
//      each other, but they do slow down if someone ahead is slower
//    • Lap counting: Detects when ego crosses from 90% index back to 10%
//    • Crash detection: Off-track or obstacle collision
//    • Synchronization: shared_mutex allows many readers (sensors, tracker)
//      to read snapshot() concurrently while step() holds exclusive lock
//
//  USAGE IN PIPELINE:
//    • Main loop calls world.step(control) each tick
//    • Sensors call world.snapshot() to capture ego pose and obstacles
//    • Tracker uses obstacles and ego state for data association
//    • Renderer visualizes the snapshot
//
//  What you learn today
//    • std::shared_mutex in anger (one writer thread, many reader threads)
//    • C++20 ranges: views::iota, ranges::min_element with a *projection*
//    • std::span as a non-owning view of a vector
//    • std::lerp, std::fmod, nested private structs for "sim-only" data
// ============================================================================
#include <algorithm>
#include <cmath>
#include <format>
#include <mutex>
#include <ranges>
#include <shared_mutex>
#include <span>
#include <string_view>
#include <utility>
#include <vector>

#include "concurrency.hpp"
#include "geometry.hpp"
#include "vehicle.hpp"

namespace adas::sim {

using geo::Vec2;

// ---------------------------------------------------------------------------
// Course — a closed loop of waypoints. Parameter `s` is a *fractional index*:
// s = 3.5 is halfway between waypoint 3 and 4. That keeps the math trivial
// and is how many production planners represent a reference line anyway
// (arc-length param + lookup table).
// ---------------------------------------------------------------------------
class Course {
public:
    Course(std::vector<Vec2> pts, double half_width)
        : pts_(std::move(pts)), half_width_(half_width) {
        double perimeter = 0.0;
        for (std::size_t i = 0; i < pts_.size(); ++i)
            perimeter += geo::distance(pts_[i], pts_[(i + 1) % pts_.size()]);
        spacing_ = perimeter / static_cast<double>(pts_.size());
    }

    // Named constructor. C++20 views::iota(0, n) is a lazy range 0..n-1.
    [[nodiscard]] static Course oval(double rx, double ry, int n, double half_width) {
        std::vector<Vec2> pts;
        pts.reserve(static_cast<std::size_t>(n));
        for (int i : std::views::iota(0, n)) {
            const double t = geo::kTwoPi * i / n;
            pts.push_back({rx * std::cos(t), ry * std::sin(t)});
        }
        return Course{std::move(pts), half_width};
    }

    [[nodiscard]] Vec2 point_at(double s) const noexcept {
        const auto n = static_cast<double>(pts_.size());
        s = std::fmod(s, n);
        if (s < 0) s += n;
        const auto   i = static_cast<std::size_t>(s);
        const double f = s - static_cast<double>(i);
        const Vec2 a = pts_[i], b = pts_[(i + 1) % pts_.size()];
        return {std::lerp(a.x, b.x, f), std::lerp(a.y, b.y, f)};   // C++20 std::lerp
    }
    [[nodiscard]] Vec2   tangent_at(double s) const noexcept { return (point_at(s + 0.5) - point_at(s)).normalized(); }
    [[nodiscard]] Vec2   normal_at(double s)  const noexcept { return tangent_at(s).rotated(geo::kPi / 2); }  // left
    [[nodiscard]] double heading_at(double s) const noexcept { return tangent_at(s).angle(); }

    // ranges::min_element(range, comparator, projection): the projection maps
    // each element to the thing we compare (squared distance). No lambda soup.
    [[nodiscard]] std::size_t nearest_index(Vec2 p) const noexcept {
        auto it = std::ranges::min_element(pts_, {}, [p](Vec2 w) { return (w - p).norm2(); });
        return static_cast<std::size_t>(it - pts_.begin());
    }

    // Signed lateral offset from centerline: + = left of travel direction.
    [[nodiscard]] double lateral_offset(Vec2 p) const noexcept {
        const auto i = nearest_index(p);
        return tangent_at(static_cast<double>(i)).cross(p - pts_[i]);
    }

    [[nodiscard]] std::span<const Vec2> points()     const noexcept { return pts_; }  // non-owning view
    [[nodiscard]] double                half_width() const noexcept { return half_width_; }
    [[nodiscard]] double                spacing()    const noexcept { return spacing_; }
    [[nodiscard]] std::size_t           size()       const noexcept { return pts_.size(); }

private:
    std::vector<Vec2> pts_;
    double            half_width_;
    double            spacing_{1.0};  // metres per index step (average)
};

// What the rest of the stack is allowed to know about another road user.
struct Obstacle {
    int    id{0};
    Vec2   pos{};
    Vec2   vel{};
    double radius{1.0};
};

// ---------------------------------------------------------------------------
// World — the simulation thread calls step(); everyone else calls snapshot().
// ---------------------------------------------------------------------------
class World {
public:
    static constexpr double kEgoRadius = 1.2;

    struct TrafficSpec {
        double s0;              // starting index along course
        double speed;           // m/s (0 = parked cone)
        double lateral;         // m, + = left
        double radius{1.0};
    };

    struct Snapshot {
        Pose                  ego{};
        double                ego_speed{0.0};
        std::vector<Obstacle> obstacles;
        int                   laps{0};
        double                sim_time{0.0};
        bool                  crashed{false};
        std::string_view      crash_reason{};   // points at a string literal
    };

    World(Course course, std::vector<TrafficSpec> traffic, Vehicle ego)
        : course_(std::move(course)), ego_(ego) {
        int id = 1;
        for (const auto& t : traffic) {
            // C++20 designated initializers: readable *and* order-checked.
            Actor a{.pub = {.id = id++, .radius = t.radius},
                    .s = t.s0, .speed = t.speed, .lateral = t.lateral, .cur_speed = t.speed};
            place(a);
            actors_.push_back(a);
        }
        last_idx_ = course_.nearest_index(ego_.pose().pos);
    }

    void step(Control u, double dt) {
        std::unique_lock lk(mtx_);   // exclusive: we are mutating
        if (crashed_) return;
        sim_time_ += dt;

        // Traffic has a toy ACC: never drive faster than whoever is directly
        // ahead in your lane (ego included). Without this, the fastest car
        // happily rear-ends ego whenever ego slows for a jam.
        const double ego_s   = static_cast<double>(last_idx_);
        const double ego_lat = course_.lateral_offset(ego_.pose().pos);
        for (auto& a : actors_) {
            double v = a.speed;
            auto consider = [&](double s, double lat, double their_v) {
                const double gap_m = ahead_distance(a.s, s) * course_.spacing();
                if (gap_m > 0.0 && gap_m < kFollowGap && std::abs(lat - a.lateral) < 2.6) v = std::min(v, their_v);
            };
            for (const auto& b : actors_)
                if (&b != &a && b.speed > 0.0) consider(b.s, b.lateral, b.cur_speed);   // cones aren't leaders
            consider(ego_s, ego_lat, ego_.speed());
            // Finite accel/decel so the tracker (Day 6) can actually see it happen.
            a.cur_speed = std::clamp(v, a.cur_speed - kTrafficAccel * dt, a.cur_speed + kTrafficAccel * dt);
        }
        for (auto& a : actors_) {
            const Vec2 before = a.pub.pos;
            a.s += a.cur_speed * dt / course_.spacing();
            place(a);
            a.pub.vel = (a.pub.pos - before) / dt;   // finite-difference velocity
        }

        ego_.step(u, dt);
        const Vec2 p = ego_.pose().pos;

        // Lap counter: index wrapped from the last 10% to the first 10%.
        const auto idx = course_.nearest_index(p);
        const auto n   = course_.size();
        if (last_idx_ > n * 9 / 10 && idx < n / 10) {
            ++laps_;
            conc::EventLog::instance().post(std::format("Lap {} complete at t={:.1f}s", laps_, sim_time_));
        }
        last_idx_ = idx;

        // Crash detection.
        if (std::abs(course_.lateral_offset(p)) + kEgoRadius > course_.half_width()) {
            crashed_ = true; crash_reason_ = "left the road";
        }
        for (const auto& a : actors_) {
            if (geo::distance(a.pub.pos, p) < a.pub.radius + kEgoRadius) {
                crashed_ = true; crash_reason_ = "hit traffic";
                conc::EventLog::instance().post(std::format("  contact with obstacle {} (ego lat {:+.1f}, obstacle lat {:+.1f})",
                                                            a.pub.id, course_.lateral_offset(p), a.lateral));
            }
        }
        if (crashed_) conc::EventLog::instance().post(std::format("CRASH: {}", crash_reason_));
    }

    [[nodiscard]] Snapshot snapshot() const {
        std::shared_lock lk(mtx_);   // shared: many threads may read together
        Snapshot s;
        s.ego = ego_.pose();  s.ego_speed = ego_.speed();  s.laps = laps_;
        s.sim_time = sim_time_;  s.crashed = crashed_;  s.crash_reason = crash_reason_;
        s.obstacles.reserve(actors_.size());
        for (const auto& a : actors_) s.obstacles.push_back(a.pub);
        return s;   // returned by value → the caller owns a private copy
    }

    [[nodiscard]] bool crashed() const { std::shared_lock lk(mtx_); return crashed_; }

    // The course never changes after construction, so no lock is needed.
    [[nodiscard]] const Course& course() const noexcept { return course_; }

private:
    // Sim-private bookkeeping the outside world must never see.
    struct Actor {
        Obstacle pub;
        double   s;
        double   speed;       // nominal
        double   lateral;
        double   cur_speed{0.0};
    };
    static constexpr double kFollowGap    = 9.0;   // m
    static constexpr double kTrafficAccel = 2.5;   // m/s²

    // How far ahead (in index units, wrapping) is `s_other` from `s_from`?
    [[nodiscard]] double ahead_distance(double s_from, double s_other) const noexcept {
        const auto n = static_cast<double>(course_.size());
        return std::fmod(s_other - s_from + n, n);
    }

    void place(Actor& a) noexcept {
        a.pub.pos = course_.point_at(a.s) + course_.normal_at(a.s) * a.lateral;
    }

    mutable std::shared_mutex mtx_;
    Course                    course_;
    std::vector<Actor>        actors_;
    Vehicle                   ego_;
    int                       laps_{0};
    double                    sim_time_{0.0};
    bool                      crashed_{false};
    std::string_view          crash_reason_{};
    std::size_t               last_idx_{0};
};

}  // namespace adas::sim
