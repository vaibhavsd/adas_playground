#pragma once
// ============================================================================
//  DAY 6 — tracker.hpp
//  "Objects have memory." A detection is a rumour; a track is a belief.
//  The multi-object tracker turns a stream of noisy, multi-sensor detections
//  into a stable list of IDs with positions *and* velocities.
//
//  DETAILED EXPLANATION:
//  Sensors give noisy snapshots; the tracker creates stable "objects" by:
//    1. Estimating position and velocity (Kalman filtering)
//    2. Matching detections to existing tracks (data association)
//    3. Managing object identity over time (hit/miss counting, confirmation)
//
//  MULTI-SENSOR FUSION:
//  Radar provides velocity (Doppler), but camera and lidar do not. The tracker
//  combines them intelligently: radar seeds velocity estimates, then all sensors
//  refine position. A track must see 2+ distinct sensor types to be "confirmed"
//  (kills false positives from camera shadows or lidar reflections).
//
//  KEY COMPONENTS:
//    • Kalman1D: Constant-velocity 1D filter on one axis (x or y)
//        - State: [position, velocity]
//        - predict(): Advance state given dt and process noise
//        - update_position(): Correct using position measurement
//        - update_velocity(): Correct using velocity measurement (radar only)
//    • Track: One object's belief state
//        - Two Kalman1D filters (x, y axes, decoupled)
//        - hits/misses: Confirmation logic
//        - sensor_mask: Bitmask of which sensor types have hit this track
//    • FusedPicture: Output published to planner/controller
//        - Ego pose & speed
//        - std::vector<Track> of confirmed tracks
//    • Tracker: Main manager
//        - predict_to(time): Advance all tracks to a timestamp
//        - update(detections): Incorporate new detections
//        - confirmed(): Return only tracks passing confirmation threshold
//
//  ALGORITHM: predict() → update() → spawn() → prune()
//    1. predict_to(t): Kalman predict step for all tracks
//    2. update(detections): For each detection:
//       a) Find nearest track within gate (gating = gated distance < threshold)
//       b) If found: Kalman update position (and velocity if radar)
//       c) If not: spawn() a new tentative track
//    3. Unmatched tracks increment miss counter
//    4. prune(): Remove dead tracks (too many misses)
//
//  CONFIRMATION LOGIC:
//    • Tentative: 1 hit, any sensor
//    • Confirmed: hits >= confirm_hits AND sensor_mask has >= confirm_sensors types
//    • Dead: misses > max_misses
//    This prevents "ghost" tracks from single sensor glitches while allowing
//    3-sensor agreement to confirm quickly.
//
//  USAGE IN PIPELINE:
//    • Sensor threads publish SensorFrames to BlockingQueue
//    • Fusion pulls all pending frames, converts to Detections via to_detection()
//    • Tracker.predict_to(now) then .update(detections)
//    • Tracker publishes FusedPicture to Latest<> for planner & controller
//
//  What you learn today
//    • A real Kalman filter (constant-velocity, per axis) written by hand
//    • Nearest-neighbour data association with gating
//    • Track lifecycle: tentative → confirmed → dead (hits / misses)
//    • C++20: std::erase_if, unordered_set::contains, views::values,
//             views::filter, ranges::sort with a member-pointer projection
// ============================================================================
#include <algorithm>
#include <bit>        // C++20 std::popcount
#include <chrono>
#include <format>
#include <ranges>
#include <span>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "concurrency.hpp"
#include "geometry.hpp"
#include "sensors.hpp"

namespace adas::fuse {

using conc::Clock;
using geo::Mat2;
using geo::Pose;
using geo::Vec2;
using sense::Detection;

// ---------------------------------------------------------------------------
// Kalman1D — state x = [position, velocity], constant-velocity model.
//
//   predict:  x = F x            P = F P Fᵀ + Q      F = [1 dt; 0 1]
//   update:   y = z - H x        S = H P Hᵀ + R
//             K = P Hᵀ / S       x = x + K y         P = (I - K H) P
//
// With a 2-state filter and scalar measurements, S is a scalar and "matrix
// inverse" is a division. That is why we can do this with Mat2 and no Eigen.
// We run one of these for x and one for y (decoupled axes).
// ---------------------------------------------------------------------------
class Kalman1D {
public:
    Kalman1D(double p0, double v0, double p_var, double v_var) noexcept
        : p_(p0), v_(v0), P_{p_var, 0.0, 0.0, v_var} {}

    void predict(double dt, double q) noexcept {
        const Mat2 F{1.0, dt, 0.0, 1.0};
        p_ += v_ * dt;
        const double d2 = dt * dt, d3 = d2 * dt, d4 = d3 * dt;
        const Mat2 Q = Mat2{d4 / 4, d3 / 2, d3 / 2, d2} * q;   // discrete white-noise accel
        P_ = F * P_ * F.transposed() + Q;
    }

    // H = [1 0]
    void update_position(double z, double r) noexcept {
        const double S  = P_.a + r;
        const double kp = P_.a / S, kv = P_.c / S;
        const double y  = z - p_;
        p_ += kp * y;
        v_ += kv * y;
        P_ = Mat2{(1 - kp) * P_.a, (1 - kp) * P_.b, P_.c - kv * P_.a, P_.d - kv * P_.b};
    }

    // H = [0 1]  — only radar gets to call this.
    void update_velocity(double z, double r) noexcept {
        const double S  = P_.d + r;
        const double kp = P_.b / S, kv = P_.d / S;
        const double y  = z - v_;
        p_ += kp * y;
        v_ += kv * y;
        P_ = Mat2{P_.a - kp * P_.c, P_.b - kp * P_.d, (1 - kv) * P_.c, (1 - kv) * P_.d};
    }

    [[nodiscard]] double position()     const noexcept { return p_; }
    [[nodiscard]] double velocity()     const noexcept { return v_; }
    [[nodiscard]] double position_var() const noexcept { return P_.a; }

private:
    double p_, v_;
    Mat2   P_;
};

struct Track {
    int               id;
    Kalman1D          x, y;
    int               hits{1};
    int               misses{0};
    unsigned          sensor_mask{0};   // bit i set ⇔ SensorKind i has hit this track
    Clock::time_point last_seen;

    [[nodiscard]] Vec2 pos() const noexcept { return {x.position(), y.position()}; }
    [[nodiscard]] Vec2 vel() const noexcept { return {x.velocity(), y.velocity()}; }
    [[nodiscard]] double pos_sigma() const noexcept {
        return std::sqrt(0.5 * (x.position_var() + y.position_var()));
    }
};

// What fusion publishes to the planner and controller every cycle.
struct FusedPicture {
    Pose               ego{};
    double             ego_speed{0.0};
    std::vector<Track> tracks;
    Clock::time_point  stamp{};
};

// ---------------------------------------------------------------------------
// Tracker
// ---------------------------------------------------------------------------
class Tracker {
public:
    struct Config {
        double gate_m{3.5};          // max association distance
        double process_noise{1.0};   // accel variance (m/s²)²
        double radar_vel_var{4.0};   // variance used when Doppler seeds a track
        bool   use_doppler_updates{false};   // see the comment in update()
        int    confirm_hits{4};
        int    confirm_sensors{2};   // distinct sensor types required — kills camera ghosts
        int    max_misses{15};
    };

    explicit Tracker(Config c) noexcept : cfg_(c) {}
    Tracker() noexcept : Tracker(Config{}) {}   // delegating ctor (see vehicle.hpp)

    void predict_to(Clock::time_point now) {
        if (last_time_ != Clock::time_point{}) {
            const double dt = std::chrono::duration<double>(now - last_time_).count();
            for (auto& t : tracks_ | std::views::values) {   // iterate map values directly
                t.x.predict(dt, cfg_.process_noise);
                t.y.predict(dt, cfg_.process_noise);
            }
        }
        last_time_ = now;
    }

    void update(std::span<const Detection> dets) {
        std::unordered_set<int> touched;

        for (const auto& d : dets) {
            // Nearest existing track (projection = distance to this detection).
            auto best = std::ranges::min_element(
                tracks_, {}, [&](const auto& kv) { return geo::distance(kv.second.pos(), d.pos_world); });

            // Gate grows with measurement uncertainty: a camera guess 40 m out
            // is allowed to be several metres off; a lidar centroid is not.
            const double gate = cfg_.gate_m + 2.0 * d.sigma;
            if (best != tracks_.end() && geo::distance(best->second.pos(), d.pos_world) < gate) {
                Track& t = best->second;
                const double r = d.sigma * d.sigma;
                t.x.update_position(d.pos_world.x, r);
                t.y.update_position(d.pos_world.y, r);
                // Radar Doppler is *radial* velocity. Treating it as a full 2-D
                // velocity update (t.x.update_velocity(...)) biases the direction
                // and — we measured this — walks predicted obstacles into the
                // wall. So Doppler only seeds new tracks (see spawn()). Doing it
                // properly needs a coupled 4-state filter: Exercise 6.3.
                if (d.vel_world && cfg_.use_doppler_updates) {
                    t.x.update_velocity(d.vel_world->x, cfg_.radar_vel_var);
                    t.y.update_velocity(d.vel_world->y, cfg_.radar_vel_var);
                }
                const bool was_confirmed = is_confirmed(t);
                ++t.hits;
                t.misses = 0;
                t.last_seen = d.stamp;
                t.sensor_mask |= 1u << static_cast<unsigned>(d.kind);
                touched.insert(t.id);
                if (!was_confirmed && is_confirmed(t))
                    conc::EventLog::instance().post(std::format("Track {} confirmed (last hit from {})", t.id, d.source));
            } else {
                spawn(d);
            }
        }

        for (auto& [id, t] : tracks_)                 // C++17 structured bindings
            if (!touched.contains(id)) ++t.misses;    // C++20 .contains

        // C++20 std::erase_if works on maps — no more erase-while-iterating dance.
        std::erase_if(tracks_, [&](const auto& kv) {
            const bool dead = kv.second.misses > cfg_.max_misses;
            if (dead && is_confirmed(kv.second))
                conc::EventLog::instance().post(std::format("Track {} dropped (lost)", kv.first));
            return dead;
        });
    }

    [[nodiscard]] std::vector<Track> confirmed() const {
        std::vector<Track> out;
        auto confirmed = [&](const Track& t) { return is_confirmed(t); };
        for (const Track& t : tracks_ | std::views::values | std::views::filter(confirmed))
            out.push_back(t);
        std::ranges::sort(out, {}, &Track::id);   // projection via pointer-to-member
        return out;
    }

    [[nodiscard]] std::size_t size() const noexcept { return tracks_.size(); }

private:
    [[nodiscard]] bool is_confirmed(const Track& t) const noexcept {
        return t.hits >= cfg_.confirm_hits &&
               std::popcount(t.sensor_mask) >= cfg_.confirm_sensors;   // C++20 <bit>
    }

    void spawn(const Detection& d) {
        const Vec2 v0 = d.vel_world.value_or(Vec2{});   // radar-initialized tracks converge faster
        const double pv = d.sigma * d.sigma;
        const double vv = d.vel_world ? cfg_.radar_vel_var : 25.0;
        tracks_.emplace(next_id_, Track{.id = next_id_,
                                        .x  = Kalman1D{d.pos_world.x, v0.x, pv, vv},
                                        .y  = Kalman1D{d.pos_world.y, v0.y, pv, vv},
                                        .hits = 1, .misses = 0,
                                        .sensor_mask = 1u << static_cast<unsigned>(d.kind),
                                        .last_seen = d.stamp});
        ++next_id_;
    }

    Config                         cfg_;
    std::unordered_map<int, Track> tracks_;
    int                            next_id_{1};
    Clock::time_point              last_time_{};
};

}  // namespace adas::fuse
