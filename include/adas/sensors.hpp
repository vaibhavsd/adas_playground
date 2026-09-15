#pragma once
// ============================================================================
//  DAY 5 — sensors.hpp
//  "Three liars, one truth." Radar, camera and lidar each see the same world
//  through a different, noisy keyhole. Fusion (Day 6) reconciles them.
//
//  DETAILED EXPLANATION:
//  This module simulates three real-world automotive sensors:
//
//    RADAR (Range + Bearing + Doppler):
//      • Strengths: Long range (~70m), measures velocity via Doppler
//      • Weaknesses: Poor angular resolution, occasional false returns
//      • Output: range [m], bearing [rad], range_rate [m/s]
//
//    CAMERA (Bearing + Estimated Range):
//      • Strengths: Excellent angular resolution, narrow FOV reduces clutter
//      • Weaknesses: Can't measure velocity, range estimate unreliable
//      • Output: bearing [rad], est_range [m] (from apparent size)
//
//    LIDAR (Point Cloud):
//      • Strengths: 360° coverage, precise 2D positions, dense
//      • Weaknesses: Short range (~35m), no velocity, range-dependent density
//      • Output: ego-frame centroid [x,y], point count (falls with distance)
//
//  KEY COMPONENTS:
//    • RadarReturn, CameraBox, LidarCluster: Raw sensor measurements in
//      sensor-native coordinates (NOT world frame yet)
//    • Measurement: std::variant combining all three types—ensures every
//      measurement type is handled (compiler enforces this)
//    • SensorFrame: Timestamped batch of measurements with ego pose & sensor id
//    • Detection: Normalized measurement after frame transformation:
//      - Always in world frame
//      - Position + optional velocity
//      - Uncertainty (sigma) for each sensor type
//    • Sensor (abstract base): Enforces NVI pattern—capture() is non-virtual
//      public interface; measure() is private virtual for subclasses
//    • Radar, Camera, Lidar (concrete): Implement physics + noise
//
//  ALGORITHM:
//    1. capture(world_snapshot): Timestamps the frame, calls private measure()
//    2. measure(): Loops over obstacles, filters by FOV/range, adds noise,
//       returns raw measurements in sensor coordinates
//    3. to_detection(): Uses std::visit to transform each raw measurement
//       into world-frame Detection—this is where sensor fusion begins
//
//  RANDOM NUMBER GENERATION:
//    Each sensor seeds its std::mt19937 RNG independently. Gaussian noise uses
//    std::normal_distribution; dropout uses std::uniform_real_distribution.
//
//  USAGE IN PIPELINE:
//    • World publishes ground truth obstacles
//    • Sensors read world snapshot, generate noisy measurements
//    • Sensors publish SensorFrames to a BlockingQueue
//    • Fusion (tracker.hpp) pulls frames and calls to_detection()
//
//  What you learn today
//    • Abstraction: an abstract base class with a pure virtual function
//    • Inheritance + runtime polymorphism (`virtual`, `override`, `final`)
//    • The Non-Virtual Interface (NVI) pattern: public non-virtual calls
//      private virtual — the base controls the *protocol*, the derived class
//      fills in the *behaviour*
//    • Rule of five: a polymorphic base gets a virtual destructor and deleted copies
//    • C++17 std::variant + std::visit + the "overloaded" trick (sum types)
//    • std::optional, std::string_view, <random>, C++20 `using enum`
// ============================================================================
#include <chrono>
#include <cmath>
#include <optional>
#include <random>
#include <string>
#include <string_view>
#include <variant>
#include <vector>

#include "concurrency.hpp"
#include "geometry.hpp"
#include "world.hpp"

namespace adas::sense {

using conc::Clock;
using geo::Pose;
using geo::Vec2;

enum class SensorKind { Radar = 0, Camera = 1, Lidar = 2 };

// ---------------------------------------------------------------------------
// Raw measurements. Each sensor speaks its own language — that is *real*.
// A radar does not give you an (x, y); it gives you range, bearing, Doppler.
// ---------------------------------------------------------------------------
struct RadarReturn  { double range; double bearing; double range_rate; };
struct CameraBox    { double bearing; double est_range; };
struct LidarCluster { Vec2 centroid_ego; int n_points; };

// A variant is a type-safe union: exactly one of these, and the compiler
// forces you to handle every alternative (see to_detection below).
using Measurement = std::variant<RadarReturn, CameraBox, LidarCluster>;

struct SensorFrame {
    SensorKind               kind;
    std::string_view         source;
    Pose                     ego_at_capture;
    Clock::time_point        stamp;
    std::vector<Measurement> raw;
};

// The common currency after normalization: a world-frame position with an
// uncertainty, and *maybe* a velocity (only radar can offer one).
struct Detection {
    SensorKind          kind;
    Vec2                pos_world;
    std::optional<Vec2> vel_world;   // "may be absent" without magic numbers
    double              sigma;       // 1-σ position noise, metres
    std::string_view    source;
    Clock::time_point   stamp;
};

// ---------------------------------------------------------------------------
// Sensor — the abstract base.
//
// `capture()` is the public, non-virtual entry point. It stamps the frame,
// records ego pose, and delegates the actual physics to the private virtual
// `measure()`. Derived classes cannot forget to timestamp; they *can't even
// see* that code. This is the NVI pattern.
// ---------------------------------------------------------------------------
class Sensor {
public:
    Sensor(std::string name, SensorKind kind, double rate_hz, unsigned seed)
        : name_(std::move(name)), kind_(kind),
          period_(static_cast<int>(1000.0 / rate_hz)), rng_(seed) {}

    virtual ~Sensor() = default;                 // deleting through a Sensor* must be safe
    Sensor(const Sensor&)            = delete;   // an RNG + identity should not be duplicated
    Sensor& operator=(const Sensor&) = delete;
    Sensor(Sensor&&)                 = delete;
    Sensor& operator=(Sensor&&)      = delete;

    [[nodiscard]] sense::SensorFrame capture(const sim::World::Snapshot& truth) {
        return SensorFrame{.kind = kind_, .source = name_, .ego_at_capture = truth.ego,
                           .stamp = Clock::now(), .raw = measure(truth)};
    }

    [[nodiscard]] std::string_view          name()   const noexcept { return name_; }
    [[nodiscard]] SensorKind                kind()   const noexcept { return kind_; }
    [[nodiscard]] std::chrono::milliseconds period() const noexcept { return period_; }

protected:
    // Helpers for the children. `protected` = "family only".
    [[nodiscard]] double noise(double sigma) { return sigma * gauss_(rng_); }
    [[nodiscard]] bool   dropped(double p)   { return uni_(rng_) < p; }
    [[nodiscard]] static bool in_fov(Vec2 rel, double max_range, double half_fov) noexcept {
        return rel.norm() <= max_range && std::abs(rel.angle()) <= half_fov;
    }

private:
    virtual std::vector<Measurement> measure(const sim::World::Snapshot& truth) = 0;  // pure virtual

    std::string                            name_;
    SensorKind                             kind_;
    std::chrono::milliseconds              period_;
    std::mt19937                           rng_;
    std::normal_distribution<double>       gauss_{0.0, 1.0};
    std::uniform_real_distribution<double> uni_{0.0, 1.0};
};

// ---------------------------------------------------------------------------
// Radar: long range, narrow-ish FOV, *measures velocity* (Doppler), drops
// returns occasionally. Great range accuracy, mediocre bearing.
// ---------------------------------------------------------------------------
class Radar final : public Sensor {
public:
    explicit Radar(unsigned seed) : Sensor("radar", SensorKind::Radar, 20.0, seed) {}

private:
    std::vector<Measurement> measure(const sim::World::Snapshot& truth) override {
        std::vector<Measurement> out;
        for (const auto& o : truth.obstacles) {
            const Vec2 rel = geo::to_ego_frame(o.pos, truth.ego);
            if (!in_fov(rel, 70.0, geo::deg2rad(60)) || dropped(0.10)) continue;
            const Vec2 los = (o.pos - truth.ego.pos).normalized();  // line of sight (world)
            out.push_back(RadarReturn{.range      = rel.norm() + noise(0.3),
                                      .bearing    = rel.angle() + noise(geo::deg2rad(2.0)),
                                      .range_rate = o.vel.dot(los) + noise(0.2)});
        }
        return out;
    }
};

// ---------------------------------------------------------------------------
// Camera: narrow FOV, excellent bearing, *terrible* range (it's guessing from
// pixel size), no velocity at all.
// ---------------------------------------------------------------------------
class Camera final : public Sensor {
public:
    explicit Camera(unsigned seed) : Sensor("camera", SensorKind::Camera, 15.0, seed) {}

private:
    std::vector<Measurement> measure(const sim::World::Snapshot& truth) override {
        std::vector<Measurement> out;
        for (const auto& o : truth.obstacles) {
            const Vec2 rel = geo::to_ego_frame(o.pos, truth.ego);
            if (!in_fov(rel, 45.0, geo::deg2rad(35)) || dropped(0.05)) continue;
            out.push_back(CameraBox{.bearing   = rel.angle() + noise(geo::deg2rad(0.5)),
                                    .est_range = rel.norm() * (1.0 + noise(0.15))});
        }
        return out;
    }
};

// ---------------------------------------------------------------------------
// Lidar: 360°, short range, precise position, no velocity. Point count
// falls off with distance (fewer beams hit a far object).
// ---------------------------------------------------------------------------
class Lidar final : public Sensor {
public:
    explicit Lidar(unsigned seed) : Sensor("lidar", SensorKind::Lidar, 10.0, seed) {}

private:
    std::vector<Measurement> measure(const sim::World::Snapshot& truth) override {
        std::vector<Measurement> out;
        for (const auto& o : truth.obstacles) {
            const Vec2 rel = geo::to_ego_frame(o.pos, truth.ego);
            if (!in_fov(rel, 35.0, geo::kPi)) continue;
            out.push_back(LidarCluster{.centroid_ego = rel + Vec2{noise(0.15), noise(0.15)},
                                       .n_points     = static_cast<int>(400.0 / std::max(rel.norm(), 1.0))});
        }
        return out;
    }
};

// ---------------------------------------------------------------------------
// The "overloaded" idiom (C++17): inherit call operators from N lambdas so
// std::visit can dispatch on the active variant alternative. The deduction
// guide (second line) lets us write overloaded{...} without template args.
// ---------------------------------------------------------------------------
template <class... Ts> struct overloaded : Ts... { using Ts::operator()...; };
template <class... Ts> overloaded(Ts...) -> overloaded<Ts...>;

// Normalize every raw measurement into world-frame Detection. If you add a
// fourth sensor type to the variant, this will *fail to compile* until you
// handle it — that is the whole point of a sum type.
[[nodiscard]] inline Detection to_detection(const Measurement& m, const SensorFrame& f) {
    const Pose& ego = f.ego_at_capture;
    return std::visit(overloaded{
        [&](const RadarReturn& r) {
            const Vec2 world = geo::to_world_frame(Vec2{r.range, 0.0}.rotated(r.bearing), ego);
            const Vec2 los   = (world - ego.pos).normalized();
            // Caveat for the curious: Doppler is *radial* velocity only. Feeding
            // it as a full 2-D velocity is a simplification (Exercise 6.3 fixes it).
            return Detection{.kind = f.kind, .pos_world = world, .vel_world = los * r.range_rate,
                             .sigma = 0.8, .source = f.source, .stamp = f.stamp};
        },
        [&](const CameraBox& c) {
            const Vec2 world = geo::to_world_frame(Vec2{c.est_range, 0.0}.rotated(c.bearing), ego);
            return Detection{.kind = f.kind, .pos_world = world, .vel_world = std::nullopt,
                             .sigma = std::max(1.0, 0.15 * c.est_range), .source = f.source, .stamp = f.stamp};
        },
        [&](const LidarCluster& l) {
            return Detection{.kind = f.kind, .pos_world = geo::to_world_frame(l.centroid_ego, ego), .vel_world = std::nullopt,
                             .sigma = 0.3, .source = f.source, .stamp = f.stamp};
        }}, m);
}

[[nodiscard]] constexpr std::string_view to_string(SensorKind k) noexcept {
    using enum SensorKind;   // C++20: bring enumerators into scope
    switch (k) {
        case Radar:  return "radar";
        case Camera: return "camera";
        case Lidar:  return "lidar";
    }
    return "?";
}

}  // namespace adas::sense
