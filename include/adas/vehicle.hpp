#pragma once
// ============================================================================
//  DAY 2 — vehicle.hpp
//  "The plant." A kinematic bicycle model — the same one you'd use to sanity
//  check a controller before touching a real vehicle.
//
//  DETAILED EXPLANATION:
//  This module models the ego vehicle's physical dynamics using a kinematic
//  bicycle model (no slip, point mass, simplified steering). It receives
//  acceleration and steering angle commands from the controller and updates
//  position/heading accordingly. State is fully encapsulated—only step() and
//  read-only getters are public, ensuring motion constraints are always checked.
//
//  KEY COMPONENTS:
//    • Control: Command struct (acceleration [m/s²], steer angle [rad])
//    • Vehicle: Simulates kinematics with enforced limits:
//        - Speed: clamped to [0, max_speed]
//        - Steer: clamped to [-max_steer, max_steer]
//        - Accel: clamped to [-max_accel, max_accel]
//    • Params: Configurable vehicle properties (wheelbase, limits)
//
//  KINEMATIC MODEL (bicycle model):
//    x' = v·cos(yaw)        (x-velocity)
//    y' = v·sin(yaw)        (y-velocity)
//    yaw' = (v/L)·tan(δ)    (rotation rate from steering)
//    v' = a                 (acceleration)
//    where L = wheelbase, δ = steering angle, a = accel, v = speed
//
//  USAGE IN PIPELINE:
//    • World creates the ego vehicle and calls step() each simulation tick
//    • Sensors read the vehicle's pose to transform measurements to world frame
//    • Controller commands the vehicle via the Control struct
//    • Tracker uses vehicle pose and speed to predict motion
//
//  What you learn today
//    • Encapsulation: private state, public *behaviour*, const-correct getters
//    • C++20 designated initializers for configuration structs
//    • `explicit`, `noexcept`, `std::clamp`
// ============================================================================
#include <algorithm>
#include <cmath>

#include "geometry.hpp"

namespace adas::sim {

using geo::Pose;
using geo::Vec2;

// The command a controller sends to the plant. A plain aggregate — data that
// crosses a module boundary should be boring.
struct Control {
    double accel{0.0};  // m/s^2, positive = throttle, negative = brake
    double steer{0.0};  // rad, positive = left
};

// ---------------------------------------------------------------------------
// Vehicle
//   x' = v cos(yaw)         y' = v sin(yaw)
//   yaw' = v / L * tan(delta)
//   v' = a
// Nobody outside this class can poke `pose_` or `speed_` directly. They can
// only *drive* it with step(). That is encapsulation in one sentence: the
// invariants (speed within limits, steer within limits) live in one place.
// ---------------------------------------------------------------------------
class Vehicle {
public:
    struct Params {
        double wheelbase{2.7};                 // m
        double max_steer{geo::deg2rad(35)};    // rad
        double max_speed{12.0};                // m/s
        double max_accel{3.0};                 // m/s^2
    };

    // `explicit` stops the compiler from silently converting a Pose into a
    // Vehicle. Surprising implicit conversions are how bugs are born.
    Vehicle(Pose start, Params p) noexcept : pose_(start), params_(p) {}
    // Delegating constructor (C++11). Why not `Params p = {}` as a default
    // argument? Because a nested struct is not "complete" until the enclosing
    // class is, and GCC refuses to use its default member initializers there.
    explicit Vehicle(Pose start) noexcept : Vehicle(start, Params{}) {}

    void step(Control u, double dt) noexcept {
        const double a     = std::clamp(u.accel, -params_.max_accel, params_.max_accel);
        const double delta = std::clamp(u.steer, -params_.max_steer, params_.max_steer);

        speed_ = std::clamp(speed_ + a * dt, 0.0, params_.max_speed);
        pose_.pos += Vec2{std::cos(pose_.yaw), std::sin(pose_.yaw)} * (speed_ * dt);
        pose_.yaw = geo::wrap_angle(pose_.yaw + speed_ / params_.wheelbase * std::tan(delta) * dt);
    }

    // Read-only views of state. Returning `const&` avoids a copy and the
    // `const` on the method promises the object is unchanged.
    [[nodiscard]] const Pose&   pose()   const noexcept { return pose_; }
    [[nodiscard]] double        speed()  const noexcept { return speed_; }
    [[nodiscard]] const Params& params() const noexcept { return params_; }

private:
    Pose   pose_;
    double speed_{0.0};
    Params params_;
};

}  // namespace adas::sim
