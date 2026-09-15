#pragma once
// ============================================================================
//  DAY 8 — controller.hpp
//  "Make the plant follow the plan." Pure pursuit for steering, PID for
//  speed. This is your home turf — so today the lesson is about *how C++
//  expresses the design*, not the control theory.
//
//  DETAILED EXPLANATION:
//  The controller takes a planned path and converts it into vehicle commands
//  (acceleration and steering). It runs at ~30 Hz and solves two independent
//  problems:
//    1. Lateral (steering): Pure pursuit—aim for a lookahead point on the path
//    2. Longitudinal (speed): PID feedback to match target speed
//
//  KEY COMPONENTS:
//    • PID: Textbook proportional-integral-derivative controller
//        - step(error, dt): Returns command given error and timestep
//        - Anti-windup: Clamp integral term to [-limit, +limit]
//        - First-order numerics: Derivative uses backward difference
//    • LateralController (concept): Static polymorphism—any type that
//      implements steer(ego, speed, path_span) satisfies this concept
//    • PurePursuit (concrete): Steering controller
//        - Compute lookahead distance: Ld = max(min_Ld, gain * speed)
//        - Find waypoint at distance >= Ld along path
//        - Compute steering angle to reach that waypoint (bicycle model inverse)
//        - wheelbase, lookahead_gain, lookahead_min are tunable
//    • VehicleController: Glues PurePursuit + PID together
//        - Template <LateralController Lat>: Accept any controller satisfying concept
//        - compute(ego, speed, path, dt): Returns Control struct for vehicle
//
//  LATERAL CONTROL (Pure Pursuit):
//    Pure pursuit is a classical path-tracking algorithm:
//      1. Find target point on path at distance Ld = max(4m, 0.6 * speed)
//      2. Compute cross-track error (in ego frame)
//      3. Compute steering angle: δ = atan2(2·L·sin(α), Ld)
//         where L = wheelbase, α = angle to target
//    Advantages: Stable, easy to tune, adapts to speed
//    Result: Steering angle [rad] fed to vehicle
//
//  LONGITUDINAL CONTROL (PID):
//    Standard PID loop tracking a speed setpoint:
//      error = target_speed - current_speed
//      accel = Kp·error + Ki·∫error·dt + Kd·d(error)/dt
//    Anti-windup prevents integral term from "saturating" on long steady-state errors
//    Result: Acceleration [m/s²] fed to vehicle
//
//  USAGE IN PIPELINE:
//    • Planner publishes Path with waypoints and target_speed
//    • Controller reads Path, calls compute() each cycle
//    • compute() invokes lateral.steer() and speed_pid.step()
//    • Returns Control (accel, steer) sent to world.step(control, dt)
//    • Vehicle executes the control via bicycle model kinematics
//
//  DESIGN PATTERN: Static Polymorphism
//    Unlike Sensor/Planner which use virtual functions (dynamic polymorphism),
//    LateralController uses C++20 concepts (static polymorphism). Compiler
//    checks "does this type implement steer()?" at instantiation time—zero
//    runtime overhead, compiler errors catch mismatches before linking.
//
//  What you learn today
//    • Static polymorphism with a concept (LateralController) vs the dynamic
//      polymorphism of Sensor/Planner. Same idea, zero virtual-call overhead,
//      checked at compile time.
//    • Class template argument deduction (CTAD) for VehicleController
//    • std::span<const Vec2> as the universal "give me a path" parameter
//    • std::ranges::find_if
// ============================================================================
#include <algorithm>
#include <cmath>
#include <concepts>
#include <span>

#include "geometry.hpp"
#include "planner.hpp"
#include "vehicle.hpp"

namespace adas::ctrl {

using geo::Pose;
using geo::Vec2;
using sim::Control;

class PID {
public:
    struct Gains { double kp, ki, kd; };

    explicit PID(Gains g, double integral_limit = 5.0) noexcept : g_(g), i_limit_(integral_limit) {}

    [[nodiscard]] double step(double err, double dt) noexcept {
        if (dt <= 0.0) return last_out_;
        integral_ = std::clamp(integral_ + err * dt, -i_limit_, i_limit_);   // anti-windup
        const double deriv = first_ ? 0.0 : (err - prev_err_) / dt;
        first_    = false;
        prev_err_ = err;
        last_out_ = g_.kp * err + g_.ki * integral_ + g_.kd * deriv;
        return last_out_;
    }

    void reset() noexcept { integral_ = 0; prev_err_ = 0; last_out_ = 0; first_ = true; }

private:
    Gains  g_;
    double i_limit_;
    double integral_{0.0};
    double prev_err_{0.0};
    double last_out_{0.0};
    bool   first_{true};
};

// ---------------------------------------------------------------------------
// A *concept* describing "anything that can compute a steering angle".
// PurePursuit satisfies it; so would a Stanley controller or an MPC — with no
// base class and no vtable. If you pass something that doesn't fit, the
// compiler tells you exactly which requirement failed.
// ---------------------------------------------------------------------------
template <typename C>
concept LateralController = requires(const C c, const Pose& ego, double speed, std::span<const Vec2> path) {
    { c.steer(ego, speed, path) } -> std::convertible_to<double>;
};

struct PurePursuit {
    double lookahead_min{4.0};
    double lookahead_gain{0.6};   // Ld = max(min, gain * v)
    double wheelbase{2.7};

    [[nodiscard]] double steer(const Pose& ego, double speed, std::span<const Vec2> path) const noexcept {
        if (path.empty()) return 0.0;
        const double Ld = std::max(lookahead_min, lookahead_gain * speed);
        auto it = std::ranges::find_if(path, [&](Vec2 p) { return geo::distance(p, ego.pos) >= Ld; });
        const Vec2   target = (it == path.end()) ? path.back() : *it;
        const Vec2   e      = geo::to_ego_frame(target, ego);
        const double alpha  = std::atan2(e.y, e.x);
        const double L      = std::max(e.norm(), 1e-3);
        return std::atan2(2.0 * wheelbase * std::sin(alpha), L);
    }
};
static_assert(LateralController<PurePursuit>);   // prove it at compile time

// ---------------------------------------------------------------------------
// VehicleController<Lat> — glue. CTAD lets main() write
//     VehicleController controller{PurePursuit{...}, PID{...}};
// and the compiler deduces Lat = PurePursuit.
// ---------------------------------------------------------------------------
template <LateralController Lat>
class VehicleController {
public:
    VehicleController(Lat lateral, PID speed_pid) noexcept
        : lat_(std::move(lateral)), pid_(std::move(speed_pid)) {}

    [[nodiscard]] Control compute(const Pose& ego, double speed, const plan::Path& path, double dt) noexcept {
        if (path.points.empty()) return {.accel = -1.0, .steer = 0.0};   // no plan → gently brake
        return {.accel = pid_.step(path.target_speed - speed, dt),
                .steer = lat_.steer(ego, speed, path.points)};
    }

private:
    Lat lat_;
    PID pid_;
};

}  // namespace adas::ctrl
