#pragma once
// ============================================================================
//  DAY 1 — geometry.hpp
//  "Every ADAS stack is secretly a geometry library with opinions."
//
//  DETAILED EXPLANATION:
//  This module provides the mathematical foundation for all ADAS operations.
//  It defines 2D vector arithmetic (Vec2), pose representations (position +
//  heading), matrix operations (Mat2 for Kalman filtering), and coordinate
//  frame transformations (ego-frame ↔ world-frame). All operations are
//  constexpr-eligible for compile-time evaluation where possible.
//
//  KEY COMPONENTS:
//    • Vec2: 2D vectors with dot/cross products, norms, rotations, normalization
//    • Pose: Position + yaw angle (heading in radians, 0 = +x axis, CCW positive)
//    • Mat2: 2×2 matrices for Kalman filter covariance / state transition
//    • Frame transforms: Convert between world coordinates and ego-relative coordinates
//    • Angle wrapping: Keep angles in [-π, π] to avoid wraparound issues
//    • Concepts: Arithmetic<T> for template metaprogramming
//
//  USAGE IN PIPELINE:
//    • Sensors use Vec2 for object positions and velocity vectors
//    • World simulates vehicle motion using pose and kinematics
//    • Tracker uses Vec2 and Mat2 for Kalman filter state
//    • Planner computes candidate paths as vectors of Vec2 waypoints
//    • Controller extracts target points from planned paths
//
//  What you learn today
//    • C++17: nested namespaces, [[nodiscard]], inline constexpr variables,
//             fold expressions, static_assert as a unit test
//    • C++20: concepts, three-way comparison (<=>), std::numbers
//    • Style : constexpr-everything, value semantics, tiny structs
// ============================================================================
#include <algorithm>
#include <cmath>
#include <compare>   // C++20: operator<=>
#include <concepts>  // C++20: std::integral, std::floating_point
#include <numbers>   // C++20: std::numbers::pi

namespace adas::geo {  // C++17: nested namespace definition in one line

// ---------------------------------------------------------------------------
// A *concept* is a named compile-time predicate on types. Think of it as a
// "type interface" that is checked at the call site instead of via inheritance.
// This is *static* polymorphism; we will meet *dynamic* polymorphism (virtual)
// on Day 5 with sensors, and you will be able to compare the two.
// ---------------------------------------------------------------------------
template <typename T>
concept Arithmetic = std::integral<T> || std::floating_point<T>;

// C++17 fold expression: expands to (a*a + b*b + c*c ... + 0.0)
template <Arithmetic... Ts>
[[nodiscard]] constexpr double sum_of_squares(Ts... v) noexcept {
    return ((static_cast<double>(v) * static_cast<double>(v)) + ... + 0.0);
}

// ---------------------------------------------------------------------------
// Vec2 — the workhorse. Note: no constructor, so it is an *aggregate*, which
// means you can write Vec2{1.0, 2.0} or Vec2{.x = 1.0, .y = 2.0} (C++20
// designated initializers). Everything is constexpr so the compiler can do
// geometry at compile time (see the static_asserts at the bottom).
// ---------------------------------------------------------------------------
struct Vec2 {
    double x{0.0};
    double y{0.0};

    [[nodiscard]] constexpr Vec2 operator+(Vec2 o) const noexcept { return {x + o.x, y + o.y}; }
    [[nodiscard]] constexpr Vec2 operator-(Vec2 o) const noexcept { return {x - o.x, y - o.y}; }
    [[nodiscard]] constexpr Vec2 operator*(double s) const noexcept { return {x * s, y * s}; }
    [[nodiscard]] constexpr Vec2 operator/(double s) const noexcept { return {x / s, y / s}; }
    constexpr Vec2& operator+=(Vec2 o) noexcept { x += o.x; y += o.y; return *this; }

    [[nodiscard]] constexpr double dot(Vec2 o) const noexcept { return x * o.x + y * o.y; }
    // 2-D cross product returns a scalar: positive if `o` is counter-clockwise of *this.
    [[nodiscard]] constexpr double cross(Vec2 o) const noexcept { return x * o.y - y * o.x; }
    [[nodiscard]] constexpr double norm2() const noexcept { return sum_of_squares(x, y); }
    [[nodiscard]] double norm() const noexcept { return std::sqrt(norm2()); }
    [[nodiscard]] double angle() const noexcept { return std::atan2(y, x); }
    [[nodiscard]] Vec2 rotated(double a) const noexcept {
        const double c = std::cos(a), s = std::sin(a);
        return {c * x - s * y, s * x + c * y};
    }
    [[nodiscard]] Vec2 normalized() const noexcept {
        const double n = norm();
        return n > 1e-12 ? *this / n : Vec2{};
    }

    // C++20: one line gives you ==, !=, <, <=, >, >= (lexicographic on x then y).
    constexpr auto operator<=>(const Vec2&) const = default;
};

[[nodiscard]] constexpr Vec2 operator*(double s, Vec2 v) noexcept { return v * s; }
[[nodiscard]] inline double distance(Vec2 a, Vec2 b) noexcept { return (a - b).norm(); }

// Distance from point p to the segment ab. Project p onto ab, clamp the
// parameter to [0,1], measure. Collision checkers that only test *points*
// along a path alias badly (the obstacle slips between samples) — see Day 7.
[[nodiscard]] inline double distance_to_segment(Vec2 p, Vec2 a, Vec2 b) noexcept {
    const Vec2 ab = b - a;
    const double len2 = ab.norm2();
    if (len2 < 1e-12) return distance(p, a);
    const double t = std::clamp((p - a).dot(ab) / len2, 0.0, 1.0);
    return distance(p, a + ab * t);
}

// A pose is position + heading. Heading is radians, counter-clockwise, 0 = +x.
struct Pose {
    Vec2 pos{};
    double yaw{0.0};
};

// C++17 `inline constexpr`: one definition across all translation units,
// no ODR headaches, no macros.
inline constexpr double kPi    = std::numbers::pi;   // C++20 <numbers>
inline constexpr double kTwoPi = 2.0 * kPi;

[[nodiscard]] constexpr double wrap_angle(double a) noexcept {
    while (a >  kPi) a -= kTwoPi;
    while (a < -kPi) a += kTwoPi;
    return a;
}

template <Arithmetic T>  // constrained template: only numbers may enter
[[nodiscard]] constexpr double deg2rad(T d) noexcept { return static_cast<double>(d) * kPi / 180.0; }

// Frame transforms. "Ego frame": x forward, y left, origin at the car.
[[nodiscard]] inline Vec2 to_ego_frame(Vec2 world_pt, const Pose& ego) noexcept {
    return (world_pt - ego.pos).rotated(-ego.yaw);
}
[[nodiscard]] inline Vec2 to_world_frame(Vec2 ego_pt, const Pose& ego) noexcept {
    return ego_pt.rotated(ego.yaw) + ego.pos;
}

// ---------------------------------------------------------------------------
// Mat2 — the smallest matrix that lets us write a real Kalman filter (Day 6).
// Layout: [a b]
//         [c d]
// ---------------------------------------------------------------------------
struct Mat2 {
    double a{1}, b{0}, c{0}, d{1};  // defaults to identity

    [[nodiscard]] constexpr Mat2 operator*(const Mat2& o) const noexcept {
        return {a * o.a + b * o.c, a * o.b + b * o.d,
                c * o.a + d * o.c, c * o.b + d * o.d};
    }
    [[nodiscard]] constexpr Mat2 operator+(const Mat2& o) const noexcept { return {a + o.a, b + o.b, c + o.c, d + o.d}; }
    [[nodiscard]] constexpr Mat2 operator*(double s) const noexcept { return {a * s, b * s, c * s, d * s}; }
    [[nodiscard]] constexpr Mat2 transposed() const noexcept { return {a, c, b, d}; }
};

// ---------------------------------------------------------------------------
// Compile-time unit tests. If any of these fail, the program does not build.
// This is the cheapest test framework in existence.
// ---------------------------------------------------------------------------
static_assert(Vec2{1, 2} + Vec2{3, 4} == Vec2{4, 6});
static_assert(Vec2{1, 0}.cross(Vec2{0, 1}) == 1.0);       // ccw is positive
static_assert(sum_of_squares(3, 4) == 25.0);
static_assert(Vec2{1, 2} < Vec2{2, 0});                    // from operator<=>
static_assert((Mat2{1, 2, 3, 4} * Mat2{}).d == 4.0);        // M * I == M

}  // namespace adas::geo
