// ============================================================================
//  DAY 11 — tests/test_main.cpp
//  "Trust, but verify." No framework, no dependencies: a CHECK macro and main.
//  Every test here is one you would actually write on a controls team.
// ============================================================================
#include <cmath>
#include <format>
#include <iostream>
#include <random>
#include <thread>
#include <vector>

#include "adas/concurrency.hpp"
#include "adas/controller.hpp"
#include "adas/planner.hpp"
#include "adas/sensors.hpp"
#include "adas/tracker.hpp"
#include "adas/world.hpp"

using namespace adas;
using namespace std::chrono_literals;

static int g_failures = 0;
#define CHECK(expr)                                                                      \
    do {                                                                                 \
        if (!(expr)) {                                                                   \
            ++g_failures;                                                                \
            std::cout << std::format("  FAIL {}:{}  {}\n", __FILE__, __LINE__, #expr);  \
        }                                                                                \
    } while (0)

static bool near(double a, double b, double tol = 1e-6) { return std::abs(a - b) < tol; }

void test_geometry() {
    CHECK(near(geo::wrap_angle(3 * geo::kPi), geo::kPi));
    CHECK(near(geo::wrap_angle(-3 * geo::kPi), -geo::kPi, 1e-9) || near(geo::wrap_angle(-3 * geo::kPi), geo::kPi, 1e-9));
    const geo::Vec2 r = geo::Vec2{1, 0}.rotated(geo::kPi / 2);
    CHECK(near(r.x, 0) && near(r.y, 1));
    const geo::Pose ego{.pos = {10, 10}, .yaw = geo::kPi / 2};
    const geo::Vec2 e = geo::to_ego_frame({10, 15}, ego);          // 5 m ahead
    CHECK(near(e.x, 5) && near(e.y, 0));
    const geo::Vec2 back = geo::to_world_frame(e, ego);
    CHECK(near(back.x, 10) && near(back.y, 15));
}

void test_course() {
    auto c = sim::Course::oval(40, 24, 160, 5.0);
    CHECK(c.size() == 160);
    // At s=0 the course point is (40,0), travel is +y (CCW), so "left" is -x.
    CHECK(c.lateral_offset({39.0, 0.0}) > 0.9);
    CHECK(c.lateral_offset({41.0, 0.0}) < -0.9);
    CHECK(c.nearest_index({40.0, 0.2}) == 0);
    CHECK(near(c.point_at(160.0).x, 40.0, 1e-9));   // wraps around
}

void test_kalman_converges() {
    std::mt19937 rng{7};
    std::normal_distribution<double> n{0.0, 0.5};
    fuse::Kalman1D kf{0.0, 0.0, 25.0, 25.0};
    double truth_p = 0.0;
    const double truth_v = 3.0, dt = 0.05;
    for (int i = 0; i < 200; ++i) {
        truth_p += truth_v * dt;
        kf.predict(dt, 1.0);
        kf.update_position(truth_p + n(rng), 0.25);
    }
    CHECK(near(kf.position(), truth_p, 0.4));
    CHECK(near(kf.velocity(), truth_v, 0.5));
    CHECK(kf.position_var() < 1.0);   // covariance shrank
}

void test_tracker_lifecycle() {
    fuse::Tracker tracker{{.confirm_hits = 3, .confirm_sensors = 2, .max_misses = 2}};
    auto t0 = conc::Clock::now();
    sense::Detection lidar{.kind = sense::SensorKind::Lidar, .pos_world = {5, 5}, .vel_world = std::nullopt,
                           .sigma = 0.3, .source = "lidar", .stamp = t0};
    sense::Detection radar = lidar;
    radar.kind = sense::SensorKind::Radar; radar.sigma = 0.8; radar.source = "radar";
    for (int i = 0; i < 3; ++i) {
        tracker.predict_to(t0 + std::chrono::milliseconds(50 * i));
        tracker.update(std::vector{lidar});
    }
    CHECK(tracker.confirmed().empty());        // 3 hits, but only one sensor type
    tracker.update(std::vector{radar});
    CHECK(tracker.confirmed().size() == 1);    // now radar agrees → confirmed
    for (int i = 3; i < 7; ++i) {   // starve it
        tracker.predict_to(t0 + std::chrono::milliseconds(50 * i));
        tracker.update(std::vector<sense::Detection>{});
    }
    CHECK(tracker.size() == 0);
}

void test_pure_pursuit_sign() {
    ctrl::PurePursuit pp;
    const geo::Pose ego{.pos = {0, 0}, .yaw = 0};
    const std::vector<geo::Vec2> left_path{{5, 2}, {10, 4}, {15, 6}};
    const std::vector<geo::Vec2> right_path{{5, -2}, {10, -4}, {15, -6}};
    CHECK(pp.steer(ego, 5.0, left_path) > 0.0);    // left target → positive steer
    CHECK(pp.steer(ego, 5.0, right_path) < 0.0);
    CHECK(pp.steer(ego, 5.0, {}) == 0.0);
}

void test_planner_avoids_obstacle() {
    auto course = sim::Course::oval(40, 24, 160, 5.0);
    plan::LatticePlanner planner;
    fuse::FusedPicture pic;
    pic.ego       = {.pos = course.point_at(0), .yaw = course.heading_at(0)};
    pic.ego_speed = 8.0;
    // Obstacle dead ahead on the centerline, stationary.
    const geo::Vec2 ahead = course.point_at(8);
    pic.tracks.push_back(fuse::Track{.id = 1,
                                     .x = fuse::Kalman1D{ahead.x, 0, 0.1, 0.1},
                                     .y = fuse::Kalman1D{ahead.y, 0, 0.1, 0.1},
                                     .hits = 10, .misses = 0, .sensor_mask = 0b111, .last_seen = {}});
    const auto p = planner.plan(pic, course);
    CHECK(std::abs(p.lateral_offset) > 1.0);   // it chose to swerve
    CHECK(p.cost < 500.0);                       // and the chosen path is collision-free
}

void test_blocking_queue_threads() {
    conc::BlockingQueue<int> q;
    std::atomic<int> total{0};
    {
        std::jthread consumer{[&](std::stop_token st) {
            while (!st.stop_requested()) for (int v : q.drain(5ms)) total += v;
            for (int v : q.drain(0ms)) total += v;   // final sweep
        }};
        std::jthread p1{[&] { for (int i = 1; i <= 100; ++i) q.push(i); }};
        std::jthread p2{[&] { for (int i = 1; i <= 100; ++i) q.push(i); }};
        p1.join(); p2.join();
        std::this_thread::sleep_for(30ms);
        consumer.request_stop();
    }
    CHECK(total.load() == 2 * 5050);
}

int main() {
    test_geometry();
    test_course();
    test_kalman_converges();
    test_tracker_lifecycle();
    test_pure_pursuit_sign();
    test_planner_avoids_obstacle();
    test_blocking_queue_threads();
    std::cout << (g_failures == 0 ? "All tests passed.\n" : std::format("{} failure(s).\n", g_failures));
    return g_failures == 0 ? 0 : 1;
}
