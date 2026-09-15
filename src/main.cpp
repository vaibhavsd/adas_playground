// ============================================================================
//  DAY 10 — main.cpp
//  "The launch file." Seven threads, one world, zero data races.
//
//      sim ──► World ◄── snapshot() ◄── radar ─┐
//       ▲                                camera ─┼─► BlockingQueue ─► fusion ─► Latest<FusedPicture>
//       │                                 lidar ─┘                                   │
//   Shared<Control> ◄── control ◄── Shared<Path> ◄── planner ◄──────────────────────┘
//                                                       render ◄──(reads everything)
//
//  What you learn today
//    • std::jthread + std::stop_token: threads that stop and join themselves
//    • std::latch: "everyone on your marks… go"
//    • consteval / constinit, std::from_chars, string_view::starts_with
//    • std::unique_ptr<Sensor> for owning polymorphic objects
//    • std::ranges::transform with a back_inserter
// ============================================================================
#include <algorithm>
#include <atomic>
#include <charconv>
#include <chrono>
#include <csignal>
#include <format>
#include <iostream>
#include <iterator>
#include <latch>
#include <memory>
#include <ranges>
#include <string_view>
#include <thread>
#include <vector>

#include "adas/concurrency.hpp"
#include "adas/controller.hpp"
#include "adas/planner.hpp"
#include "adas/render.hpp"
#include "adas/sensors.hpp"
#include "adas/tracker.hpp"
#include "adas/world.hpp"

using namespace std::chrono_literals;
using adas::conc::Clock;

// consteval = *must* run at compile time. A typo like hz_to_period(0) is a
// compile error, not a runtime divide-by-zero.
consteval std::chrono::milliseconds hz_to_period(int hz) { return std::chrono::milliseconds{1000 / hz}; }

inline constexpr auto kSimPeriod     = hz_to_period(100);
inline constexpr auto kControlPeriod = hz_to_period(50);
inline constexpr auto kPlanPeriod    = hz_to_period(10);
inline constexpr auto kRenderPeriod  = hz_to_period(12);

// constinit: guaranteed constant-initialized (no static-init-order fiasco),
// but still mutable at runtime — exactly what a signal flag needs.
constinit std::atomic<bool> g_interrupted{false};
extern "C" void on_sigint(int) { g_interrupted.store(true); }

struct Args {
    int      seconds{60};
    unsigned seed{42};
};

Args parse_args(int argc, char** argv) {
    Args a;
    for (std::string_view arg : std::span(argv + 1, argv + argc)) {
        auto parse_into = [](std::string_view s, auto& out) {
            std::from_chars(s.data(), s.data() + s.size(), out);   // C++17, no exceptions, no locale
        };
        if (arg.starts_with("--seconds=")) parse_into(arg.substr(10), a.seconds);   // C++20 starts_with
        else if (arg.starts_with("--seed=")) parse_into(arg.substr(7), a.seed);
    }
    return a;
}

int main(int argc, char** argv) {
    const Args args = parse_args(argc, argv);
    std::signal(SIGINT, on_sigint);

    // ---- The world ---------------------------------------------------------
    auto course = adas::sim::Course::oval(40.0, 24.0, 160, 5.0);
    adas::sim::Vehicle ego{adas::geo::Pose{.pos = course.point_at(0), .yaw = course.heading_at(0)}};
    adas::sim::World world{course,
                           {{.s0 = 20, .speed = 4.0, .lateral = 0.0},
                            {.s0 = 50, .speed = 3.0, .lateral = 1.5},
                            {.s0 = 80, .speed = 5.0, .lateral = -1.0},
                            {.s0 = 110, .speed = 3.5, .lateral = 0.5},
                            {.s0 = 140, .speed = 4.5, .lateral = -1.5},
                            {.s0 = 35, .speed = 0.0, .lateral = 2.5, .radius = 0.6},   // parked cone
                            {.s0 = 95, .speed = 0.0, .lateral = -2.5, .radius = 0.6}},
                           ego};

    // ---- The stack ---------------------------------------------------------
    // Owning polymorphic objects: unique_ptr<Base> to concrete Derived.
    std::vector<std::unique_ptr<adas::sense::Sensor>> sensors;
    sensors.push_back(std::make_unique<adas::sense::Radar>(args.seed + 1));
    sensors.push_back(std::make_unique<adas::sense::Camera>(args.seed + 2));
    sensors.push_back(std::make_unique<adas::sense::Lidar>(args.seed + 3));

    adas::fuse::Tracker tracker;
    std::unique_ptr<adas::plan::Planner> planner = std::make_unique<adas::plan::LatticePlanner>();
    adas::ctrl::VehicleController controller{adas::ctrl::PurePursuit{.wheelbase = ego.params().wheelbase},
                                             adas::ctrl::PID{{.kp = 1.2, .ki = 0.1, .kd = 0.02}}};
    adas::ui::Renderer renderer{96, 34, {-48, -32}, {48, 32}};

    // ---- The phone lines ---------------------------------------------------
    adas::conc::BlockingQueue<adas::sense::SensorFrame>       queue;
    adas::conc::Latest<adas::fuse::FusedPicture>              picture;
    // Raw per-cycle detections (pre-tracker, world frame), for recording/analysis
    // only — the renderer draws the fused picture, not this; see render.hpp.
    adas::conc::Latest<std::vector<adas::sense::Detection>>   raw_dets;
    adas::conc::Shared<adas::plan::Path>               path;
    adas::conc::Shared<adas::sim::Control>             command;
    adas::conc::Stats                                  stats;
    std::atomic<bool>                                  game_over{false};
    const auto deadline = Clock::now() + std::chrono::seconds(args.seconds);

    // ---- The threads -------------------------------------------------------
    const std::ptrdiff_t n_threads = 5 + static_cast<std::ptrdiff_t>(sensors.size());
    std::latch start_gate{n_threads};   // nobody runs until everybody is ready
    std::vector<std::jthread> threads;

    // 1. Simulation: 100 Hz. The only writer to World.
    threads.emplace_back([&](std::stop_token st) {
        start_gate.arrive_and_wait();
        adas::conc::run_at_rate(st, kSimPeriod, [&](double dt) {
            world.step(command.get(), dt);
            ++stats.sim_ticks;
            if (world.crashed()) game_over = true;
        });
    });

    // 2..4. Sensors: each at its own rate. The `sensor = s.get()` init-capture
    // gives each lambda its own raw pointer; the unique_ptrs in `sensors` keep owning.
    for (auto& s : sensors) {
        threads.emplace_back([&, sensor = s.get()](std::stop_token st) {
            start_gate.arrive_and_wait();
            adas::conc::run_at_rate(st, sensor->period(), [&](double) {
                queue.push(sensor->capture(world.snapshot()));
            });
        });
    }

    // 5. Fusion: event-driven — wakes when frames arrive.
    threads.emplace_back([&](std::stop_token st) {
        start_gate.arrive_and_wait();
        while (!st.stop_requested()) {
            auto frames = queue.drain(20ms);
            if (frames.empty()) continue;

            const auto now = Clock::now();
            tracker.predict_to(now);

            std::vector<adas::sense::Detection> dets;
            for (const auto& f : frames) {
                ++stats.sensor_frames[static_cast<std::size_t>(f.kind)];
                std::ranges::transform(f.raw, std::back_inserter(dets),
                                       [&](const adas::sense::Measurement& m) { return adas::sense::to_detection(m, f); });
            }
            raw_dets.publish(dets);   // snapshot before the tracker consumes/associates them
            tracker.update(dets);

            const auto truth = world.snapshot();   // "perfect localization" — see README Day 6
            picture.publish({.ego = truth.ego, .ego_speed = truth.ego_speed,
                             .tracks = tracker.confirmed(), .stamp = now});
            ++stats.fusion_cycles;
        }
    });

    // 6. Planner: 10 Hz.
    threads.emplace_back([&](std::stop_token st) {
        start_gate.arrive_and_wait();
        adas::conc::run_at_rate(st, kPlanPeriod, [&](double) {
            if (picture.version() == 0) return;   // nothing fused yet — lock-free check
            path.set(planner->plan(picture.read(), world.course()));
            ++stats.plans;
        });
    });

    // 7. Control: 50 Hz.
    threads.emplace_back([&](std::stop_token st) {
        start_gate.arrive_and_wait();
        adas::conc::run_at_rate(st, kControlPeriod, [&](double dt) {
            if (picture.version() == 0) return;
            const auto pic = picture.read();
            command.set(controller.compute(pic.ego, pic.ego_speed, path.get(), dt));
            ++stats.control_ticks;
        });
    });

    // 8. Render: 12 Hz, reads everything, writes nothing (except stdout).
    threads.emplace_back([&](std::stop_token st) {
        start_gate.arrive_and_wait();
        std::cout << "\x1b[2J";   // clear once
        adas::conc::run_at_rate(st, kRenderPeriod, [&](double) {
            const int left = static_cast<int>(std::chrono::duration_cast<std::chrono::seconds>(deadline - Clock::now()).count());
            std::cout << renderer.draw(world.snapshot(), picture.read(), path.get(), world.course(),
                                       stats, planner->name(), std::max(left, 0), raw_dets.read(),
                                       command.get())
                      << std::flush;
            ++stats.render_frames;
        });
    });

    adas::conc::EventLog::instance().post(std::format("Started: {} s, seed {}, {} threads", args.seconds, args.seed, n_threads));

    // ---- Main thread: referee ---------------------------------------------
    while (!g_interrupted && !game_over && Clock::now() < deadline) std::this_thread::sleep_for(50ms);

    for (auto& t : threads) t.request_stop();   // cooperative stop via stop_token
    queue.close();
    threads.clear();                            // jthread joins in its destructor

    // ---- Post-race summary -------------------------------------------------
    const auto final = world.snapshot();
    std::cout << std::format("\n\n{}\n", std::string(60, '='));
    if (final.crashed)          std::cout << std::format("  GAME OVER — {} after {:.1f} s\n", final.crash_reason, final.sim_time);
    else if (g_interrupted)     std::cout << "  Interrupted.\n";
    else                        std::cout << std::format("  Time's up! Survived {:.1f} s\n", final.sim_time);
    std::cout << std::format("  Laps: {}   Fusion cycles: {}   Plans: {}   Sim ticks: {}\n",
                             final.laps, stats.fusion_cycles.load(), stats.plans.load(), stats.sim_ticks.load());
    std::cout << std::format("{}\n", std::string(60, '='));
    return final.crashed ? 1 : 0;
}
