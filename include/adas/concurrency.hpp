#pragma once
// ============================================================================
//  DAY 3 — concurrency.hpp
//  "Before we build the world, we build the phone lines."
//
//  DETAILED EXPLANATION:
//  This module is the "middleware" layer that enables safe multi-threaded
//  operation. In a real ADAS system, components would talk via ROS2/DDS/Autosar.
//  Here, we provide thread-safe communication primitives and utilities to safely
//  pass data between independent processing loops (sensors, fusion, planning,
//  control, world simulation, rendering).
//
//  KEY COMPONENTS:
//    • BlockingQueue<T>: Many-to-one channel. Producers push(); consumer drains()
//      all pending items at once (efficient for batch consumers like fusion).
//    • Latest<T>: One-to-many publish-subscribe. Writer calls publish(); many
//      readers call read() to get the latest snapshot. Old values are discarded.
//    • Shared<T>: Simple mutex-protected box. Use when you need a lock and nothing fancier.
//    • Clock: Monotonic (steady) clock for measuring elapsed time without jumps.
//    • EventLog: Thread-safe singleton logger. Posts tagged with file:line via std::source_location.
//    • Stats: Lock-free atomic counters for frame/cycle statistics.
//    • run_at_rate(): Fixed-rate loop using stop_token. Body runs at exact average
//      rate despite variable computation times (sleep_until prevents drift).
//
//  THREADING PATTERN:
//    Each major component (sensor thread, fusion thread, planner thread, etc.)
//    runs its own event loop via run_at_rate(). They communicate via Latest<>
//    for "latest state" channels and BlockingQueue<> for "batch" channels.
//
//  SYNCHRONIZATION STRATEGY:
//    • Read-heavy data (e.g., world snapshot): Latest<> with shared_mutex
//    • Write-once-read-many (e.g., sensor frames): BlockingQueue<>
//    • Rare updates (e.g., state): Shared<>
//
//  USAGE IN PIPELINE:
//    • World publishes Snapshot to Latest<>; sensors/tracker read it
//    • Sensors push DetectionFrames into BlockingQueue<>
//    • Fusion pulls BatchedDetections, publishes FusedPicture to Latest<>
//    • Planner reads FusedPicture, publishes Path to Latest<>
//    • Controller reads Path, publishes Control to World
//
//  What you learn today
//    • std::mutex, std::lock_guard, std::scoped_lock, std::unique_lock,
//      std::shared_mutex + std::shared_lock, std::condition_variable
//    • std::atomic with memory orders, C++20 std::stop_token
//    • C++17 if-with-initializer, C++20 std::source_location, std::format,
//      std::invocable concept, chrono literals, Meyers singleton
// ============================================================================
#include <array>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <format>
#include <mutex>
#include <shared_mutex>
#include <source_location>
#include <stop_token>
#include <string>
#include <string_view>
#include <thread>
#include <utility>
#include <vector>

namespace adas::conc {

using Clock = std::chrono::steady_clock;  // monotonic; never use system_clock for dt

// ---------------------------------------------------------------------------
// BlockingQueue — producers push, one consumer drains everything at once.
//
// Why drain-all instead of pop-one? Because the fusion loop wants *all*
// measurements that arrived since last cycle; it is a batch consumer.
//
// The pattern: lock → mutate → unlock → notify. Notifying outside the lock
// avoids waking the consumer only for it to block on the mutex we still hold.
// ---------------------------------------------------------------------------
template <typename T>
class BlockingQueue {
public:
    void push(T item) {
        {
            std::lock_guard lk(m_);   // C++17 CTAD: no need for <std::mutex>
            items_.push_back(std::move(item));
        }
        cv_.notify_one();
    }

    // Blocks until at least one item exists or `timeout` elapses. Returns
    // whatever is there (possibly empty). unique_lock is required because the
    // condition variable must be able to unlock/relock it while waiting.
    [[nodiscard]] std::vector<T> drain(std::chrono::milliseconds timeout) {
        std::unique_lock lk(m_);
        cv_.wait_for(lk, timeout, [&] { return !items_.empty() || closed_; });
        std::vector<T> out;
        out.swap(items_);   // O(1): steal the buffer, leave an empty one behind
        return out;
    }

    void close() {
        { std::lock_guard lk(m_); closed_ = true; }
        cv_.notify_all();
    }

private:
    std::mutex              m_;
    std::condition_variable cv_;
    std::vector<T>          items_;
    bool                    closed_{false};
};

// ---------------------------------------------------------------------------
// Latest<T> — "publish the newest value, readers copy it out".
//
// shared_mutex lets N readers proceed concurrently and only the writer takes
// the exclusive lock. `version_` is an atomic so cheap "has anything been
// published yet?" checks don't need a lock at all.
//
// memory_order_release on the writer + memory_order_acquire on the reader
// guarantees: if you see version N, you also see every write that happened
// before version N was stored. (Here the mutex already gives that guarantee,
// so this is belt-and-braces — but it is *the* idiom worth memorizing.)
// ---------------------------------------------------------------------------
template <typename T>
class Latest {
public:
    void publish(T value) {
        {
            std::scoped_lock lk(m_);     // scoped_lock: can lock several mutexes deadlock-free
            value_ = std::move(value);
        }
        version_.fetch_add(1, std::memory_order_release);
    }

    [[nodiscard]] T read() const {
        std::shared_lock lk(m_);         // many readers at once
        return value_;                   // copy out; the lock ends here
    }

    [[nodiscard]] std::uint64_t version() const noexcept {
        return version_.load(std::memory_order_acquire);
    }

private:
    mutable std::shared_mutex  m_;       // `mutable`: lockable inside const methods
    T                          value_{};
    std::atomic<std::uint64_t> version_{0};
};

// ---------------------------------------------------------------------------
// Shared<T> — when you just need a lock and nothing clever.
// ---------------------------------------------------------------------------
template <typename T>
class Shared {
public:
    void set(T v) { std::scoped_lock lk(m_); value_ = std::move(v); }
    [[nodiscard]] T get() const { std::scoped_lock lk(m_); return value_; }

private:
    mutable std::mutex m_;
    T                  value_{};
};

// ---------------------------------------------------------------------------
// EventLog — a Meyers singleton. Since C++11 the initialization of a
// function-local static is guaranteed thread-safe by the language.
//
// std::source_location (C++20) captures file/line *at the call site* as a
// default argument — no more __FILE__/__LINE__ macros.
// ---------------------------------------------------------------------------
class EventLog {
public:
    static EventLog& instance() {
        static EventLog log;
        return log;
    }

    void post(std::string msg,
              std::source_location loc = std::source_location::current()) {
        std::string_view file = loc.file_name();
        // C++17 if-with-initializer: `p` is scoped to the if statement.
        if (auto p = file.find_last_of("/\\"); p != std::string_view::npos) {
            file.remove_prefix(p + 1);
        }
        std::scoped_lock lk(m_);
        entries_.push_back(std::format("{:>14}:{:<4} {}", file, loc.line(), msg));
        while (entries_.size() > kMax) entries_.pop_front();
    }

    [[nodiscard]] std::vector<std::string> tail(std::size_t n) const {
        std::scoped_lock lk(m_);
        const auto start = entries_.size() > n ? entries_.size() - n : 0;
        return {entries_.begin() + static_cast<std::ptrdiff_t>(start), entries_.end()};
    }

    EventLog(const EventLog&)            = delete;   // singletons don't copy
    EventLog& operator=(const EventLog&) = delete;

private:
    EventLog() = default;
    static constexpr std::size_t kMax = 200;
    mutable std::mutex       m_;
    std::deque<std::string>  entries_;
};

// ---------------------------------------------------------------------------
// Stats — lock-free counters. std::atomic<int>::operator++ is atomic; no
// mutex needed for "how many frames did the radar produce?".
// ---------------------------------------------------------------------------
struct Stats {
    std::array<std::atomic<int>, 3> sensor_frames{};  // indexed by SensorKind
    std::atomic<int> fusion_cycles{0};
    std::atomic<int> plans{0};
    std::atomic<int> control_ticks{0};
    std::atomic<int> sim_ticks{0};
    std::atomic<int> render_frames{0};
};

// ---------------------------------------------------------------------------
// run_at_rate — the heartbeat of every thread in this project.
//
//   run_at_rate(stop_token, 10ms, [&](double dt){ ... });
//
// `sleep_until(next)` (instead of sleep_for(period)) keeps the *average* rate
// exact even if a body call runs long — the schedule never drifts.
// The concept std::invocable<double> rejects lambdas with the wrong signature
// at the call site, with a readable error.
// ---------------------------------------------------------------------------
template <std::invocable<double> Body>
void run_at_rate(std::stop_token st, std::chrono::milliseconds period, Body&& body) {
    auto next = Clock::now();
    auto last = next;
    bool first = true;
    while (!st.stop_requested()) {
        const auto now = Clock::now();
        const double dt = first ? std::chrono::duration<double>(period).count()
                                : std::chrono::duration<double>(now - last).count();
        first = false;
        last = now;
        body(dt);
        next += period;
        std::this_thread::sleep_until(next);
    }
}

}  // namespace adas::conc
