# ADAS Playground — learn modern C++ by building a tiny self-driving stack

A top-down ASCII "game": your car must lap an oval track while overtaking
slow traffic and dodging parked cones. The catch — ego can only see the world
through three noisy simulated sensors, and every decision is made by code
*you* will read, line by line, over ~12 days.

```
      sim ──► World ◄── snapshot() ◄── radar ─┐
       ▲                                camera ─┼─► BlockingQueue ─► fusion ─► Latest<FusedPicture>
       │                                 lidar ─┘                                   │
   Shared<Control> ◄── control ◄── Shared<Path> ◄── planner ◄──────────────────────┘
                                                       render ◄── (reads everything)
```

Eight threads, one ground truth, zero data races. ~1,400 lines of C++20,
no dependencies beyond the standard library.

Every file is one lesson. Every lesson opens with a banner that lists the
C++ features it teaches, and the code comments explain *why*, not just what.
Read them in order; the layering is deliberate (each header only includes
the ones before it).

---

## Build & run

```bash
# Option A: CMake
cmake -B build -DCMAKE_BUILD_TYPE=Release && cmake --build build -j
./build/adas_tests            # 7 unit tests, no framework
./build/adas_sim --seconds=60 --seed=42

# Option B: no CMake
./build.sh && ./adas_tests && ./adas_sim
```

Requirements: GCC 13+, Clang 17+, or Apple Clang 16+ (Xcode 16). The one
feature older compilers miss is `std::format`. On a Mac with an older Xcode:
`brew install gcc && CXX=g++-14 ./build.sh`.

The terminal view needs truecolor (any modern terminal: iTerm2, Terminal.app,
Windows Terminal, GNOME Terminal, Alacritty, VS Code). Make the window at
least **100 × 45** characters. `Ctrl-C` stops cleanly
(there is a lesson in how — Day 10).

Legend: `O` truth · `x` track · `+` planned path · `.` centreline · `:` road edge · `>`/`^`/`<`/`v` you.

Sanity check: `--seed=42 --seconds=60` should finish ~2 laps with no crash.
The exit code is 1 on a crash, so you can script experiments.

---

## The schedule

One header per day. Read the banner, read the code, do the "Try this",
move on. Days 1–4 are foundations, 5–8 are the ADAS stack, 9–11 wire it up
and prove it works, 12 is where you take over.

| Day | File | You build | C++ spotlight |
|-----|------|-----------|---------------|
| 1 | `geometry.hpp` | Vec2, Pose, Mat2, frame transforms | concepts, `<=>`, fold expressions, `inline constexpr`, `static_assert` tests |
| 2 | `vehicle.hpp` | Kinematic bicycle model | encapsulation, `explicit`, delegating constructors |
| 3 | `concurrency.hpp` | Queue, Latest, Shared, EventLog, rate loop | mutex/condvar/shared_mutex, atomics + memory order, `stop_token`, `source_location` |
| 4 | `world.hpp` | Course, traffic w/ ACC, ground truth | `shared_mutex` in practice, ranges projections, `std::span`, designated init |
| 5 | `sensors.hpp` | Radar, Camera, Lidar | abstract base, NVI pattern, rule of five, `variant`/`visit`, `optional` |
| 6 | `tracker.hpp` | Kalman filter + multi-object tracker | `erase_if`, `views::values`/`filter`, `popcount`, structured bindings |
| 7 | `planner.hpp` | Lattice planner w/ prediction | second abstract interface, `std::async`/`future` |
| 8 | `controller.hpp` | Pure pursuit + PID | concept-based static polymorphism, CTAD |
| 9 | `render.hpp` | Truecolor viewer, HUD, + browser replay | `std::format`, `if constexpr`, `mutable` caching |
| 10 | `main.cpp` | The launch file | `jthread`, `latch`, `consteval`/`constinit`, `from_chars`, `unique_ptr<Base>` |
| 11 | `tests/test_main.cpp` | Proof | testing threads, filters and planners with zero dependencies |
| 12 | — | Your turn | exercises, in ascending order of pain |

---

## Day 1 — `geometry.hpp` · "Every ADAS stack is secretly a geometry library"

**Read for:** how a `concept` replaces "SFINAE soup"; why `Vec2` has no
constructor (aggregates get designated initializers for free); what one
`operator<=>` line buys you; why `norm2()` is `constexpr` but `norm()` isn't
(`std::sqrt` isn't constexpr until C++26).

**The trick worth stealing:** the `static_assert`s at the bottom are unit
tests that run *inside the compiler*. If someone breaks `cross()`, the build
fails. Free, instant, unforgettable.

**Try this**
- 1.1 Add `static_assert(geo::deg2rad(180) == geo::kPi)`. Does it hold? Why not exactly? (Floating point; use `std::abs(...) < 1e-12` — and note you *can* do that in a static_assert because `std::abs` is constexpr for doubles since C++23 only… so write your own `cabs`.)
- 1.2 Change `deg2rad`'s constraint from `Arithmetic` to `std::floating_point` and call it with `35`. Read the compiler's message. That message is why concepts exist.
- 1.3 `distance_to_segment` was added *after* the first collision bug (Day 7 tells the story). Write a `static_assert` for the case where `p` projects beyond `b`.

**ADAS link:** `to_ego_frame` / `to_world_frame` are the two functions you
will write in every job you ever have. Bearing conventions kill more sensor
integrations than bad hardware.

**Deeper dive:** `docs/geometry.html` — worked examples (with real numbers)
for `to_ego_frame`/`to_world_frame` and `distance_to_segment`, plus every
call site across the codebase that uses them and why.

---

## Day 2 — `vehicle.hpp` · "The plant"

**Read for:** what "encapsulation" actually protects (the invariants
*inside* `step()` — clamped steer, clamped speed — live in exactly one
place); why `explicit` on a one-argument constructor is mandatory hygiene;
why there are two constructors (the comment explains a real GCC limitation
with `Params p = {}` on nested structs — you will hit this at work).

**Try this**
- 2.1 Add lateral-acceleration limiting: if `v²·tan(δ)/L` exceeds 4 m/s², reduce `δ`. Where should that live so no caller can bypass it?
- 2.2 Make `Vehicle` non-copyable. Watch `World`'s constructor break. Fix it with `std::move`. Now you understand why `World` takes `Vehicle` by value.

---

## Day 3 — `concurrency.hpp` · "Before the world, the phone lines"

This is the day. Read slowly. There are four idioms here and you will see
each one in every multi-threaded C++ codebase for the rest of your career:

1. **`BlockingQueue`** — mutex + condition_variable. Note that `notify_one()`
   is called *after* the lock is released. Note that `drain()` uses
   `unique_lock` (the condvar needs to unlock it) while `push()` uses
   `lock_guard` (nothing fancier needed). Note the predicate in `wait_for` —
   it handles spurious wakeups for you.
2. **`Latest<T>`** — `shared_mutex`: many readers or one writer. The
   `version_` atomic lets threads ask "has anything been published yet?"
   without touching the lock. Read the comment on `memory_order_release` /
   `acquire`; it's the one pair of memory orders you must actually understand.
3. **`EventLog`** — a Meyers singleton (function-local static: thread-safe
   init guaranteed by the language since C++11) and `std::source_location`
   replacing `__FILE__`/`__LINE__` macros.
4. **`run_at_rate`** — `sleep_until(next)` not `sleep_for(period)`. The
   difference is drift. Every real-time loop you write should look like this.
   The `std::invocable<double>` constraint gives readable errors if the body
   has the wrong signature.

**Try this**
- 3.1 Replace `scoped_lock` in `Latest::publish` with `lock_guard`. Compiles? (Yes — `scoped_lock` matters when you lock *two* mutexes at once; it orders them to avoid deadlock.)
- 3.2 Build with `-DADAS_TSAN=ON` (CMake) or `-fsanitize=thread`. Run the sim. It should be silent. Now delete the lock in `Shared::get()` and run again. ThreadSanitizer will name the exact two lines that race. This is the most valuable tool you're not using yet.
- 3.3 `Stats` uses `std::array<std::atomic<int>,3>`. Why can't it be a `std::vector<std::atomic<int>>` that you `push_back` into? (Atomics aren't copyable or movable.)

**Deeper dive:** `docs/concurrency.html` — every primitive with its real call
sites across the codebase, plus a worked timing trace and side-by-side
diagrams of why `sleep_until` doesn't drift and `sleep_for` does.

---

## Day 4 — `world.hpp` · "Ground truth"

**Read for:** the *only* writer is `step()`, which takes `unique_lock`;
every reader takes `shared_lock` and gets a *copy* (`Snapshot`). Returning
by value is what makes this safe — no reference escapes the lock.

`Course::nearest_index` uses `std::ranges::min_element(range, {}, projection)`.
The projection maps each point to the thing being compared. Compare it
mentally with the pre-C++20 lambda-comparator version: half the code, and
the intent is explicit.

The traffic has a toy ACC (`kFollowGap`, `kTrafficAccel`). It wasn't in the
first draft. The first draft's fastest car happily rear-ended ego every time
ego slowed for a jam. Then the fix made cars stop dead behind *cones*. Both
bugs are described in the comments — they're the shape of bugs you'll
actually meet in a simulator.

**Try this**
- 4.1 `Course::oval` spaces waypoints evenly in *angle*, so `spacing_` is an average and traffic speed varies around the loop. Replace it with an arc-length parameterization. (This is why real planners use `s` in metres.)
- 4.2 Add a `Course::figure8()`. The planner and controller should need zero changes — if they do, something leaked.

---

## Day 5 — `sensors.hpp` · "Three liars, one truth"

The OOP day. Three ideas:

- **Abstraction**: `Sensor` declares *what* a sensor does (`capture`) and leaves *how* (`measure`) pure virtual.
- **NVI pattern**: `capture()` is public and non-virtual; `measure()` is private and virtual. The base class controls the protocol (timestamp, ego pose), the derived class fills in physics, and the derived class *cannot even see* the code it isn't allowed to change.
- **Rule of five**: a polymorphic base gets a virtual destructor; here copies are deleted because an RNG + identity shouldn't be duplicated.

Then the C++17 half: measurements are a `std::variant`. `to_detection` uses
`std::visit` with the `overloaded` idiom. Add a fourth alternative to the
variant and the build breaks until you handle it. That is what "sum type"
means and it's a superpower for message-handling code.

Read the radar's Doppler comment. It says the velocity is radial-only and
that treating it as a full 2-D velocity is a simplification. Day 6 shows
how that simplification, left in, drove the car into a wall.

**Try this**
- 5.1 Add a `Sonar` sensor (5 m range, 360°, bearing only). You'll touch: the enum, the variant, `to_detection`, `Stats`'s array size, and `main`. Count how many places the compiler *forced* you to update vs. remembered on your own.
- 5.2 Add latency: `capture()` sets `stamp = now - 80ms` for the camera. Then look at Day 6 — nothing there uses per-detection stamps for prediction. That's a real gap; fixing it is Exercise 6.4.

---

## Day 6 — `tracker.hpp` · "Objects have memory"

**Read for:** a Kalman filter written out with 2×2 matrices — predict,
innovation, gain, covariance update. With a 2-state filter and scalar
measurements, `S` is a scalar and "matrix inverse" is a division. That is
why this fits in 40 lines with no Eigen.

Then the tracker: nearest-neighbour association with a gate that *scales
with the measurement's σ* (a camera guess 40 m out may be 6 m off; a lidar
centroid may not). Track confirmation requires hits from **two distinct
sensor types** (`std::popcount` on a bitmask) — that single rule killed all
the camera ghost tracks.

**The war story:** the first version applied radar Doppler as a 2-D velocity
update. Doppler is radial. In a corner, the line of sight and the car's
velocity aren't aligned, so the velocity *direction* was wrong by ~15°. Over
a 2 s prediction horizon that put the predicted car 0.9 m toward the wall,
the planner saw "room", and ego overtook into it. `use_doppler_updates`
is now `false`. Set it `true`, run `--seed=42`, and watch the crash at ~15 s.

**Try this**
- 6.1 Make `Kalman1D` a template on state size (start by writing `Kalman<2>`). You'll immediately want a real matrix type — that's the moment Eigen earns its place.
- 6.2 Replace nearest-neighbour with global nearest-neighbour (Hungarian). The `std::ranges::min_element` call is the seam.
- 6.3 The real fix for Doppler: a coupled 4-state (x, y, vx, vy) filter where the radar measurement is `H = [ux uy]` projected onto the line of sight. You now need a 4×4 and a proper inverse — do 6.1 first.
- 6.4 Predict each track to the *detection's* stamp, not "now". Then latency (5.2) is handled correctly.

---

## Day 7 — `planner.hpp` · "Where should I be in 3 seconds?"

A lattice planner: five candidate paths at lateral offsets, each scored
against predicted obstacle positions, cheapest wins. The scoring runs in
parallel with `std::async(std::launch::async, …)` — fork/join in three
lines. (Note `launch::async` — the default policy is allowed to run lazily
when you call `.get()`, which is a famous surprise.)

Two bugs live in the comments here, both worth reading twice:

1. **Aliasing.** Path points are 1.5 m apart; relative to a slower car the
   check advanced ~1 m per point; the collision window is ±0.9 m. The
   point-only check *stepped over* the collision. Fix: distance to the
   **segment** (`distance_to_segment`, Day 1).
2. **Constant-velocity prediction on a bend.** The car ahead was on a
   14 m-radius corner. CV extrapolation leaves the curve along the tangent and
   under-predicts how far the car swings inward by ~1.3 m over 2 s. Fix:
   predict *along the road* (`road_aware_prediction`). Flip it to `false`
   and ego crashes at the same corner, every seed, at t ≈ 15 s.

That toggle is the most instructive line in the project: it's the difference
between a physics-only predictor and a map-aware one, which is roughly the
difference between 2012 and 2018 in AV prediction.

**Try this**
- 7.1 Add hysteresis: penalize changing `lateral_offset` from last cycle. Watch the `+`/`-` flip-flopping on the HUD disappear.
- 7.2 Replace `std::async` with a `std::vector<std::jthread>` and a results vector. Which is cleaner? Which is safer if `build_and_score` throws?
- 7.3 Write a second `Planner` (e.g. pure "follow the leader" ACC). `main` should only change the `make_unique` line.

---

## Day 8 — `controller.hpp` · "Your home turf"

You know pure pursuit and PID. Today is about *how C++ expresses the design*:

```cpp
template <typename C>
concept LateralController = requires(const C c, const Pose& ego, double v, std::span<const Vec2> path) {
    { c.steer(ego, v, path) } -> std::convertible_to<double>;
};
```

`PurePursuit` satisfies this with no base class, no `virtual`, no vtable.
`VehicleController<Lat>` accepts anything that satisfies it. Compare with
`Sensor` (Day 5): same idea — "things that can do X" — but resolved at
compile time. When do you pick which?
- Set of types known at compile time, hot path → concept.
- Types chosen at runtime (config file, plugin) → virtual.

Also note CTAD: `VehicleController controller{PurePursuit{…}, PID{…}}` — no
template arguments written.

**Try this**
- 8.1 Write `Stanley` and swap it in. One line in `main`.
- 8.2 Break the concept: rename `steer` to `compute_steer` in `PurePursuit`. Read the error. It names the requirement that failed.
- 8.3 Feed-forward: add curvature of the planned path to the steer command. You'll need curvature from three consecutive points — `std::span` makes the windowing trivial.

---

## Day 9 — `render.hpp` · "Seeing is debugging"

The only file that exists purely to look good, and therefore the only one you
can rewrite freely without breaking anything. It produces two outputs.

**1. The terminal view.** Each character cell is the half-block glyph `▀`
with both a foreground and a background colour, so one cell holds *two*
square pixels — a 96x34 terminal becomes a 96x68 pixel display. Cars are
oriented rectangles with a windscreen and a headlight wedge, the track has
kerbs and dashed lane markings, and ground truth is drawn in each car's own
colour while fusion's belief is a yellow ring sized by the track's σ. When
the two drift apart, you can see it.

Read for:
- `std::format` throughout — `{:6.1f}`, `{:<5}`, and truecolor escapes built with `\x1b[38;2;{};{};{}m`.
- `if constexpr` in `plot<Blend>()`: the opaque path compiles to a plain store, with no branch and no unused alpha read.
- `mutable` + lazy caching: the track never moves, so `build_background_once` computes its pixels one time and every frame starts as a vector copy. This happens inside a `const` member function, which is exactly what `mutable` is for (you met it on Day 3 guarding a mutex).
- Colour escapes are only emitted when a colour actually changes. Without that a frame is ~150 kB; with it, about a tenth of that.

**Two bugs you can see in the pixels.** The grass texture was originally keyed
off `nearest_index`, which drew a sunburst radiating from the track centre —
because the index changes with *angle*, not with distance. It now keys off
pixel position. And the centreline dashes were the same yellow as the track
markers, so fusion's output was camouflaged against the road. Both comments
are in the file.

**2. The browser viewer** (`viewer.html`, Day 9½ — optional).

```bash
ADAS_RECORD=run.jsonl ./adas_sim --seconds=60 --seed=42
open viewer.html          # then drop run.jsonl onto the page
```

Setting `ADAS_RECORD` makes the renderer append one line of JSON per frame.
Nothing in `main.cpp` changes; the recorder lives in `record_frame()` at the
bottom of `render.hpp` and does nothing when the variable is unset.

`viewer.html` is a single standalone file — vanilla JS, canvas, no build step,
no dependencies, no server. It gives you a proper window with a scrubber,
frame stepping (arrow keys), play/pause (space), and a stats panel.

Scrubbing is the point. A live terminal view shows you *that* the car
crashed; stepping backward frame by frame shows you *why*. Every bug
documented in this project was found that way. Try it on the Day 7 story:
set `road_aware_prediction = false`, record a run, then step through the two
seconds before impact and watch the predicted position peel off the curve.

**Try this**
- 9.1 Draw each track's recent history as a fading trail. In the viewer that's ten lines; in the terminal you'll want a small ring buffer.
- 9.2 Colour each fused track by which sensors have hit it (you already have `sensor_mask` from Day 6). Camera-only tracks will light up in exactly the places you'd expect.
- 9.3 Plot the *residual* — the line between each track and its nearest ground-truth obstacle. That picture is your tracker's error, live.

## Day 10 — `main.cpp` · "The launch file"

**Read for:**
- `std::jthread` + `std::stop_token`: the thread body loops on
  `!st.stop_requested()`; `request_stop()` ends it; the destructor joins.
  No flags, no `join()` you can forget.
- `std::latch start_gate{n}`: every thread does `arrive_and_wait()` so
  nobody runs until everybody exists. Without it the first frames are garbage.
- `consteval hz_to_period(int)`: `hz_to_period(0)` is a *compile error*.
- `constinit` on the SIGINT flag: no static-init-order fiasco, still mutable.
- `picture.version() == 0` guard: a lock-free "has fusion published yet?"
  check. Before it existed the controller steered toward the origin for 100 ms.
- The fusion thread is *event-driven* (blocks on the queue); everything else
  is *rate-driven* (`run_at_rate`). Real stacks mix both exactly like this.

The one cheat: fusion reads the ego pose from `world.snapshot()`. That's
"perfect localization". Exercise 12.2 removes it.

**Try this**
- 10.1 Set `kPlanPeriod = hz_to_period(2)`. Watch the car struggle. Now you have intuition for planner latency.
- 10.2 Remove the `latch`. What breaks and why is it intermittent?
- 10.3 Run under `perf` or Instruments. Where is the time? (Hint: `world.snapshot()` copies a vector 60 times a second. Is that a problem? Measure, don't guess.)

---

## Day 11 — `tests/test_main.cpp` · "Trust, but verify"

Seven tests, no framework: geometry, course sign conventions, Kalman
convergence, tracker lifecycle (tentative → confirmed → dead, including the
two-sensor rule), pure pursuit sign, planner-swerves-around-obstacle, and a
multi-producer queue test using `jthread`.

Notice `test_course` failed on first run — I had asserted the wrong nearest
waypoint. The test was right and I was wrong. That's the point.

**Try this**
- 11.1 Add a regression test for the aliasing bug: obstacle that a point-only check would miss. It should fail if you swap `distance_to_segment` back to `distance`.
- 11.2 Add a "deterministic sim" test: run the whole pipeline single-threaded (see the harness idea in Day 7's story) for 15 s with seed 42 and assert no crash. Then it's a CI gate.

---

## Day 12 — Your turn

Ascending order of pain.

1. **Difficulty knob.** Add `--traffic=N`. Find the N where seed 42 stops surviving. That's your benchmark; every change below should move it.
2. **Localization.** Replace the `world.snapshot()` cheat in fusion with a dead-reckoning estimate (integrate the last `Control`), then fuse a noisy "GPS" detection of ego at 1 Hz. Same Kalman1D, new customer.
3. **Rear-facing radar.** Currently ego is blind behind. Add one, and watch the track count on the HUD change.
4. **Speed planner.** Today `target_speed` is a lerp on clearance. Make it a proper 1-D longitudinal plan: time-to-collision with the leader, comfortable decel. Print TTC on the HUD.
5. **MPC.** Replace pure pursuit with a small MPC on the bicycle model (you already have the model in `vehicle.hpp` — that's not an accident). This is where the concept-based `LateralController` design pays off: `main` doesn't change.
6. **Split the headers.** Everything is header-only for reading convenience. Split `tracker.hpp` into `.hpp`/`.cpp`, add `adas` as a real static library in CMake. Now you know why `inline` was on every free function.

---

## Feature index — where to find each C++17/20 feature

**C++17**
- Nested namespaces `adas::geo` — every header
- Structured bindings `auto& [id, t]` — `tracker.hpp` update loop
- `if` with initializer — `concurrency.hpp` `EventLog::post`
- `if constexpr` — `render.hpp` `put()`
- Fold expressions — `geometry.hpp` `sum_of_squares`
- `inline constexpr` variables — `geometry.hpp`, `main.cpp`
- `[[nodiscard]]` — everywhere a return value shouldn't be dropped
- CTAD — `std::lock_guard lk(m_)`, `VehicleController controller{…}`
- `std::optional` — `Detection::vel_world`
- `std::variant` / `std::visit` / `overloaded` — `sensors.hpp`
- `std::string_view` — sensor names, crash reasons, arg parsing
- `std::from_chars` — `main.cpp` `parse_args`
- `std::scoped_lock`, `std::shared_mutex` — `concurrency.hpp`
- `std::clamp` — `vehicle.hpp`, `controller.hpp`

**C++20**
- Concepts (`Arithmetic`, `LateralController`, `std::invocable`) — `geometry.hpp`, `controller.hpp`, `concurrency.hpp`
- `operator<=>` = default — `Vec2`
- Designated initializers — `world.hpp`, `sensors.hpp`, `main.cpp`
- Ranges: `views::iota`, `views::values`, `views::filter`, `ranges::min_element` w/ projection, `ranges::sort` w/ member pointer, `ranges::find_if`, `ranges::transform` — `world.hpp`, `tracker.hpp`, `planner.hpp`, `controller.hpp`, `main.cpp`
- `std::span` — `Course::points`, `Tracker::update`, `PurePursuit::steer`
- `std::format` — `render.hpp`, `EventLog`, `main.cpp`
- `std::jthread` / `std::stop_token` — `main.cpp`, `run_at_rate`
- `std::latch` — `main.cpp`
- `std::source_location` — `EventLog::post`
- `consteval` / `constinit` — `main.cpp`
- `std::erase_if` on a map, `unordered_set::contains`, `string_view::starts_with` — `tracker.hpp`, `main.cpp`
- `std::numbers::pi`, `std::lerp` — `geometry.hpp`, `world.hpp`, `planner.hpp`
- `std::popcount` — `tracker.hpp`
- `using enum` — `sensors.hpp` `to_string`
- Chrono literals `20ms` — `main.cpp`, tests

**Threading primitives**
`mutex`, `lock_guard`, `unique_lock`, `scoped_lock`, `shared_mutex`,
`shared_lock`, `condition_variable`, `atomic` (+ `memory_order`), `jthread`,
`stop_token`, `latch`, `async`/`future`, `this_thread::sleep_until`.

---

## Layout

```
include/adas/     one header per day (1–9), each depends only on earlier days
src/main.cpp      Day 10
tests/            Day 11
CMakeLists.txt    -DADAS_TSAN=ON builds with ThreadSanitizer
build.sh          no-CMake fallback
viewer.html       optional browser replay viewer (no build, no dependencies)
```

Have fun. When it crashes, read the events panel first — the code tells
you which file and line saw it happen.
