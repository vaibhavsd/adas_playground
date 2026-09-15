#pragma once
// ============================================================================
//  DAY 9 — render.hpp
//  "Seeing is debugging."
//
//  DETAILED EXPLANATION:
//  The renderer is the visualization system—it draws the ADAS simulation to
//  the terminal in real time AND optionally records JSON for playback in a
//  web viewer. Its only job is display; it doesn't affect control or physics.
//
//  RENDERING TECHNIQUE:
//    Standard terminals display one character per cell. By using the Unicode
//    half-block character "▀" (U+2580) with separate foreground and background
//    colors, each cell displays TWO pixels (top and bottom). This gives a
//    96-wide terminal 96×68 effective pixels—enough for a legible track view.
//
//  KEY COMPONENTS:
//    • RGB: 8-bit RGB color struct; mix() blends two colors
//    • Renderer: Main drawing engine
//        - Constructor: Size in cells, world bounds (min/max coordinate)
//        - draw(): Compose one frame (called every ~33ms)
//        - Pixel buffer: fb_ (frame buffer), bg_ (cached track background)
//    • Geometric primitives:
//        - plot<Blend>(): Single pixel (blended or opaque)
//        - disc(): Filled circle at a position
//        - ring(): Hollow ring (used for track detections)
//        - car(): Oriented rectangle with nose pointing in heading direction
//        - headlights(): Decorative light cone ahead of ego
//    • UI elements:
//        - header(): Statistics bar (speed, clearance, frame counts, laps)
//        - footer(): Tracked objects and event log tail
//        - bar(): Progress bars with color coding
//
//  DRAWING ORDER (each frame):
//    1. Build background (static track) if needed (lazy)
//    2. Copy bg_ to fb_ (one vector copy; fast)
//    3. Draw planned path (green fading ribbon)
//    4. Draw fusion tracks (hollow rings)
//    5. Draw ground-truth traffic (colored cars)
//    6. Draw ego vehicle (cyan with windscreen)
//    7. Overlay crash tint if crashed
//    8. Compose text UI
//    9. (Optional) Record JSON to file
//
//  OPTIMIZATION: Lazy background caching
//    The track never changes, so bg_ is built once and cached. Every frame
//    copies it to fb_ (O(pixels) but very fast), then draws dynamic objects
//    on top. Without this, every frame would recompute the road texturing.
//
//  ESCAPE CODE OPTIMIZATION:
//    Instead of a full ansi clear (slow), the renderer uses ANSI escape codes
//    (Cursor Home, color changes only when needed). This reduces data from
//    ~150 KB per frame to ~15 KB—important for smooth scrollback.
//
//  OPTIONAL JSON RECORDING:
//    If ADAS_RECORD=/path/to/file.jsonl is set:
//      • Every frame appends one JSON object per line (jsonl format)
//      • Objects contain: time, laps, crashed, ego pose/speed, obstacles,
//        tracks, planned path, cost
//      • Can be loaded into viewer.html for frame-by-frame playback
//      • No changes to main.cpp needed (transparent to the rest of the stack)
//
//  PALETTE:
//    Color constants for grass, asphalt, lane markings, vehicles, etc.
//    Chosen for good contrast and visibility in both light/dark terminals.
//
//  USAGE IN PIPELINE:
//    • Each cycle, render thread calls renderer.draw(world.snapshot(), ...)
//    • Outputs ANSI escape codes to terminal (via stdout)
//    • Optionally writes JSON to ADAS_RECORD file
//    • Cycles at ~30 Hz for smooth animation
//
//  What you learn today
//    • C++20 std::format (goodbye printf, goodbye iostream manipulators)
//    • C++17 `if constexpr` to branch on a compile-time bool inside a template
//    • `mutable` + lazy caching inside a const member function — the track
//      never moves, so its pixels are computed once and memcpy'd every frame
//    • std::getenv, std::ofstream, std::optional
// ============================================================================
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <format>
#include <fstream>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "concurrency.hpp"
#include "planner.hpp"
#include "sensors.hpp"
#include "tracker.hpp"
#include "world.hpp"

namespace adas::ui {

using geo::Vec2;

// ---------------------------------------------------------------------------
// Colour
// ---------------------------------------------------------------------------
struct RGB {
    std::uint8_t r{0}, g{0}, b{0};
    constexpr bool operator==(const RGB&) const = default;   // C++20 again
};

[[nodiscard]] inline RGB mix(RGB a, RGB b, double t) noexcept {
    t = std::clamp(t, 0.0, 1.0);
    const auto ch = [t](std::uint8_t x, std::uint8_t y) {
        return static_cast<std::uint8_t>(x + (y - x) * t);
    };
    return {ch(a.r, b.r), ch(a.g, b.g), ch(a.b, b.b)};
}

namespace palette {
inline constexpr RGB grass_dark {  22,  42,  28 };
inline constexpr RGB grass_light{  32,  58,  38 };
inline constexpr RGB asphalt    {  48,  50,  56 };
inline constexpr RGB asphalt_alt{  54,  56,  63 };
inline constexpr RGB kerb_red   { 178,  52,  52 };
inline constexpr RGB kerb_white { 226, 226, 230 };
inline constexpr RGB lane_yellow{ 166, 140,  66 };   // muted, so track markers stand out
inline constexpr RGB ego_body   {  64, 214, 236 };
inline constexpr RGB ego_glass  { 232, 252, 255 };
inline constexpr RGB headlight  { 255, 236, 170 };
inline constexpr RGB track_ring { 246, 210,  74 };
inline constexpr RGB plan_near  { 116, 246, 140 };
inline constexpr RGB plan_far   {  40, 132,  84 };
inline constexpr RGB cone       { 244, 138,  40 };
inline constexpr RGB crash_tint { 200,  40,  40 };

// Traffic cars, cycled by obstacle id so a car keeps its colour all run.
inline constexpr std::array<RGB, 6> cars{{
    { 224,  86,  96 },   // red
    { 190, 118, 232 },   // violet
    { 238, 146,  62 },   // orange
    { 108, 176, 240 },   // blue
    { 236, 214,  92 },   // sand
    { 120, 218, 168 },   // mint
}};
}  // namespace palette

// ---------------------------------------------------------------------------
// Renderer
// ---------------------------------------------------------------------------
class Renderer {
public:
    // `height` is terminal ROWS; the pixel grid is twice as tall.
    Renderer(int width, int height, Vec2 world_min, Vec2 world_max)
        : w_(width), h_(height), ph_(height * 2), min_(world_min), max_(world_max),
          bg_(static_cast<std::size_t>(width * height * 2)),
          fb_(static_cast<std::size_t>(width * height * 2)) {}

    [[nodiscard]] std::string draw(const sim::World::Snapshot& truth, const fuse::FusedPicture& pic,
                                   const plan::Path& path, const sim::Course& course,
                                   const conc::Stats& stats, std::string_view planner_name,
                                   int seconds_left,
                                   const std::vector<sense::Detection>& dets = {},
                                   const sim::Control& cmd = {}) const {
        build_background_once(course);
        fb_ = bg_;   // one vector copy per frame; cheaper than recomputing the road

        // --- planned path: a fading green ribbon --------------------------
        for (std::size_t i = 0; i < path.points.size(); ++i) {
            const double t = static_cast<double>(i) / std::max<std::size_t>(path.points.size() - 1, 1);
            plot<true>(path.points[i], mix(palette::plan_near, palette::plan_far, t), 0.85);
        }

        // --- what fusion believes: hollow rings ---------------------------
        for (const auto& tr : pic.tracks) ring(tr.pos(), 1.6, palette::track_ring, 0.55);

        // --- ground truth traffic -----------------------------------------
        for (const auto& o : truth.obstacles) {
            if (o.radius < 0.8) {                      // a parked cone, not a car
                disc(o.pos, 0.9, palette::cone, 1.0);
                continue;
            }
            const RGB body = palette::cars[static_cast<std::size_t>(o.id) % palette::cars.size()];
            const double heading = o.vel.norm() > 0.4 ? o.vel.angle() : 0.0;
            car({.pos = o.pos, .yaw = heading}, body, false);
        }

        // --- ego ------------------------------------------------------------
        headlights(truth.ego);
        car(truth.ego, palette::ego_body, true);

        if (truth.crashed) tint(palette::crash_tint, 0.28);

        record_frame(truth, pic, path, dets, stats, planner_name, seconds_left, cmd);
        return compose(truth, pic, path, stats, planner_name, seconds_left);
    }

private:
    // ---- pixel plumbing --------------------------------------------------
    struct Px { int col, row; };

    [[nodiscard]] Px to_pixel(Vec2 p) const noexcept {
        return {static_cast<int>(std::lround((p.x - min_.x) / (max_.x - min_.x) * (w_ - 1))),
                static_cast<int>(std::lround((ph_ - 1) - (p.y - min_.y) / (max_.y - min_.y) * (ph_ - 1)))};
    }
    [[nodiscard]] Vec2 to_world(int col, int row) const noexcept {
        return {min_.x + static_cast<double>(col) / (w_ - 1) * (max_.x - min_.x),
                min_.y + static_cast<double>((ph_ - 1) - row) / (ph_ - 1) * (max_.y - min_.y)};
    }

    // `if constexpr` picks the blend path at compile time, so the opaque
    // version costs nothing at runtime — no branch, no unused argument read.
    template <bool Blend>
    void plot(Vec2 p, RGB c, [[maybe_unused]] double alpha = 1.0) const noexcept {
        const auto [col, row] = to_pixel(p);
        if (col < 0 || col >= w_ || row < 0 || row >= ph_) return;
        RGB& dst = fb_[static_cast<std::size_t>(row) * static_cast<std::size_t>(w_) + static_cast<std::size_t>(col)];
        if constexpr (Blend) dst = mix(dst, c, alpha);
        else                 dst = c;
    }

    void disc(Vec2 centre, double radius_m, RGB c, double alpha) const noexcept {
        for_each_in_box(centre, radius_m, [&](Vec2 p) {
            if (geo::distance(p, centre) <= radius_m) plot<true>(p, c, alpha);
        });
    }

    void ring(Vec2 centre, double radius_m, RGB c, double alpha) const noexcept {
        for_each_in_box(centre, radius_m + 0.6, [&](Vec2 p) {
            const double d = geo::distance(p, centre);
            if (d <= radius_m && d >= radius_m - 0.8) plot<true>(p, c, alpha);
        });
    }

    // An oriented rectangle with a windscreen and a bright nose. At ~1 m per
    // pixel a car is about 4x2 px — small, but the heading reads clearly.
    void car(const geo::Pose& pose, RGB body, bool is_ego) const noexcept {
        constexpr double kLen = 5.2, kWid = 2.6;
        const RGB outline = mix(body, RGB{0, 0, 0}, 0.55);
        const RGB glass   = is_ego ? palette::ego_glass : mix(body, RGB{255, 255, 255}, 0.55);
        for_each_in_box(pose.pos, kLen, [&](Vec2 p) {
            const Vec2 rel = geo::to_ego_frame(p, pose);
            if (std::abs(rel.x) > kLen / 2 || std::abs(rel.y) > kWid / 2) return;
            const bool edge = std::abs(rel.x) > kLen / 2 - 0.7 || std::abs(rel.y) > kWid / 2 - 0.6;
            const bool nose = rel.x > kLen / 2 - 1.4;
            plot<false>(p, nose ? glass : (edge ? outline : body));
        });
    }

    // A soft wedge of light ahead of ego. Pure decoration, and worth it.
    void headlights(const geo::Pose& ego) const noexcept {
        constexpr double kReach = 11.0, kHalfAngle = 0.42;
        for_each_in_box(ego.pos, kReach, [&](Vec2 p) {
            const Vec2 rel = geo::to_ego_frame(p, ego);
            const double d = rel.norm();
            if (rel.x <= 1.0 || d > kReach || std::abs(std::atan2(rel.y, rel.x)) > kHalfAngle) return;
            plot<true>(p, palette::headlight, 0.20 * (1.0 - d / kReach));
        });
    }

    void tint(RGB c, double alpha) const noexcept {
        for (auto& px : fb_) px = mix(px, c, alpha);
    }

    template <typename F>
    void for_each_in_box(Vec2 centre, double radius_m, F&& f) const noexcept {
        const auto [c0, r0] = to_pixel(centre);
        const double mx = (max_.x - min_.x) / (w_ - 1);
        const double my = (max_.y - min_.y) / (ph_ - 1);
        const int rc = static_cast<int>(std::ceil(radius_m / mx)) + 1;
        const int rr = static_cast<int>(std::ceil(radius_m / my)) + 1;
        for (int row = r0 - rr; row <= r0 + rr; ++row)
            for (int col = c0 - rc; col <= c0 + rc; ++col)
                if (col >= 0 && col < w_ && row >= 0 && row < ph_) f(to_world(col, row));
    }

    // ---- the static track, computed once ---------------------------------
    // `bg_` is mutable so this can happen inside a const draw(). The course
    // is immutable after construction (Day 4), so caching is safe.
    void build_background_once(const sim::Course& course) const {
        if (bg_ready_) return;
        const double hw = course.half_width();
        for (int row = 0; row < ph_; ++row) {
            for (int col = 0; col < w_; ++col) {
                const Vec2 p = to_world(col, row);
                const auto idx = course.nearest_index(p);
                const double lat = std::abs(course.lateral_offset(p));
                const bool stripe = (idx / 3) % 2 == 0;

                RGB c;
                if (lat <= hw) {
                    c = stripe ? palette::asphalt : palette::asphalt_alt;
                    if (lat < 0.55 && (idx / 4) % 2 == 0) c = palette::lane_yellow;   // dashed centre
                    else if (lat > hw - 0.7)              c = mix(c, palette::kerb_white, 0.45);
                } else if (lat <= hw + 1.1) {
                    c = stripe ? palette::kerb_red : palette::kerb_white;             // kerbs
                } else {
                    // Texture the grass from the PIXEL position, not the
                    // waypoint index. Using the index here drew a sunburst:
                    // every stripe radiated from the track centre, because
                    // nearest_index changes with angle, not with distance.
                    const bool checker = ((col / 3) + (row / 3)) % 2 == 0;
                    c = checker ? palette::grass_dark : palette::grass_light;
                }
                bg_[static_cast<std::size_t>(row) * static_cast<std::size_t>(w_) + static_cast<std::size_t>(col)] = c;
            }
        }
        bg_ready_ = true;
    }

    // ---- turning pixels into a string ------------------------------------
    [[nodiscard]] std::string compose(const sim::World::Snapshot& truth, const fuse::FusedPicture& pic,
                                      const plan::Path& path, const conc::Stats& stats,
                                      std::string_view planner_name, int seconds_left) const {
        std::string out;
        out.reserve(static_cast<std::size_t>(w_ * h_ * 14 + 2048));
        out += "\x1b[H";   // cursor home; no full clear, so no flicker

        out += header(truth, pic, path, stats, planner_name, seconds_left);

        // Emit escape codes only when a colour actually changes. Without this
        // a frame is ~150 kB; with it, roughly a tenth of that.
        std::optional<RGB> cur_fg, cur_bg;
        for (int row = 0; row < h_; ++row) {
            out += "  ";
            for (int col = 0; col < w_; ++col) {
                const RGB top = fb_[static_cast<std::size_t>(row * 2)     * static_cast<std::size_t>(w_) + static_cast<std::size_t>(col)];
                const RGB bot = fb_[static_cast<std::size_t>(row * 2 + 1) * static_cast<std::size_t>(w_) + static_cast<std::size_t>(col)];
                if (cur_fg != top) { out += std::format("\x1b[38;2;{};{};{}m", top.r, top.g, top.b); cur_fg = top; }
                if (cur_bg != bot) { out += std::format("\x1b[48;2;{};{};{}m", bot.r, bot.g, bot.b); cur_bg = bot; }
                out += "\u2580";   // ▀ upper half block
            }
            out += "\x1b[0m\n";
            cur_fg.reset();
            cur_bg.reset();
        }
        out += footer(pic);
        return out;
    }

    [[nodiscard]] std::string header(const sim::World::Snapshot& truth, const fuse::FusedPicture& pic,
                                     const plan::Path& path, const conc::Stats& stats,
                                     std::string_view planner_name, int seconds_left) const {
        const auto sf = [&](sense::SensorKind k) { return stats.sensor_frames[static_cast<std::size_t>(k)].load(); };
        const double clr = std::isfinite(path.min_clearance) ? path.min_clearance : 99.9;

        std::string s;
        s += std::format("\x1b[1;97m  ADAS PLAYGROUND\x1b[0m  t=\x1b[96m{:6.1f}\x1b[0ms  left=\x1b[96m{:3d}\x1b[0ms  "
                         "laps=\x1b[1;93m{}\x1b[0m  {}\x1b[K\n",
                         truth.sim_time, seconds_left, truth.laps,
                         truth.crashed ? std::format("\x1b[1;91m*** CRASHED: {} ***\x1b[0m", truth.crash_reason)
                                       : std::string{});
        s += std::format("  speed {} \x1b[97m{:4.1f}\x1b[0m m/s   clearance {} \x1b[97m{:4.1f}\x1b[0m m\x1b[K\n",
                         bar(truth.ego_speed / 12.0, 18, RGB{80, 210, 255}),
                         truth.ego_speed,
                         bar(std::min(clr, 6.0) / 6.0, 12, clr < 1.0 ? RGB{240, 90, 90} : RGB{120, 230, 140}),
                         clr);
        s += std::format("  \x1b[90mframes\x1b[0m radar={:<5} camera={:<5} lidar={:<5} \x1b[90m|\x1b[0m "
                         "fusion={:<5} plans={:<5} ctrl={:<5} sim={:<6}\x1b[K\n",
                         sf(sense::SensorKind::Radar), sf(sense::SensorKind::Camera), sf(sense::SensorKind::Lidar),
                         stats.fusion_cycles.load(), stats.plans.load(), stats.control_ticks.load(),
                         stats.sim_ticks.load());
        s += std::format("  \x1b[90m{}\x1b[0m offset={:+.1f}m target={:4.1f} m/s cost={:7.1f}  \x1b[90m|\x1b[0m "
                         "tracks={} truth={}   {} you  {} traffic  {} cone  {} plan  {} track\x1b[K\n",
                         planner_name, path.lateral_offset, path.target_speed, path.cost,
                         pic.tracks.size(), truth.obstacles.size(),
                         swatch(palette::ego_body), swatch(palette::cars[1]), swatch(palette::cone),
                         swatch(palette::plan_near), swatch(palette::track_ring));
        return s;
    }

    [[nodiscard]] std::string footer(const fuse::FusedPicture& pic) const {
        std::string s = "  \x1b[90mtracks:\x1b[0m ";
        for (const auto& t : pic.tracks)
            s += std::format("\x1b[93m[{} v={:4.1f} \u03c3={:.2f}]\x1b[0m ", t.id, t.vel().norm(), t.pos_sigma());
        s += "\x1b[K\n  \x1b[90m--- events ---\x1b[0m\x1b[K\n";
        for (const auto& e : conc::EventLog::instance().tail(4)) s += std::format("  \x1b[90m{}\x1b[0m\x1b[K\n", e);
        return s;
    }

    [[nodiscard]] static std::string bar(double frac, int width, RGB c) {
        frac = std::clamp(frac, 0.0, 1.0);
        const int filled = static_cast<int>(std::lround(frac * width));
        std::string s = std::format("\x1b[38;2;{};{};{}m", c.r, c.g, c.b);
        for (int i = 0; i < width; ++i) s += (i < filled) ? "\u2588" : "\u2591";
        s += "\x1b[0m";
        return s;
    }

    [[nodiscard]] static std::string swatch(RGB c) {
        return std::format("\x1b[38;2;{};{};{}m\u2588\u2588\x1b[0m", c.r, c.g, c.b);
    }

    // ---- optional recording for viewer.html ------------------------------
    // Schema (one JSON object per line) — deliberately mirrors *everything*
    // the terminal header/footer show, plus the raw material behind it, so
    // viewer.html's analysis panels never have to guess at a value the
    // terminal already had:
    //   t, laps, crashed, crash_reason, speed, ego[x,y,yaw], left            world/sim (World::Snapshot)
    //   obs[[id,x,y,vx,vy,radius], ...]                                     ground truth (sim::Obstacle)
    //   trk[[id,x,y,vx,vy,sigma,hits,misses,sensor_mask], ...]              fused/confirmed tracks (fuse::Track)
    //   det[[kind,x,y], ...]                                                raw per-sensor detections, 0=radar/1=camera/2=lidar (sense::Detection)
    //   path[[x,y], ...], cost, off, tspeed, minclr                         planner diagnostics (plan::Path)
    //   cmd[accel,steer]                                                    controller output this cycle (sim::Control)
    //   planner                                                             active planner's name()
    //   stats{radar,camera,lidar,fusion,plans,ctrl,sim}                     cumulative conc::Stats counters
    //   events[...]                                                        tail of conc::EventLog, newest last
    // See docs/tracker-explainer.html for what hits/misses/sensor_mask/sigma mean.
    void record_frame(const sim::World::Snapshot& truth, const fuse::FusedPicture& pic,
                      const plan::Path& path, const std::vector<sense::Detection>& dets,
                      const conc::Stats& stats, std::string_view planner_name,
                      int seconds_left, const sim::Control& cmd) const {
        if (!rec_checked_) {
            rec_checked_ = true;
            if (const char* p = std::getenv("ADAS_RECORD")) rec_.open(p, std::ios::trunc);
        }
        if (!rec_.is_open()) return;

        std::string s = std::format(R"({{"t":{:.2f},"laps":{},"crashed":{},"crash_reason":"{}","speed":{:.2f},)"
                                    R"("ego":[{:.2f},{:.2f},{:.3f}],"left":{},"obs":[)",
                                    truth.sim_time, truth.laps, truth.crashed ? "true" : "false",
                                    truth.crash_reason, truth.ego_speed, truth.ego.pos.x, truth.ego.pos.y,
                                    truth.ego.yaw, seconds_left);
        for (std::size_t i = 0; i < truth.obstacles.size(); ++i) {
            const auto& o = truth.obstacles[i];
            s += std::format("{}[{},{:.2f},{:.2f},{:.2f},{:.2f},{:.2f}]", i ? "," : "",
                             o.id, o.pos.x, o.pos.y, o.vel.x, o.vel.y, o.radius);
        }
        s += R"(],"trk":[)";
        for (std::size_t i = 0; i < pic.tracks.size(); ++i) {
            const auto& t = pic.tracks[i];
            s += std::format("{}[{},{:.2f},{:.2f},{:.2f},{:.2f},{:.2f},{},{},{}]", i ? "," : "",
                             t.id, t.pos().x, t.pos().y, t.vel().x, t.vel().y, t.pos_sigma(),
                             t.hits, t.misses, t.sensor_mask);
        }
        s += R"(],"det":[)";
        for (std::size_t i = 0; i < dets.size(); ++i) {
            const auto& d = dets[i];
            s += std::format("{}[{},{:.2f},{:.2f}]", i ? "," : "",
                             static_cast<int>(d.kind), d.pos_world.x, d.pos_world.y);
        }
        s += R"(],"path":[)";
        for (std::size_t i = 0; i < path.points.size(); ++i)
            s += std::format("{}[{:.2f},{:.2f}]", i ? "," : "", path.points[i].x, path.points[i].y);
        s += std::format(R"(],"cost":{:.1f},"off":{:.2f},"tspeed":{:.2f},"minclr":{:.2f},)"
                         R"("cmd":[{:.3f},{:.3f}],"planner":"{}",)"
                         R"("stats":{{"radar":{},"camera":{},"lidar":{},"fusion":{},"plans":{},"ctrl":{},"sim":{}}},)"
                         R"("events":[)",
                         path.cost, path.lateral_offset, path.target_speed,
                         std::isfinite(path.min_clearance) ? path.min_clearance : 99.9,
                         cmd.accel, cmd.steer, planner_name,
                         stats.sensor_frames[0].load(), stats.sensor_frames[1].load(), stats.sensor_frames[2].load(),
                         stats.fusion_cycles.load(), stats.plans.load(), stats.control_ticks.load(), stats.sim_ticks.load());
        const auto events = conc::EventLog::instance().tail(6);
        for (std::size_t k = 0; k < events.size(); ++k) {
            std::string esc;
            esc.reserve(events[k].size());
            for (char c : events[k]) { if (c == '"' || c == '\\') esc += '\\'; esc += c; }
            s += std::format("{}\"{}\"", k ? "," : "", esc);
        }
        s += "]}";
        rec_ << s << '\n';
    }

    int  w_, h_, ph_;
    Vec2 min_, max_;

    mutable std::vector<RGB> bg_;
    mutable std::vector<RGB> fb_;
    mutable bool             bg_ready_{false};
    mutable bool             rec_checked_{false};
    mutable std::ofstream    rec_;
};

}  // namespace adas::ui
