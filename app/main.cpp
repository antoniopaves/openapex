// SPDX-FileCopyrightText: 2026 OpenApex contributors
// SPDX-License-Identifier: GPL-3.0-or-later
#include "openapex/sim/track.hpp"
#include "openapex/sim/vehicle.hpp"

#include <SDL3/SDL.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <cstring>
#include <vector>

using namespace openapex::sim;

namespace {

constexpr double kSimDt = 1.0 / 360.0;
constexpr double kMaxFrameTime = 0.1;
constexpr double kGravity = 9.80665;
constexpr float kPiF = 3.14159265f;
constexpr std::size_t kHistory = 600;
constexpr std::size_t kTrail = 90;

struct Camera {
    double x = 0.0;
    double y = 0.0;
    float ppm = 12.0f; ///< píxeles por metro
};

SDL_FPoint to_screen(double wx, double wy, const Camera& cam, int w, int h) {
    return {static_cast<float>(w) * 0.5f + static_cast<float>(wx - cam.x) * cam.ppm,
            static_cast<float>(h) * 0.5f - static_cast<float>(wy - cam.y) * cam.ppm};
}

void body_to_world(const VehicleState& s, double bx, double by, double& wx, double& wy) {
    const double c = std::cos(s.yaw);
    const double sn = std::sin(s.yaw);
    wx = s.x + bx * c - by * sn;
    wy = s.y + bx * sn + by * c;
}

void line(SDL_Renderer* r, const Camera& cam, int w, int h, double x0, double y0, double x1,
          double y1) {
    const SDL_FPoint a = to_screen(x0, y0, cam, w, h);
    const SDL_FPoint b = to_screen(x1, y1, cam, w, h);
    SDL_RenderLine(r, a.x, a.y, b.x, b.y);
}

// ---------------------------------------------------------------- pista

// Carga una pista desde archivo, con fallback al demo si falla
Track load_track_or_demo(const std::string& path) {
    try {
        return Track::load_from_file(path);
    } catch (const std::exception& e) {
        std::fprintf(stderr, "Warning: Failed to load track from %s: %s\n", path.c_str(), e.what());
        std::fprintf(stderr, "Falling back to demo track\n");
        return Track::make_demo();
    }
}

struct TrackEdges {
    std::vector<Vec2> left;
    std::vector<Vec2> right;
    std::vector<Vec2> kerb_left;
    std::vector<Vec2> kerb_right;
};

TrackEdges build_edges(const Track& track) {
    TrackEdges e;
    const auto& c = track.centerline();
    const std::size_t n = c.size();
    for (std::size_t i = 0; i < n; ++i) {
        const double hw = 0.5 * track.width_at(i);
        const double kw = hw + track.kerb_width_at(i);
        const Vec2& prev = c[(i + n - 1) % n];
        const Vec2& next = c[(i + 1) % n];
        const double dx = next.x - prev.x;
        const double dy = next.y - prev.y;
        const double len = std::hypot(dx, dy);
        const double nx = -dy / len;
        const double ny = dx / len;
        e.left.push_back({c[i].x + nx * hw, c[i].y + ny * hw});
        e.right.push_back({c[i].x - nx * hw, c[i].y - ny * hw});
        e.kerb_left.push_back({c[i].x + nx * kw, c[i].y + ny * kw});
        e.kerb_right.push_back({c[i].x - nx * kw, c[i].y - ny * kw});
    }
    return e;
}

void draw_loop(SDL_Renderer* r, const Camera& cam, int w, int h, const std::vector<Vec2>& pts) {
    std::vector<SDL_FPoint> screen;
    screen.reserve(pts.size() + 1);
    for (const Vec2& p : pts) {
        screen.push_back(to_screen(p.x, p.y, cam, w, h));
    }
    screen.push_back(screen.front());
    SDL_RenderLines(r, screen.data(), static_cast<int>(screen.size()));
}

void draw_track(SDL_Renderer* r, const Camera& cam, int w, int h, const Track& track,
                const TrackEdges& e) {
    SDL_SetRenderDrawColor(r, 70, 74, 80, 255);
    draw_loop(r, cam, w, h, track.centerline());
    SDL_SetRenderDrawColor(r, 200, 60, 60, 255);
    draw_loop(r, cam, w, h, e.kerb_left);
    draw_loop(r, cam, w, h, e.kerb_right);
    SDL_SetRenderDrawColor(r, 225, 225, 225, 255);
    draw_loop(r, cam, w, h, e.left);
    draw_loop(r, cam, w, h, e.right);

    // Línea de meta.
    SDL_SetRenderDrawColor(r, 255, 255, 255, 255);
    line(r, cam, w, h, e.left[0].x, e.left[0].y, e.right[0].x, e.right[0].y);
}

void draw_grid(SDL_Renderer* r, const Camera& cam, int w, int h) {
    constexpr double step = 10.0;
    const double ppm = static_cast<double>(cam.ppm);
    const double half_w = static_cast<double>(w) / (2.0 * ppm) + step;
    const double half_h = static_cast<double>(h) / (2.0 * ppm) + step;
    SDL_SetRenderDrawColor(r, 34, 58, 38, 255);
    for (double gx = std::floor((cam.x - half_w) / step) * step; gx <= cam.x + half_w; gx += step) {
        line(r, cam, w, h, gx, cam.y - half_h, gx, cam.y + half_h);
    }
    for (double gy = std::floor((cam.y - half_h) / step) * step; gy <= cam.y + half_h; gy += step) {
        line(r, cam, w, h, cam.x - half_w, gy, cam.x + half_w, gy);
    }
}

// ---------------------------------------------------------------- coche

void draw_car(SDL_Renderer* r, const Camera& cam, int w, int h, const VehicleParams& p,
              const VehicleState& s) {
    const double front = p.cg_to_front + 0.4;
    const double rear = -p.cg_to_rear - 0.4;
    const double half = 0.5 * p.track_width + 0.2;
    const std::array<std::array<double, 2>, 4> corners{
        {{front, half}, {front, -half}, {rear, -half}, {rear, half}}};

    SDL_SetRenderDrawColor(r, 240, 240, 245, 255);
    for (std::size_t i = 0; i < corners.size(); ++i) {
        const auto& a = corners[i];
        const auto& b = corners[(i + 1) % corners.size()];
        double ax = 0.0, ay = 0.0, bx = 0.0, by = 0.0;
        body_to_world(s, a[0], a[1], ax, ay);
        body_to_world(s, b[0], b[1], bx, by);
        line(r, cam, w, h, ax, ay, bx, by);
    }

    const std::array<std::array<double, 3>, 4> wheels{{{p.cg_to_front, 0.5 * p.track_width, 1.0},
                                                       {p.cg_to_front, -0.5 * p.track_width, 1.0},
                                                       {-p.cg_to_rear, 0.5 * p.track_width, 0.0},
                                                       {-p.cg_to_rear, -0.5 * p.track_width, 0.0}}};
    for (std::size_t i = 0; i < wheels.size(); ++i) {
        const auto& wh = wheels[i];
        const bool is_front = wh[2] > 0.5;
        const double heading = s.yaw + (is_front ? s.steer_angle : 0.0);
        double cx = 0.0, cy = 0.0;
        body_to_world(s, wh[0], wh[1], cx, cy);
        const double dx = 0.35 * std::cos(heading);
        const double dy = 0.35 * std::sin(heading);
        const bool sliding =
            std::abs(s.wheel[i].slip_ratio) > 0.2 || std::abs(s.wheel[i].tan_alpha) > 0.3;
        if (sliding) {
            SDL_SetRenderDrawColor(r, 255, 70, 70, 255);
        } else if (is_front) {
            SDL_SetRenderDrawColor(r, 255, 200, 60, 255);
        } else {
            SDL_SetRenderDrawColor(r, 120, 200, 255, 255);
        }
        line(r, cam, w, h, cx - dx, cy - dy, cx + dx, cy + dy);
    }
}

std::array<Vec2, 4> wheel_positions(const VehicleParams& p, const VehicleState& s) {
    const double half = 0.5 * p.track_width;
    const std::array<std::array<double, 2>, 4> local{{{p.cg_to_front, half},
                                                      {p.cg_to_front, -half},
                                                      {-p.cg_to_rear, half},
                                                      {-p.cg_to_rear, -half}}};
    std::array<Vec2, 4> out{};
    for (std::size_t i = 0; i < out.size(); ++i) {
        body_to_world(s, local[i][0], local[i][1], out[i].x, out[i].y);
    }
    return out;
}

std::array<double, 4> wheel_grip(const Track& track, const std::array<Vec2, 4>& pos) {
    std::array<double, 4> g{};
    for (std::size_t i = 0; i < g.size(); ++i) {
        g[i] = surface_grip(track.query(pos[i].x, pos[i].y).surface);
    }
    return g;
}

void reset_car(VehicleState& s, const Track& track) {
    s = VehicleState{};
    const Vec2 p = track.start_position();
    s.x = p.x;
    s.y = p.y;
    s.yaw = track.start_heading();
}

// ---------------------------------------------------------------- telemetría

struct Sample {
    float speed;
    float throttle;
    float brake;
    float steer;
    float slip;
};

struct History {
    std::array<Sample, kHistory> data{};
    std::size_t head = 0;
    void push(const Sample& s) {
        data[head] = s;
        head = (head + 1) % kHistory;
    }
};

struct Trail {
    std::array<SDL_FPoint, kTrail> p{};
    std::size_t head = 0;
    void push(float lateral_g, float long_g) {
        p[head] = SDL_FPoint{lateral_g, long_g};
        head = (head + 1) % kTrail;
    }
};

float norm_speed(const Sample& s) { return s.speed / 250.0f; }
float norm_throttle(const Sample& s) { return s.throttle; }
float norm_brake(const Sample& s) { return s.brake; }
float norm_steer(const Sample& s) { return s.steer * 0.5f + 0.5f; }
float norm_slip(const Sample& s) { return s.slip + 0.5f; }

void draw_graph(SDL_Renderer* r, float x, float y, float width, float height,
                const History& hist) {
    const SDL_FRect box{x, y, width, height};
    SDL_SetRenderDrawColor(r, 14, 16, 20, 255);
    SDL_RenderFillRect(r, &box);
    SDL_SetRenderDrawColor(r, 90, 94, 104, 255);
    SDL_RenderRect(r, &box);

    using Getter = float (*)(const Sample&);
    struct Channel {
        Uint8 red;
        Uint8 green;
        Uint8 blue;
        Getter get;
        const char* label;
    };
    const std::array<Channel, 5> channels{{{80, 220, 255, norm_speed, "Vel"},
                                           {90, 230, 90, norm_throttle, "Gas"},
                                           {255, 90, 90, norm_brake, "Freno"},
                                           {255, 210, 70, norm_steer, "Volante"},
                                           {230, 110, 255, norm_slip, "Slip tras."}}};

    std::vector<SDL_FPoint> pts(kHistory);
    float label_x = x + 6.0f;
    for (const Channel& ch : channels) {
        SDL_SetRenderDrawColor(r, ch.red, ch.green, ch.blue, 255);
        for (std::size_t i = 0; i < kHistory; ++i) {
            const Sample& s = hist.data[(hist.head + i) % kHistory];
            const float v = std::clamp(ch.get(s), 0.0f, 1.0f);
            pts[i] = SDL_FPoint{x + width * static_cast<float>(i) / static_cast<float>(kHistory - 1),
                                y + height - v * height};
        }
        SDL_RenderLines(r, pts.data(), static_cast<int>(pts.size()));
        SDL_RenderDebugText(r, label_x, y - 12.0f, ch.label);
        label_x += 8.0f * (static_cast<float>(std::strlen(ch.label)) + 2.0f);
    }
}

void draw_gg(SDL_Renderer* r, float x, float y, float size, const Trail& trail) {
    const SDL_FRect box{x, y, size, size};
    SDL_SetRenderDrawColor(r, 14, 16, 20, 255);
    SDL_RenderFillRect(r, &box);
    SDL_SetRenderDrawColor(r, 90, 94, 104, 255);
    SDL_RenderRect(r, &box);

    const float cx = x + size * 0.5f;
    const float cy = y + size * 0.5f;
    const float scale = size * 0.5f / 2.0f; // el cuadro cubre +-2 g

    SDL_SetRenderDrawColor(r, 60, 64, 72, 255);
    SDL_RenderLine(r, x, cy, x + size, cy);
    SDL_RenderLine(r, cx, y, cx, y + size);

    std::array<SDL_FPoint, 49> circle{};
    for (std::size_t i = 0; i < circle.size(); ++i) {
        const float a = 2.0f * kPiF * static_cast<float>(i) / 48.0f;
        circle[i] = SDL_FPoint{cx + std::cos(a) * 1.5f * scale, cy + std::sin(a) * 1.5f * scale};
    }
    SDL_RenderLines(r, circle.data(), static_cast<int>(circle.size()));

    std::array<SDL_FPoint, kTrail> pts{};
    for (std::size_t i = 0; i < kTrail; ++i) {
        const SDL_FPoint& g = trail.p[(trail.head + i) % kTrail];
        pts[i] = SDL_FPoint{cx - g.x * scale, cy - g.y * scale};
    }
    SDL_SetRenderDrawColor(r, 255, 200, 60, 255);
    SDL_RenderLines(r, pts.data(), static_cast<int>(pts.size()));
    const SDL_FRect dot{pts.back().x - 3.0f, pts.back().y - 3.0f, 6.0f, 6.0f};
    SDL_RenderFillRect(r, &dot);
}

// ---------------------------------------------------------------- utilidades

double approach(double value, double target, double rate, double dt) {
    const double delta = rate * dt;
    if (value < target) return std::min(value + delta, target);
    return std::max(value - delta, target);
}

void format_time(char* out, std::size_t n, double t) {
    if (t < 0.0) {
        std::snprintf(out, n, "--:--.---");
        return;
    }
    const int minutes = static_cast<int>(t / 60.0);
    std::snprintf(out, n, "%d:%06.3f", minutes, t - minutes * 60.0);
}

const char* surface_name(Surface s) {
    switch (s) {
    case Surface::Asphalt:
        return "Asfalto";
    case Surface::Kerb:
        return "Piano";
    case Surface::Grass:
        return "Hierba";
    }
    return "?";
}

} // namespace

int main() {
    if (!SDL_Init(SDL_INIT_VIDEO)) {
        std::fprintf(stderr, "SDL_Init fallo: %s\n", SDL_GetError());
        return 1;
    }
    SDL_Window* window = SDL_CreateWindow("OpenApex", 1280, 720, SDL_WINDOW_RESIZABLE);
    SDL_Renderer* renderer = window ? SDL_CreateRenderer(window, nullptr) : nullptr;
    if (!window || !renderer) {
        std::fprintf(stderr, "No se pudo crear ventana o renderer: %s\n", SDL_GetError());
        SDL_Quit();
        return 1;
    }
    SDL_SetRenderVSync(renderer, 1);

    // Cargar pista desde archivo (con fallback a demo)
    const Track track = load_track_or_demo("data/tracks/demo.track.json");
    const TrackEdges edges = build_edges(track);
    VehicleParams params;
    VehicleState state;
    VehicleInput input;
    Camera cam;
    History history;
    Trail trail;
    reset_car(state, track);

    std::array<double, 4> grip{1.0, 1.0, 1.0, 1.0};
    double lap_time = 0.0;
    double last_lap = -1.0;
    double best_lap = -1.0;
    int laps = 0;
    double prev_along = 0.0;

    bool running = true;
    double accumulator = 0.0;
    Uint64 previous = SDL_GetPerformanceCounter();
    const double frequency = static_cast<double>(SDL_GetPerformanceFrequency());

    while (running) {
        SDL_Event e;
        while (SDL_PollEvent(&e)) {
            if (e.type == SDL_EVENT_QUIT) running = false;
            if (e.type == SDL_EVENT_MOUSE_WHEEL) {
                cam.ppm = std::clamp(cam.ppm * std::pow(1.1f, e.wheel.y), 1.5f, 40.0f);
            }
            if (e.type == SDL_EVENT_KEY_DOWN && !e.key.repeat) {
                if (e.key.key == SDLK_ESCAPE) running = false;
                if (e.key.key == SDLK_R) {
                    reset_car(state, track);
                    input = VehicleInput{};
                    lap_time = 0.0;
                    prev_along = 0.0;
                }
                if (e.key.key == SDLK_E) shift_gear(params, state, +1);
                if (e.key.key == SDLK_Q) shift_gear(params, state, -1);
                if (e.key.key == SDLK_T) state.auto_shift = !state.auto_shift;
                // ABS/TC/LSD toggles
                if (e.key.key == SDLK_A) params.abs_enabled = !params.abs_enabled;
                if (e.key.key == SDLK_C) params.tc_enabled = !params.tc_enabled;
                if (e.key.key == SDLK_D) {
                    using DT = VehicleParams::DiffType;
                    params.diff_type = (params.diff_type == DT::Open) ? DT::Viscous :
                                       (params.diff_type == DT::Viscous) ? DT::Clutch :
                                       (params.diff_type == DT::Clutch) ? DT::Torsen : DT::Open;
                }
            }
        }

        const Uint64 now = SDL_GetPerformanceCounter();
        const double frame_time =
            std::min(static_cast<double>(now - previous) / frequency, kMaxFrameTime);
        previous = now;

        const bool* keys = SDL_GetKeyboardState(nullptr);
        const bool up = keys[SDL_SCANCODE_W] || keys[SDL_SCANCODE_UP];
        const bool down = keys[SDL_SCANCODE_S] || keys[SDL_SCANCODE_DOWN];
        const bool left = keys[SDL_SCANCODE_A] || keys[SDL_SCANCODE_LEFT];
        const bool right = keys[SDL_SCANCODE_D] || keys[SDL_SCANCODE_RIGHT];

        const double steer_target = (left ? 1.0 : 0.0) - (right ? 1.0 : 0.0);
        const double steer_rate = steer_target == 0.0 ? 5.0 : 3.0;
        input.steer = approach(input.steer, steer_target, steer_rate, frame_time);
        input.throttle = approach(input.throttle, up ? 1.0 : 0.0, up ? 3.0 : 8.0, frame_time);
        input.brake = approach(input.brake, down ? 1.0 : 0.0, down ? 5.0 : 10.0, frame_time);

        accumulator += frame_time;
        while (accumulator >= kSimDt) {
            grip = wheel_grip(track, wheel_positions(params, state));
            step_vehicle(params, state, input, kSimDt, grip);
            accumulator -= kSimDt;
            lap_time += kSimDt;
        }

        // Cronometraje: cruce de meta hacia delante, con un mínimo de 10 s por vuelta.
        const TrackQuery centre = track.query(state.x, state.y);
        if (prev_along > 0.75 * track.length() && centre.along < 0.25 * track.length() &&
            lap_time > 10.0) {
            last_lap = lap_time;
            if (best_lap < 0.0 || last_lap < best_lap) best_lap = last_lap;
            lap_time = 0.0;
            ++laps;
        }
        prev_along = centre.along;

        const double speed = std::hypot(state.vx, state.vy);
        const double rear_slip = 0.5 * (state.wheel[2].slip_ratio + state.wheel[3].slip_ratio);
        history.push({static_cast<float>(speed * 3.6), static_cast<float>(input.throttle),
                      static_cast<float>(input.brake), static_cast<float>(input.steer),
                      static_cast<float>(rear_slip)});
        trail.push(static_cast<float>(state.ay / kGravity), static_cast<float>(state.ax / kGravity));

        cam.x = state.x;
        cam.y = state.y;

        int w = 0;
        int h = 0;
        SDL_GetRenderOutputSize(renderer, &w, &h);
        SDL_SetRenderDrawColor(renderer, 24, 42, 28, 255);
        SDL_RenderClear(renderer);
        draw_grid(renderer, cam, w, h);
        draw_track(renderer, cam, w, h, track, edges);
        draw_car(renderer, cam, w, h, params, state);

        char gear_text[4];
        if (state.gear < 0) {
            std::snprintf(gear_text, sizeof gear_text, "R");
        } else if (state.gear == 0) {
            std::snprintf(gear_text, sizeof gear_text, "N");
        } else {
            std::snprintf(gear_text, sizeof gear_text, "%d", state.gear);
        }
        char cur[16], lst[16], bst[16];
        format_time(cur, sizeof cur, lap_time);
        format_time(lst, sizeof lst, last_lap);
        format_time(bst, sizeof bst, best_lap);

        SDL_SetRenderDrawColor(renderer, 240, 240, 240, 255);
        SDL_RenderDebugTextFormat(renderer, 10.0f, 10.0f, "%.0f km/h   %s", speed * 3.6,
                                  surface_name(centre.surface));
        SDL_RenderDebugTextFormat(renderer, 10.0f, 22.0f, "Marcha %s (%s)   %.0f rpm", gear_text,
                                  state.auto_shift ? "auto" : "manual", state.engine_rpm);
        SDL_RenderDebugTextFormat(renderer, 10.0f, 34.0f, "Long %.2f g   Lat %.2f g",
                                  state.ax / kGravity, state.ay / kGravity);
        SDL_RenderDebugTextFormat(renderer, 10.0f, 46.0f, "Volante %.1f grados   Slip tras. %.2f",
                                  state.steer_angle * 57.29578, rear_slip);
        SDL_RenderDebugTextFormat(renderer, 10.0f, 66.0f, "Vuelta %d   Actual %s", laps + 1, cur);
        SDL_RenderDebugTextFormat(renderer, 10.0f, 78.0f, "Ultima %s   Mejor %s", lst, bst);

        // ABS/TC/LSD telemetría
        const char* abs_str = params.abs_enabled ? "ON" : "OFF";
        const char* tc_str = params.tc_enabled ? "ON" : "OFF";
        const char* lsd_str[] = {"Open", "Viscous", "Clutch", "Torsen"};
        const char* lsd_name = lsd_str[static_cast<int>(params.diff_type)];
        SDL_RenderDebugTextFormat(renderer, 10.0f, 94.0f,
            "ABS: %s [%c%c%c%c]   TC: %s   LSD: %s (%.0f Nm)",
            abs_str,
            state.abs_active[0] ? 'X' : '.',
            state.abs_active[1] ? 'X' : '.',
            state.abs_active[2] ? 'X' : '.',
            state.abs_active[3] ? 'X' : '.',
            tc_str, lsd_name, state.diff_lock_torque);

        const float panel_x = static_cast<float>(w) - 190.0f;
        draw_gg(renderer, panel_x, 20.0f, 170.0f, trail);

        const char* names[4] = {"FL", "FR", "RL", "RR"};
        SDL_SetRenderDrawColor(renderer, 240, 240, 240, 255);
        for (std::size_t i = 0; i < 4; ++i) {
            const WheelState& ws = state.wheel[i];
            SDL_RenderDebugTextFormat(
                renderer, static_cast<float>(w) - 400.0f, 200.0f + 12.0f * static_cast<float>(i),
                "%s %5.0f N  k %+.2f  a %+5.1f  mu %.2f", names[i], ws.normal_load,
                ws.slip_ratio, std::atan(ws.tan_alpha) * 57.29578, grip[i]);
        }

        draw_graph(renderer, 10.0f, static_cast<float>(h) - 170.0f, 600.0f, 130.0f, history);
        SDL_SetRenderDrawColor(renderer, 240, 240, 240, 255);
        SDL_RenderDebugText(renderer, 10.0f, static_cast<float>(h) - 20.0f,
                            "W/S acelerar-frenar  A/D girar  E/Q marcha  T auto/manual  R reiniciar  A ABS  C TC  D LSD  Rueda zoom  Esc salir");
        SDL_RenderPresent(renderer);
    }

    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();
    return 0;
}