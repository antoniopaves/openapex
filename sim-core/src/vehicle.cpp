// SPDX-FileCopyrightText: 2026 OpenApex contributors
// SPDX-License-Identifier: GPL-3.0-or-later
#include "openapex/sim/vehicle.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>

namespace openapex::sim {
namespace {

constexpr double kGravity = 9.80665;
constexpr double kPi = 3.14159265358979323846;
constexpr double kRadPerSecToRpm = 60.0 / (2.0 * kPi);
constexpr double kMinSlipSpeed = 2.0;
constexpr double kAccelFilterHz = 20.0;
constexpr double kRpmFilterHz = 15.0;
constexpr double kSlipDelta = 1e-3;

struct WheelLayout {
    double x;
    double y;
    bool front;
};

constexpr double sq(double v) noexcept { return v * v; }

double torque_curve(const VehicleParams& p, double rpm) noexcept {
    const auto& r = p.torque_rpm;
    const auto& t = p.torque_nm;
    if (rpm <= r.front()) return t.front();
    if (rpm >= r.back()) return t.back();
    for (std::size_t i = 1; i < r.size(); ++i) {
        if (rpm <= r[i]) {
            const double f = (rpm - r[i - 1]) / (r[i] - r[i - 1]);
            return t[i - 1] + f * (t[i] - t[i - 1]);
        }
    }
    return t.back();
}

} // namespace

double total_gear_ratio(const VehicleParams& p, int gear) noexcept {
    if (gear == 0) return 0.0;
    if (gear < 0) return -p.reverse_ratio * p.final_drive;
    const auto index = static_cast<std::size_t>(gear - 1);
    return p.gear_ratios[std::min(index, p.gear_ratios.size() - 1)] * p.final_drive;
}

void shift_gear(const VehicleParams& p, VehicleState& s, int direction) noexcept {
    const int top = static_cast<int>(p.gear_ratios.size());
    const int target = std::clamp(s.gear + (direction > 0 ? 1 : -1), -1, top);
    if (target == s.gear) return;
    s.gear = target;
    s.shift_timer = p.shift_time;
}

void init_rolling(const VehicleParams& p, VehicleState& s, double speed) noexcept {
    s.vx = speed;
    s.vy = 0.0;
    for (auto& w : s.wheel) {
        w.omega = speed / p.wheel_radius;
        w.tan_alpha = 0.0;
    }
    const double wheel_rad_s = speed / p.wheel_radius;
    int gear = 1;
    const int top = static_cast<int>(p.gear_ratios.size());
    while (gear < top && wheel_rad_s * total_gear_ratio(p, gear) * kRadPerSecToRpm > 5500.0) {
        ++gear;
    }
    s.gear = gear;
    s.shift_timer = 0.0;
    s.engine_rpm = std::max(p.idle_rpm, wheel_rad_s * total_gear_ratio(p, gear) * kRadPerSecToRpm);
}
void step_vehicle(const VehicleParams& p, VehicleState& s, const VehicleInput& in, double dt,
                  const std::array<double, 4>& grip) noexcept {
    const double wheelbase = p.cg_to_front + p.cg_to_rear;
    const double speed = std::hypot(s.vx, s.vy);
    const double throttle = std::clamp(in.throttle, 0.0, 1.0);
    const double brake = std::clamp(in.brake, 0.0, 1.0);

    s.steer_angle = std::clamp(in.steer, -1.0, 1.0) * p.max_steer_angle /
                    (1.0 + sq(speed / p.steer_reference_speed));

    // Cargas verticales (transferencia longitudinal y lateral).
    const double half_track = 0.5 * p.track_width;
    const double front_axle_load = p.mass * kGravity * p.cg_to_rear / wheelbase -
                                   p.mass * s.ax * p.cg_height / wheelbase;
    const double rear_axle_load = p.mass * kGravity * p.cg_to_front / wheelbase +
                                  p.mass * s.ax * p.cg_height / wheelbase;
    const double lateral_transfer = p.mass * s.ay * p.cg_height / p.track_width;

    // Motor y embrague.
    s.shift_timer = std::max(0.0, s.shift_timer - dt);
    const double ratio = total_gear_ratio(p, s.gear);
    const bool coupled = s.gear != 0 && s.shift_timer <= 0.0;
    const double rear_omega = 0.5 * (s.wheel[2].omega + s.wheel[3].omega);
    const double wheel_rpm = std::max(0.0, rear_omega * ratio * kRadPerSecToRpm);
    const double free_rpm = p.idle_rpm + throttle * (p.launch_rpm - p.idle_rpm);
    const double engine_rpm = coupled ? std::max(wheel_rpm, free_rpm)
                                      : p.idle_rpm + throttle * (p.redline_rpm - p.idle_rpm);

    double engine_torque = throttle * torque_curve(p, engine_rpm) -
                           (1.0 - throttle) * (p.engine_brake_torque + p.engine_brake_per_rpm * engine_rpm);
    if (engine_rpm >= p.redline_rpm) engine_torque = std::min(engine_torque, 0.0);
    if (wheel_rpm < p.idle_rpm && engine_torque < 0.0) engine_torque = 0.0;
    const double axle_torque = coupled ? engine_torque * ratio * p.driveline_efficiency : 0.0;

    const std::array<WheelLayout, 4> wheels{{{p.cg_to_front, half_track, true},
                                             {p.cg_to_front, -half_track, true},
                                             {-p.cg_to_rear, half_track, false},
                                             {-p.cg_to_rear, -half_track, false}}};

    double total_fx = 0.0;
    double total_fy = 0.0;
    double total_mz = 0.0;

    for (std::size_t i = 0; i < wheels.size(); ++i) {
        const auto& w = wheels[i];
        auto& ws = s.wheel[i];

        const double axle_load = w.front ? front_axle_load : rear_axle_load;
        const double side = w.y > 0.0 ? -1.0 : 1.0;
        const double fz = std::max(0.0, 0.5 * axle_load + side * 0.5 * lateral_transfer);

        const double delta = w.front ? s.steer_angle : 0.0;
        const double cd = std::cos(delta);
        const double sd = std::sin(delta);

        const double vwx = s.vx - s.yaw_rate * w.y;
        const double vwy = s.vy + s.yaw_rate * w.x;
        const double v_long = vwx * cd + vwy * sd;
        const double v_lat = -vwx * sd + vwy * cd;

        const double slip_speed = std::max(std::abs(v_long), kMinSlipSpeed);

        // Slip lateral con relaxation length.
        const double rolling_speed = std::max(std::hypot(v_long, v_lat), kMinSlipSpeed);
        const double relax = 1.0 - std::exp(-rolling_speed * dt / p.relaxation_length);
        ws.tan_alpha += (v_lat / slip_speed - ws.tan_alpha) * relax;

        // Slip ratio y fuerzas combinadas.
        const double kappa = (ws.omega * p.wheel_radius - v_long) / slip_speed;
        PacejkaCoefficients tire = p.tire;
        tire.D *= grip[i];
        const TireForce f = combined_slip_force(tire, kappa, ws.tan_alpha, fz);
        const TireForce fh = combined_slip_force(tire, kappa + kSlipDelta, ws.tan_alpha, fz);
        const double dfx_dkappa = std::max(0.0, (fh.fx - f.fx) / kSlipDelta);

        // Dinámica de la rueda, semi-implícita en la fuerza del neumático.
        const double inertia = p.wheel_inertia + (w.front ? 0.0 : p.driveline_inertia);
        const double drive_torque = w.front ? 0.0 : 0.5 * axle_torque;
        const double denom = inertia + dt * sq(p.wheel_radius) * dfx_dkappa / slip_speed;
        double omega = ws.omega + dt * (drive_torque - p.wheel_radius * f.fx) / denom;

        // Freno como fricción seca: nunca invierte el giro de la rueda.
        const double bias = w.front ? p.brake_bias_front : 1.0 - p.brake_bias_front;
        const double brake_torque = brake * p.max_brake_force * p.wheel_radius * 0.5 * bias;
        const double d_brake = dt * brake_torque / inertia;
        omega = std::abs(omega) <= d_brake ? 0.0 : omega - std::copysign(d_brake, omega);

        ws.omega = omega;
        ws.slip_ratio = kappa;
        ws.normal_load = fz;

        const double bx = f.fx * cd - f.fy * sd;
        const double by = f.fx * sd + f.fy * cd;
        total_fx += bx;
        total_fy += by;
        total_mz += w.x * by - w.y * bx;
    }

    total_fx -= p.drag_factor * speed * s.vx +
                p.rolling_resistance * std::clamp(s.vx / 0.5, -1.0, 1.0);
    total_fy -= p.drag_factor * speed * s.vy;

    const double ax_new = total_fx / p.mass;
    const double ay_new = total_fy / p.mass;
    const double vx_old = s.vx;
    const double vy_old = s.vy;

    s.vx += (ax_new + s.yaw_rate * vy_old) * dt;
    s.vy += (ay_new - s.yaw_rate * vx_old) * dt;
    s.yaw_rate += total_mz / p.yaw_inertia * dt;

    const double alpha = std::min(1.0, kAccelFilterHz * dt);
    s.ax += (ax_new - s.ax) * alpha;
    s.ay += (ay_new - s.ay) * alpha;

    const double cy = std::cos(s.yaw);
    const double sy = std::sin(s.yaw);
    s.x += (s.vx * cy - s.vy * sy) * dt;
    s.y += (s.vx * sy + s.vy * cy) * dt;
    s.yaw += s.yaw_rate * dt;

    // Cambio automático (solo entre 1..6; en N y R no actúa).
    if (s.auto_shift && s.gear >= 1 && s.shift_timer <= 0.0) {
        if (engine_rpm > p.upshift_rpm && s.gear < static_cast<int>(p.gear_ratios.size())) {
            shift_gear(p, s, +1);
        } else if (s.gear > 1 && engine_rpm < p.downshift_rpm) {
            shift_gear(p, s, -1);
        }
    }

    s.engine_rpm += (engine_rpm - s.engine_rpm) * std::min(1.0, kRpmFilterHz * dt);
}

void step_vehicle(const VehicleParams& p, VehicleState& s, const VehicleInput& in,
                  double dt) noexcept {
    step_vehicle(p, s, in, dt, std::array<double, 4>{1.0, 1.0, 1.0, 1.0});
}

} // namespace openapex::sim
