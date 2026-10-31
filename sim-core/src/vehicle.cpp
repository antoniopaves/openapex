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

    // Inicializar presión de freno ABS
    s.brake_pressure = {1.0, 1.0, 1.0, 1.0};
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

    // ABS: usar presión de freno modulada persistente (inicializar si es primera vez)
    // brake_pressure ya está en VehicleState y persiste entre steps

    // TC: calcular multiplicador de torque al inicio del step usando slip del frame anterior
    // Esto garantiza simetría entre ambas ruedas traseras
    double tc_torque_multiplier = 1.0;
    if (p.tc_enabled && throttle > 0.0) {
        const double rear_slip_rl = std::max(0.0, s.wheel[2].slip_ratio);
        const double rear_slip_rr = std::max(0.0, s.wheel[3].slip_ratio);
        const double max_rear_slip = std::max(rear_slip_rl, rear_slip_rr);
        if (max_rear_slip > p.tc_slip_threshold) {
            const double excess = (max_rear_slip - p.tc_slip_threshold) / p.tc_slip_threshold;
            // Reducción suave limitada por rate
            const double reduction = std::min(1.0, excess * p.tc_torque_reduction_rate * dt);
            tc_torque_multiplier = std::max(0.0, 1.0 - reduction);
            s.tc_active = true;
        } else {
            s.tc_active = false;
        }
    } else {
        s.tc_active = false;
    }

    // LSD: calcular torque de bloqueo y reparto L/R
    // Ruedas traseras: índice 2 = izquierda (y > 0), índice 3 = derecha (y < 0)
    double lsd_lock_torque = 0.0;
    double drive_torque_rl = 0.5 * axle_torque * tc_torque_multiplier; // rear left
    double drive_torque_rr = 0.5 * axle_torque * tc_torque_multiplier; // rear right

    if (p.diff_type != VehicleParams::DiffType::Open && axle_torque != 0.0) {
        const bool on_power = axle_torque > 0.0;
        const double lock_ratio = on_power ? p.diff_lock_power : p.diff_lock_coast;
        const double ramp_angle_deg = on_power ? p.diff_ramp_angle_power : p.diff_ramp_angle_coast;
        const double ramp_angle_rad = ramp_angle_deg * kPi / 180.0;

        switch (p.diff_type) {
            case VehicleParams::DiffType::Clutch: {
                // Clutch-type LSD: T_lock_max = preload + lock_ratio * T_input * tan(ramp_angle)
                // Torque de bloqueo real proporcional a diferencia de velocidad, limitado por T_lock_max
                const double speed_diff = std::abs(s.wheel[2].omega - s.wheel[3].omega);
                const double lsd_capacity = p.diff_preload
                                          + lock_ratio * std::abs(axle_torque) * std::tan(ramp_angle_rad)
                                          * static_cast<double>(p.diff_clutch_packs);
                // Rigidez del diferencial (Nm/rad/s) - alta para bloquear rápido
                const double lsd_stiffness = 200.0;
                lsd_lock_torque = std::min(lsd_stiffness * speed_diff, lsd_capacity);
                break;
            }
            case VehicleParams::DiffType::Viscous: {
                // Viscous LSD: T_lock = viscous_coeff * speed_diff
                const double speed_diff = std::abs(s.wheel[2].omega - s.wheel[3].omega);
                const double viscous_coeff = 50.0; // Nm/(rad/s)
                lsd_lock_torque = viscous_coeff * speed_diff;
                break;
            }
            case VehicleParams::DiffType::Torsen: {
                // Torsen LSD: bias ratio basado en ángulo de rampas helicoidales
                const double bias_ratio = 1.0 + 2.0 * std::tan(ramp_angle_rad);
                // T_lock_max = (bias_ratio - 1) / (bias_ratio + 1) * T_input
                const double lsd_capacity = (bias_ratio - 1.0) / (bias_ratio + 1.0) * std::abs(axle_torque);
                // Torque real proporcional a diferencia de velocidad
                const double speed_diff = std::abs(s.wheel[2].omega - s.wheel[3].omega);
                const double torsen_stiffness = 300.0;
                lsd_lock_torque = std::min(torsen_stiffness * speed_diff, lsd_capacity);
                break;
            }
            default:
                lsd_lock_torque = 0.0;
        }

        // Limitar torque de bloqueo al torque de entrada
        lsd_lock_torque = std::min(lsd_lock_torque, std::abs(axle_torque));

        // Reparto: la rueda que gira más lento recibe más torque (bloqueo)
        // En aceleración (power): rueda exterior (más carga) tiende a girar más rápido -> darle más torque
        // En retención (coast): rueda interior tiende a girar más rápido
        const double sign = on_power ? 1.0 : -1.0;
        drive_torque_rl = 0.5 * axle_torque * tc_torque_multiplier + 0.5 * sign * lsd_lock_torque;
        drive_torque_rr = 0.5 * axle_torque * tc_torque_multiplier - 0.5 * sign * lsd_lock_torque;
    }

    // Guardar para telemetría
    s.diff_lock_torque = lsd_lock_torque;

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

        // ABS: modular presión de freno si slip excede threshold
        if (p.abs_enabled && brake > 0.0) {
            const double abs_slip = std::abs(kappa);
            if (abs_slip > p.abs_slip_threshold) {
                // Reducir presión proporcionalmente al exceso de slip
                const double excess = (abs_slip - p.abs_slip_threshold) / p.abs_slip_threshold;
                const double reduction = std::min(1.0, excess * p.abs_modulation_rate * dt);
                s.brake_pressure[i] = std::max(0.0, s.brake_pressure[i] - reduction);
                s.abs_active[i] = true;
            } else {
                // Recuperar presión gradualmente
                s.brake_pressure[i] = std::min(1.0, s.brake_pressure[i] + p.abs_modulation_rate * dt);
                s.abs_active[i] = false;
            }
        } else {
            s.brake_pressure[i] = 1.0;
            s.abs_active[i] = false;
        }

        // Dinámica de la rueda, semi-implícita en la fuerza del neumático.
        const double inertia = p.wheel_inertia + (w.front ? 0.0 : p.driveline_inertia);
        double drive_torque;
        if (w.front) {
            drive_torque = 0.0;
        } else if (i == 2) { // rear left (y > 0)
            drive_torque = drive_torque_rl;
        } else { // i == 3, rear right (y < 0)
            drive_torque = drive_torque_rr;
        }
        const double denom = inertia + dt * sq(p.wheel_radius) * dfx_dkappa / slip_speed;
        double omega = ws.omega + dt * (drive_torque - p.wheel_radius * f.fx) / denom;

        // Freno como fricción seca: nunca invierte el giro de la rueda.
        const double bias = w.front ? p.brake_bias_front : 1.0 - p.brake_bias_front;
        const double brake_torque = brake * p.max_brake_force * p.wheel_radius * 0.5 * bias * s.brake_pressure[i];
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
