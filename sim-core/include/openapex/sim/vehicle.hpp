// SPDX-FileCopyrightText: 2026 OpenApex contributors
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include "openapex/sim/tire.hpp"

#include <array>

namespace openapex::sim {

/// Parámetros del coche. Ejes del cuerpo: x adelante, y a la izquierda, z arriba. SI.
/// Tracción trasera. Ruedas en orden: delantera izq., delantera der., trasera izq., trasera der.
struct VehicleParams {
    // Chasis
    double mass = 1200.0;
    double yaw_inertia = 1600.0;
    double cg_to_front = 1.2;
    double cg_to_rear = 1.4;
    double track_width = 1.5;
    double cg_height = 0.5;

    // Dirección
    double max_steer_angle = 0.55;
    double steer_reference_speed = 30.0;

    // Aerodinámica y rodadura
    double drag_factor = 0.42;
    double rolling_resistance = 180.0;

    // Frenos
    double max_brake_force = 12000.0; ///< Fuerza total de frenado en el suelo (N)
    double brake_bias_front = 0.65;

    // Ruedas y neumáticos
    double wheel_radius = 0.32;
    double wheel_inertia = 1.2;        ///< kg*m^2 por rueda
    double relaxation_length = 0.5;    ///< m, retardo del slip lateral
    PacejkaCoefficients tire{10.0, 1.9, 1.5, 0.97};

    // Motor
    double idle_rpm = 1000.0;
    double launch_rpm = 4500.0; ///< rpm que alcanza el motor con el embrague patinando a fondo
    double redline_rpm = 7500.0;
    std::array<double, 8> torque_rpm{1000.0, 2000.0, 3000.0, 4000.0, 5000.0, 6000.0, 7000.0, 7500.0};
    std::array<double, 8> torque_nm{150.0, 190.0, 230.0, 260.0, 280.0, 275.0, 250.0, 210.0};
    double engine_brake_torque = 25.0;
    double engine_brake_per_rpm = 0.012;

    // Transmisión
    std::array<double, 6> gear_ratios{3.6, 2.2, 1.6, 1.25, 1.0, 0.82};
    double reverse_ratio = 3.4;
    double final_drive = 3.7;
    double driveline_efficiency = 0.9;
    double driveline_inertia = 1.5; ///< kg*m^2 equivalentes por rueda motriz
    double shift_time = 0.2;
    double upshift_rpm = 6800.0;
    double downshift_rpm = 2400.0;

    // ABS
    bool abs_enabled = true;
    double abs_slip_threshold = 0.15;      // Optimal slip ratio for peak braking
    double abs_modulation_rate = 10.0;     // How fast to modulate (Hz)

    // Traction Control
    bool tc_enabled = true;
    double tc_slip_threshold = 0.08;       // Optimal slip ratio for acceleration
    double tc_torque_reduction_rate = 5.0; // Torque reduction speed

    // LSD (Limited Slip Differential)
    enum class DiffType { Open, Viscous, Clutch, Torsen };
    DiffType diff_type = DiffType::Open;
    double diff_preload = 50.0;            // Nm base locking torque
    double diff_lock_coast = 0.3;          // Lock ratio on coast (0-1)
    double diff_lock_power = 0.7;          // Lock ratio on power (0-1)
    double diff_ramp_angle_power = 45.0;   // Degrees
    double diff_ramp_angle_coast = 30.0;   // Degrees
    int diff_clutch_packs = 4;             // Number of clutch plates
};

struct WheelState {
    double omega = 0.0;     ///< rad/s
    double tan_alpha = 0.0; ///< tangente del slip angle (ya con relaxation)
    double slip_ratio = 0.0;
    double normal_load = 0.0;
};

struct VehicleState {
    double x = 0.0;
    double y = 0.0;
    double yaw = 0.0;
    double vx = 0.0;
    double vy = 0.0;
    double yaw_rate = 0.0;
    double ax = 0.0;
    double ay = 0.0;
    double steer_angle = 0.0;

    std::array<WheelState, 4> wheel{};
    int gear = 1; ///< -1 marcha atrás, 0 punto muerto, 1..6
    double engine_rpm = 1000.0;
    double shift_timer = 0.0;
    bool auto_shift = true;

    // ABS/TC/LSD states
    std::array<bool, 4> abs_active{};
    std::array<double, 4> brake_pressure{1.0, 1.0, 1.0, 1.0}; ///< Modulated brake pressure per wheel (persists across steps)
    bool tc_active = false;
    double diff_lock_torque = 0.0;
};

struct VehicleInput {
    double throttle = 0.0;
    double brake = 0.0;
    double steer = 0.0;
};

/// Relación total (marcha * diferencial). Negativa en reversa, 0 en punto muerto.
[[nodiscard]] double total_gear_ratio(const VehicleParams& params, int gear) noexcept;

/// Cambia de marcha un paso (direction > 0 sube, <= 0 baja). Respeta los límites -1..6.
void shift_gear(const VehicleParams& params, VehicleState& state, int direction) noexcept;

/// Pone el coche rodando a `speed` m/s hacia delante, con ruedas sincronizadas y marcha adecuada.
void init_rolling(const VehicleParams& params, VehicleState& state, double speed) noexcept;

/// Avanza la simulación dt segundos (pensado para dt = 1/360). Agarre 1.0 en las cuatro ruedas.
void step_vehicle(const VehicleParams& params, VehicleState& state, const VehicleInput& input,
                  double dt) noexcept;

/// Igual, con un multiplicador de agarre por rueda (orden: FL, FR, RL, RR).
void step_vehicle(const VehicleParams& params, VehicleState& state, const VehicleInput& input,
                  double dt, const std::array<double, 4>& grip) noexcept;

} // namespace openapex::sim
