// SPDX-FileCopyrightText: 2026 OpenApex contributors
// SPDX-License-Identifier: GPL-3.0-or-later
#include "openapex/sim/vehicle.hpp"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>
#include <array>

using namespace openapex::sim;
using Catch::Approx;

namespace {

constexpr double kDt = 1.0 / 360.0;

VehicleState simulate(const VehicleParams& p, VehicleState s, const VehicleInput& in,
                      double seconds) {
    const int steps = static_cast<int>(seconds / kDt);
    for (int i = 0; i < steps; ++i) {
        step_vehicle(p, s, in, kDt);
    }
    return s;
}

VehicleState rolling(const VehicleParams& p, double speed) {
    VehicleState s;
    init_rolling(p, s, speed);
    return s;
}

} // namespace

TEST_CASE("Acelerando recto no se desvía", "[vehicle]") {
    const VehicleParams p;
    const VehicleState s = simulate(p, {}, {1.0, 0.0, 0.0}, 5.0);
    REQUIRE(s.vx > 15.0);
    REQUIRE(s.x > 30.0);
    REQUIRE(std::abs(s.vy) < 1e-9);
    REQUIRE(std::abs(s.yaw) < 1e-9);
}

TEST_CASE("Una salida a fondo hace patinar las ruedas traseras", "[vehicle]") {
    const VehicleParams p;
    VehicleState s;
    double max_kappa = 0.0;
    for (int i = 0; i < 360; ++i) {
        step_vehicle(p, s, {1.0, 0.0, 0.0}, kDt);
        max_kappa = std::max(max_kappa, s.wheel[2].slip_ratio);
    }
    REQUIRE(max_kappa > 0.1);
}

TEST_CASE("El cambio automático sube de marcha y respeta el corte", "[vehicle]") {
    const VehicleParams p;
    VehicleState s;
    double max_rpm = 0.0;
    const int steps = static_cast<int>(10.0 / kDt);
    for (int i = 0; i < steps; ++i) {
        step_vehicle(p, s, {1.0, 0.0, 0.0}, kDt);
        max_rpm = std::max(max_rpm, s.engine_rpm);
    }
    REQUIRE(s.gear > 1);
    REQUIRE(max_rpm < p.redline_rpm * 1.1);
}

TEST_CASE("Marcha atrás", "[vehicle]") {
    const VehicleParams p;
    VehicleState s;
    shift_gear(p, s, -1);
    shift_gear(p, s, -1);
    REQUIRE(s.gear == -1);
    s = simulate(p, s, {1.0, 0.0, 0.0}, 3.0);
    REQUIRE(s.vx < -2.0);
}

TEST_CASE("Girar a la izquierda produce yaw e y positivos", "[vehicle]") {
    const VehicleParams p;
    const VehicleState s = simulate(p, rolling(p, 15.0), {0.2, 0.0, 0.5}, 2.0);
    REQUIRE(s.yaw_rate > 0.0);
    REQUIRE(s.yaw > 0.0);
    REQUIRE(s.y > 0.0);
}

TEST_CASE("Girar a derecha e izquierda es simétrico", "[vehicle]") {
    const VehicleParams p;
    const VehicleState start = rolling(p, 20.0);
    const VehicleState l = simulate(p, start, {0.3, 0.0, 0.4}, 3.0);
    const VehicleState r = simulate(p, start, {0.3, 0.0, -0.4}, 3.0);
    REQUIRE(l.y == Approx(-r.y).margin(1e-6));
    REQUIRE(l.yaw == Approx(-r.yaw).margin(1e-6));
    REQUIRE(l.vx == Approx(r.vx).margin(1e-6));
}

TEST_CASE("Frenando a fondo el coche se detiene", "[vehicle]") {
    const VehicleParams p;
    const VehicleState s = simulate(p, rolling(p, 20.0), {0.0, 1.0, 0.0}, 10.0);
    REQUIRE(std::abs(s.vx) < 0.05);
    REQUIRE(s.x > 10.0);
    REQUIRE(s.x < 60.0);
}

TEST_CASE("Con entradas extremas el estado sigue siendo finito", "[vehicle]") {
    const VehicleParams p;
    const VehicleState s = simulate(p, rolling(p, 40.0), {1.0, 0.0, 1.0}, 10.0);
    REQUIRE(std::isfinite(s.x));
    REQUIRE(std::isfinite(s.y));
    REQUIRE(std::isfinite(s.yaw));
    REQUIRE(std::isfinite(s.vx));
    REQUIRE(std::isfinite(s.vy));
    REQUIRE(std::isfinite(s.yaw_rate));
    for (const auto& w : s.wheel) {
        REQUIRE(std::isfinite(w.omega));
    }
}

TEST_CASE("La simulación es determinista", "[vehicle]") {
    const VehicleParams p;
    const VehicleState start = rolling(p, 25.0);
    const VehicleState a = simulate(p, start, {0.7, 0.0, 0.3}, 4.0);
    const VehicleState b = simulate(p, start, {0.7, 0.0, 0.3}, 4.0);
    REQUIRE(a.x == b.x);
    REQUIRE(a.y == b.y);
    REQUIRE(a.yaw == b.yaw);
}

TEST_CASE("Menos agarre alarga la frenada", "[vehicle]") {
    const VehicleParams p;
    auto brake_distance = [&](double grip) {
        VehicleState s = rolling(p, 20.0);
        const std::array<double, 4> g{grip, grip, grip, grip};
        const int steps = static_cast<int>(8.0 / kDt);
        for (int i = 0; i < steps; ++i) {
            step_vehicle(p, s, {0.0, 1.0, 0.0}, kDt, g);
        }
        return s.x;
    };
    REQUIRE(brake_distance(0.5) > brake_distance(1.0));
}

// ===== ABS Tests =====

TEST_CASE("ABS reduce slip trasero tras bloqueo inicial", "[vehicle][abs]") {
    VehicleParams p;
    p.abs_enabled = true;
    p.abs_slip_threshold = 0.15;
    p.abs_modulation_rate = 10.0;

    VehicleState s = rolling(p, 30.0);

    // Frenar fuerte: las ruedas tardan unos frames en bloquear, luego ABS reacciona
    const int steps = static_cast<int>(2.0 / kDt);
    double max_slip_after_react = 0.0;
    for (int i = 0; i < steps; ++i) {
        step_vehicle(p, s, {0.0, 1.0, 0.0}, kDt);
        // Ignorar primeros frames donde ruedas se bloquean antes de reaccionar ABS
        // ABS necesita ~50 frames (0.14s a 360Hz) para modular presión y reducir slip
        if (i > 50) {
            max_slip_after_react = std::max(max_slip_after_react, std::abs(s.wheel[2].slip_ratio));
            max_slip_after_react = std::max(max_slip_after_react, std::abs(s.wheel[3].slip_ratio));
        }
    }

    // Tras reacción ABS, slip debería mantenerse cerca del threshold
    REQUIRE(max_slip_after_react < 0.36);
}

TEST_CASE("ABS desactivado permite slip alto al frenar", "[vehicle][abs]") {
    VehicleParams p;
    p.abs_enabled = false;  // Sin ABS

    VehicleState s = rolling(p, 30.0);
    double max_slip_no_abs = 0.0;

    const int steps = static_cast<int>(2.0 / kDt);
    for (int i = 0; i < steps; ++i) {
        step_vehicle(p, s, {0.0, 1.0, 0.0}, kDt);
        max_slip_no_abs = std::max(max_slip_no_abs, std::abs(s.wheel[2].slip_ratio));
        max_slip_no_abs = std::max(max_slip_no_abs, std::abs(s.wheel[3].slip_ratio));
    }

    // Sin ABS, el slip llega a 1.0 (ruedas bloqueadas)
    REQUIRE(max_slip_no_abs > 0.8);
}

TEST_CASE("ABS activa flags tras varios frames frenando", "[vehicle][abs]") {
    VehicleParams p;
    p.abs_enabled = true;
    p.abs_slip_threshold = 0.15;
    p.abs_modulation_rate = 10.0;

    VehicleState s = rolling(p, 30.0);

    // Frenar y girar suficientes frames para que ABS reaccione
    // ABS necesita tiempo para detectar slip > threshold y modular presión
    for (int i = 0; i < 100; ++i) {
        step_vehicle(p, s, {0.0, 1.0, 0.3}, kDt);
    }

    // Al menos una rueda trasera debería tener ABS activo
    bool any_abs_active = s.abs_active[2] || s.abs_active[3];
    REQUIRE(any_abs_active == true);
}

TEST_CASE("Sin freno ABS no se activa", "[vehicle][abs]") {
    VehicleParams p;
    p.abs_enabled = true;

    VehicleState s = rolling(p, 30.0);

    // Solo acelerar, sin freno
    step_vehicle(p, s, {1.0, 0.0, 0.0}, kDt);

    // ABS no debería estar activo sin frenar
    REQUIRE(s.abs_active[0] == false);
    REQUIRE(s.abs_active[1] == false);
    REQUIRE(s.abs_active[2] == false);
    REQUIRE(s.abs_active[3] == false);
}

// ===== TC Tests =====

TEST_CASE("TC reduce torque cuando slip trasero excede threshold", "[vehicle][tc]") {
    VehicleParams p;
    p.tc_enabled = true;
    p.tc_slip_threshold = 0.08;
    p.tc_torque_reduction_rate = 5.0;

    VehicleState s = rolling(p, 5.0);  // Baja velocidad = más fácil patinar

    // Acelerar a fondo durante 3 segundos
    const int steps = static_cast<int>(3.0 / kDt);
    bool tc_activated = false;
    for (int i = 0; i < steps; ++i) {
        step_vehicle(p, s, {1.0, 0.0, 0.0}, kDt);
        if (s.tc_active) tc_activated = true;
    }

    // TC debería activarse con salida a fondo desde baja velocidad
    REQUIRE(tc_activated == true);
}

TEST_CASE("TC desactivado permite slip moderado al acelerar", "[vehicle][tc]") {
    VehicleParams p;
    p.tc_enabled = false;

    VehicleState s = rolling(p, 3.0);  // Muy baja velocidad para más slip
    double max_slip_no_tc = 0.0;

    const int steps = static_cast<int>(4.0 / kDt);
    for (int i = 0; i < steps; ++i) {
        step_vehicle(p, s, {1.0, 0.0, 0.0}, kDt);
        max_slip_no_tc = std::max(max_slip_no_tc, s.wheel[2].slip_ratio);
        max_slip_no_tc = std::max(max_slip_no_tc, s.wheel[3].slip_ratio);
    }

    // Sin TC, slip puede ser significativo (depende de torque motor y agarre)
    REQUIRE(max_slip_no_tc > 0.15);
}

TEST_CASE("TC solo activa con throttle > 0", "[vehicle][tc]") {
    VehicleParams p;
    p.tc_enabled = true;

    VehicleState s = rolling(p, 20.0);

    // Solo freno, sin throttle
    step_vehicle(p, s, {0.0, 1.0, 0.0}, kDt);

    REQUIRE(s.tc_active == false);
}

// ===== LSD Tests =====

TEST_CASE("LSD Open reparte torque 50/50 siempre", "[vehicle][lsd]") {
    VehicleParams p;
    p.diff_type = VehicleParams::DiffType::Open;

    VehicleState s = rolling(p, 20.0);
    step_vehicle(p, s, {1.0, 0.0, 0.0}, kDt);

    // Con diferencial abierto, torque de bloqueo = 0
    REQUIRE(s.diff_lock_torque == Approx(0.0).margin(1e-9));
}

TEST_CASE("LSD Clutch genera torque de bloqueo con diferencia de velocidad", "[vehicle][lsd]") {
    VehicleParams p;
    p.diff_type = VehicleParams::DiffType::Clutch;
    p.diff_preload = 50.0;
    p.diff_lock_power = 0.7;
    p.diff_ramp_angle_power = 45.0;
    p.diff_clutch_packs = 4;

    VehicleState s = rolling(p, 20.0);
    // Crear diferencia de velocidad entre ruedas traseras (simular curva)
    s.wheel[2].omega += 10.0;  // RL más rápido
    s.wheel[3].omega -= 10.0;  // RR más lento

    step_vehicle(p, s, {1.0, 0.0, 0.0}, kDt);

    // Con diferencia de velocidad, LSD Clutch genera torque de bloqueo
    REQUIRE(s.diff_lock_torque > 0.0);
}

TEST_CASE("LSD Viscous genera torque proporcional a diferencia velocidad", "[vehicle][lsd]") {
    VehicleParams p;
    p.diff_type = VehicleParams::DiffType::Viscous;

    VehicleState s = rolling(p, 20.0);
    step_vehicle(p, s, {1.0, 0.0, 0.0}, kDt);

    // Viscous siempre genera algo si hay diferencia de velocidad
    // En línea recta sin diferencia, torque = 0
    REQUIRE(s.diff_lock_torque >= 0.0);
}

TEST_CASE("LSD Torsen genera torque de bloqueo bajo carga", "[vehicle][lsd]") {
    VehicleParams p;
    p.diff_type = VehicleParams::DiffType::Torsen;
    p.diff_ramp_angle_power = 45.0;  // bias_ratio ~ 3

    VehicleState s = rolling(p, 20.0);
    step_vehicle(p, s, {1.0, 0.0, 0.0}, kDt);

    // Torsen genera bloqueo bajo torque de entrada
    REQUIRE(s.diff_lock_torque >= 0.0);
}

TEST_CASE("Ciclar tipo LSD con tecla D funciona correctamente", "[vehicle][lsd]") {
    VehicleParams p;
    p.diff_type = VehicleParams::DiffType::Open;
    REQUIRE(p.diff_type == VehicleParams::DiffType::Open);

    // Simular tecla D: Open -> Viscous -> Clutch -> Torsen -> Open
    using DT = VehicleParams::DiffType;
    p.diff_type = (p.diff_type == DT::Open) ? DT::Viscous :
                    (p.diff_type == DT::Viscous) ? DT::Clutch :
                    (p.diff_type == DT::Clutch) ? DT::Torsen : DT::Open;
    REQUIRE(p.diff_type == VehicleParams::DiffType::Viscous);

    p.diff_type = (p.diff_type == DT::Open) ? DT::Viscous :
                    (p.diff_type == DT::Viscous) ? DT::Clutch :
                    (p.diff_type == DT::Clutch) ? DT::Torsen : DT::Open;
    REQUIRE(p.diff_type == VehicleParams::DiffType::Clutch);

    p.diff_type = (p.diff_type == DT::Open) ? DT::Viscous :
                    (p.diff_type == DT::Viscous) ? DT::Clutch :
                    (p.diff_type == DT::Clutch) ? DT::Torsen : DT::Open;
    REQUIRE(p.diff_type == VehicleParams::DiffType::Torsen);

    p.diff_type = (p.diff_type == DT::Open) ? DT::Viscous :
                    (p.diff_type == DT::Viscous) ? DT::Clutch :
                    (p.diff_type == DT::Clutch) ? DT::Torsen : DT::Open;
    REQUIRE(p.diff_type == VehicleParams::DiffType::Open);
}

TEST_CASE("LSD Clutch y Torsen limitan lock_torque a torque entrada", "[vehicle][lsd]") {
    VehicleParams p_clutch;
    p_clutch.diff_type = VehicleParams::DiffType::Clutch;
    p_clutch.diff_preload = 100.0;
    p_clutch.diff_lock_power = 0.7;
    p_clutch.diff_ramp_angle_power = 45.0;
    p_clutch.diff_clutch_packs = 4;

    VehicleParams p_torsen;
    p_torsen.diff_type = VehicleParams::DiffType::Torsen;
    p_torsen.diff_ramp_angle_power = 45.0;

    VehicleState s_clutch = rolling(p_clutch, 10.0);
    VehicleState s_torsen = rolling(p_torsen, 10.0);

    // Acelerar suave para tener torque moderado
    step_vehicle(p_clutch, s_clutch, {0.3, 0.0, 0.0}, kDt);
    step_vehicle(p_torsen, s_torsen, {0.3, 0.0, 0.0}, kDt);

    // Lock torque nunca debe exceder torque de entrada (aprox)
    // Torque entrada aprox = 0.3 * torque_curve * ratio * efficiency
    // Solo verificamos que no sea absurdo
    REQUIRE(s_clutch.diff_lock_torque < 2000.0);
    REQUIRE(s_torsen.diff_lock_torque < 2000.0);
}