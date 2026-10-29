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