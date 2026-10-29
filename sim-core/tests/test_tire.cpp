// SPDX-FileCopyrightText: 2026 OpenApex contributors
// SPDX-License-Identifier: GPL-3.0-or-later
#include "openapex/sim/tire.hpp"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>

using namespace openapex::sim;
using Catch::Approx;

TEST_CASE("Sin deslizamiento no hay fuerza", "[tire]") {
    PacejkaCoefficients c;
    REQUIRE(lateral_force(c, 0.0, 4000.0) == Approx(0.0).margin(1e-9));
}

TEST_CASE("La fuerza es impar respecto al deslizamiento", "[tire]") {
    PacejkaCoefficients c;
    for (double a : {0.01, 0.05, 0.1, 0.3}) {
        REQUIRE(lateral_force(c, -a, 3000.0) == Approx(-lateral_force(c, a, 3000.0)));
    }
}

TEST_CASE("La fuerza pico se acerca a D*Fz y nunca la supera", "[tire]") {
    PacejkaCoefficients c;
    const double fz = 4000.0;
    double peak = 0.0;
    for (double a = 0.0; a <= 0.5; a += 0.001) {
        peak = std::max(peak, lateral_force(c, a, fz));
    }
    REQUIRE(peak <= fz * c.D * (1.0 + 1e-9));
    REQUIRE(peak >= fz * c.D * 0.99);
}

TEST_CASE("La fuerza es proporcional a la carga vertical", "[tire]") {
    PacejkaCoefficients c;
    REQUIRE(lateral_force(c, 0.1, 6000.0) == Approx(2.0 * lateral_force(c, 0.1, 3000.0)));
}
