// SPDX-FileCopyrightText: 2026 OpenApex contributors
// SPDX-License-Identifier: GPL-3.0-or-later
#include "openapex/sim/track.hpp"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cmath>

using namespace openapex::sim;
using Catch::Approx;

TEST_CASE("La pista de demostración tiene una longitud razonable", "[track]") {
    const Track t = Track::make_demo();
    REQUIRE(t.length() > 1000.0);
    REQUIRE(t.length() < 3000.0);
}

TEST_CASE("El eje de la pista es asfalto con lateral cero", "[track]") {
    const Track t = Track::make_demo();
    const Vec2 p = t.centerline()[100];
    const TrackQuery q = t.query(p.x, p.y);
    REQUIRE(q.lateral == Approx(0.0).margin(1e-6));
    REQUIRE(q.surface == Surface::Asphalt);
}

TEST_CASE("Lejos de la pista es hierba", "[track]") {
    const Track t = Track::make_demo();
    const Vec2 p = t.start_position();
    const double h = t.start_heading();
    const TrackQuery q = t.query(p.x - std::sin(h) * 30.0, p.y + std::cos(h) * 30.0);
    REQUIRE(q.surface == Surface::Grass);
}

TEST_CASE("Lateral positivo a la izquierda y negativo a la derecha", "[track]") {
    const Track t = Track::make_demo();
    const Vec2 p = t.start_position();
    const double h = t.start_heading();
    const double nx = -std::sin(h);
    const double ny = std::cos(h);
    REQUIRE(t.query(p.x + nx * 3.0, p.y + ny * 3.0).lateral == Approx(3.0).margin(0.05));
    REQUIRE(t.query(p.x - nx * 3.0, p.y - ny * 3.0).lateral == Approx(-3.0).margin(0.05));
}

TEST_CASE("La distancia recorrida crece a lo largo del eje", "[track]") {
    const Track t = Track::make_demo();
    const Vec2 a = t.centerline()[50];
    const Vec2 b = t.centerline()[100];
    REQUIRE(t.query(a.x, a.y).along < t.query(b.x, b.y).along);
}

TEST_CASE("El agarre baja de asfalto a piano a hierba", "[track]") {
    REQUIRE(surface_grip(Surface::Asphalt) > surface_grip(Surface::Kerb));
    REQUIRE(surface_grip(Surface::Kerb) > surface_grip(Surface::Grass));
}