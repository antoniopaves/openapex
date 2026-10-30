// SPDX-FileCopyrightText: 2026 OpenApex contributors
// SPDX-License-Identifier: GPL-3.0-or-later
#include "openapex/sim/track.hpp"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <fstream>
#include <filesystem>

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

// --- File I/O Tests ---

TEST_CASE("Track save and load roundtrip preserves data", "[track][io]") {
    // Create a JSON track file with all features
    const std::string temp_path = "test_track_roundtrip.track.json";
    {
        std::ofstream file(temp_path);
        file << R"({
    "version": 1,
    "metadata": {
        "name": "Test Track",
        "author": "Test Author",
        "description": "Test Description",
        "version": 1
    },
    "geometry": {
        "centerline": [
            {"x": 0.0, "y": 0.0, "width": 12.0, "kerb_width": 1.5, "elevation": 0.0, "banking": 0.0, "surface_left": "asphalt", "surface_right": "asphalt"},
            {"x": 20.0, "y": 0.0, "width": 12.0, "kerb_width": 1.5, "elevation": 0.5, "banking": 0.05, "surface_left": "asphalt", "surface_right": "asphalt"},
            {"x": 40.0, "y": 0.0, "width": 12.0, "kerb_width": 1.5, "elevation": 1.0, "banking": 0.1, "surface_left": "asphalt", "surface_right": "asphalt"},
            {"x": 60.0, "y": 0.0, "width": 12.0, "kerb_width": 1.5, "elevation": 1.5, "banking": 0.15, "surface_left": "asphalt", "surface_right": "asphalt"},
            {"x": 60.0, "y": 20.0, "width": 12.0, "kerb_width": 1.5, "elevation": 2.0, "banking": 0.2, "surface_left": "asphalt", "surface_right": "asphalt"},
            {"x": 60.0, "y": 40.0, "width": 12.0, "kerb_width": 1.5, "elevation": 2.5, "banking": 0.25, "surface_left": "kerb", "surface_right": "grass"},
            {"x": 40.0, "y": 40.0, "width": 12.0, "kerb_width": 1.5, "elevation": 3.0, "banking": 0.3, "surface_left": "kerb", "surface_right": "grass"},
            {"x": 20.0, "y": 40.0, "width": 12.0, "kerb_width": 1.5, "elevation": 3.5, "banking": 0.35, "surface_left": "kerb", "surface_right": "grass"},
            {"x": 0.0, "y": 40.0, "width": 12.0, "kerb_width": 1.5, "elevation": 4.0, "banking": 0.4, "surface_left": "kerb", "surface_right": "grass"},
            {"x": 0.0, "y": 20.0, "width": 12.0, "kerb_width": 1.5, "elevation": 4.5, "banking": 0.45, "surface_left": "kerb", "surface_right": "grass"}
        ]
    },
    "objects": [
        {"type": "cone", "position": {"x": 5.0, "y": 3.0}, "rotation": 0.0, "scale": 1.0},
        {"type": "tire_barrier", "position": {"x": 50.0, "y": -2.0}, "rotation": 1.57, "scale": 1.0}
    ],
    "pit_lane": {"entry": 100.0, "exit": 200.0, "speed_limit": 22.222},
    "start_position": {"x": 0.0, "y": 0.0, "heading": 0.0}
})";
        file.close();
    }

    // Load the track
    Track original = Track::load_from_file(temp_path);

    // Verify metadata
    REQUIRE(original.metadata().name == "Test Track");
    REQUIRE(original.metadata().author == "Test Author");
    REQUIRE(original.metadata().description == "Test Description");
    REQUIRE(original.metadata().version == 1);

    // Verify geometry
    REQUIRE(original.points().size() == 10);

    // Save it back
    original.save_to_file(temp_path);

    // Load it again
    Track loaded = Track::load_from_file(temp_path);

    // Verify metadata preserved
    REQUIRE(loaded.metadata().name == "Test Track");
    REQUIRE(loaded.metadata().author == "Test Author");
    REQUIRE(loaded.metadata().description == "Test Description");
    REQUIRE(loaded.metadata().version == 1);

    // Verify geometry preserved
    REQUIRE(loaded.points().size() == 10);
    REQUIRE(loaded.length() == Approx(original.length()).margin(0.01));

    for (std::size_t i = 0; i < 10; ++i) {
        REQUIRE(loaded.points()[i].pos.x == Approx(original.points()[i].pos.x).margin(1e-6));
        REQUIRE(loaded.points()[i].pos.y == Approx(original.points()[i].pos.y).margin(1e-6));
        REQUIRE(loaded.points()[i].width == Approx(original.points()[i].width).margin(1e-6));
        REQUIRE(loaded.points()[i].kerb_width == Approx(original.points()[i].kerb_width).margin(1e-6));
        REQUIRE(loaded.points()[i].elevation == Approx(original.points()[i].elevation).margin(1e-6));
        REQUIRE(loaded.points()[i].banking == Approx(original.points()[i].banking).margin(1e-6));
        REQUIRE(loaded.points()[i].surface_left == original.points()[i].surface_left);
        REQUIRE(loaded.points()[i].surface_right == original.points()[i].surface_right);
    }

    // Verify objects preserved
    REQUIRE(loaded.objects().size() == 2);
    REQUIRE(loaded.objects()[0].type == TrackObject::Type::Cone);
    REQUIRE(loaded.objects()[0].position.x == Approx(5.0).margin(1e-6));
    REQUIRE(loaded.objects()[0].position.y == Approx(3.0).margin(1e-6));
    REQUIRE(loaded.objects()[1].type == TrackObject::Type::TireBarrier);
    REQUIRE(loaded.objects()[1].position.x == Approx(50.0).margin(1e-6));
    REQUIRE(loaded.objects()[1].rotation == Approx(1.57).margin(1e-6));

    // Verify pit lane preserved
    REQUIRE(loaded.pit_lane().entry_along == Approx(100.0).margin(1e-6));
    REQUIRE(loaded.pit_lane().exit_along == Approx(200.0).margin(1e-6));
    REQUIRE(loaded.pit_lane().speed_limit == Approx(22.222).margin(1e-3));

    // Verify start position and heading
    REQUIRE(loaded.start_position().x == Approx(original.start_position().x).margin(1e-6));
    REQUIRE(loaded.start_position().y == Approx(original.start_position().y).margin(1e-6));
    REQUIRE(loaded.start_heading() == Approx(original.start_heading()).margin(1e-6));

    // Clean up
    std::filesystem::remove(temp_path);
}

TEST_CASE("Load from file fails gracefully on missing file", "[track][io]") {
    REQUIRE_THROWS_AS(Track::load_from_file("nonexistent.track.json"), std::runtime_error);
}

TEST_CASE("Load from file fails on invalid JSON", "[track][io]") {
    const std::string temp_path = "test_invalid.track.json";
    std::ofstream file(temp_path);
    file << "{ invalid json }";
    file.close();

    REQUIRE_THROWS_AS(Track::load_from_file(temp_path), std::runtime_error);

    std::filesystem::remove(temp_path);
}

TEST_CASE("Load from file fails on unsupported version", "[track][io]") {
    const std::string temp_path = "test_version.track.json";
    std::ofstream file(temp_path);
    file << R"({"version": 999, "geometry": {"centerline": [{"x": 0, "y": 0}, {"x": 10, "y": 0}]}})";
    file.close();

    REQUIRE_THROWS_AS(Track::load_from_file(temp_path), std::runtime_error);

    std::filesystem::remove(temp_path);
}

TEST_CASE("Load from file fails on insufficient points", "[track][io]") {
    const std::string temp_path = "test_few_points.track.json";
    std::ofstream file(temp_path);
    file << R"({"version": 1, "geometry": {"centerline": [{"x": 0, "y": 0}]}})";
    file.close();

    REQUIRE_THROWS_AS(Track::load_from_file(temp_path), std::runtime_error);

    std::filesystem::remove(temp_path);
}

TEST_CASE("Track validation catches unclosed loop", "[track][validation]") {
    std::vector<TrackPoint> points;
    points.push_back({{0.0, 0.0}, 12.0, 1.5, 0.0, 0.0, Surface::Asphalt, Surface::Asphalt});
    points.push_back({{10.0, 0.0}, 12.0, 1.5, 0.0, 0.0, Surface::Asphalt, Surface::Asphalt});
    points.push_back({{20.0, 10.0}, 12.0, 1.5, 0.0, 0.0, Surface::Asphalt, Surface::Asphalt});
    // Not closed - last point far from first

    Track track(std::move(points));
    auto err = track.validate();
    REQUIRE(err.has_value());
    REQUIRE(err.value().find("does not close") != std::string::npos);
}

TEST_CASE("Track validation catches invalid width", "[track][validation]") {
    std::vector<TrackPoint> points;
    points.push_back({{0.0, 0.0}, 12.0, 1.5, 0.0, 0.0, Surface::Asphalt, Surface::Asphalt});
    points.push_back({{10.0, 0.0}, -5.0, 1.5, 0.0, 0.0, Surface::Asphalt, Surface::Asphalt}); // Invalid width
    points.push_back({{20.0, 0.0}, 12.0, 1.5, 0.0, 0.0, Surface::Asphalt, Surface::Asphalt});
    points.push_back({{10.0, 10.0}, 12.0, 1.5, 0.0, 0.0, Surface::Asphalt, Surface::Asphalt});
    points.push_back({{0.0, 10.0}, 12.0, 1.5, 0.0, 0.0, Surface::Asphalt, Surface::Asphalt});

    Track track(std::move(points));
    auto err = track.validate();
    REQUIRE(err.has_value());
    REQUIRE(err.value().find("Invalid width") != std::string::npos);
}

TEST_CASE("Track validation catches invalid kerb width", "[track][validation]") {
    std::vector<TrackPoint> points;
    points.push_back({{0.0, 0.0}, 12.0, -1.5, 0.0, 0.0, Surface::Asphalt, Surface::Asphalt}); // Invalid kerb
    points.push_back({{10.0, 0.0}, 12.0, 1.5, 0.0, 0.0, Surface::Asphalt, Surface::Asphalt});
    points.push_back({{20.0, 0.0}, 12.0, 1.5, 0.0, 0.0, Surface::Asphalt, Surface::Asphalt});
    points.push_back({{10.0, 10.0}, 12.0, 1.5, 0.0, 0.0, Surface::Asphalt, Surface::Asphalt});
    points.push_back({{0.0, 10.0}, 12.0, 1.5, 0.0, 0.0, Surface::Asphalt, Surface::Asphalt});

    Track track(std::move(points));
    auto err = track.validate();
    REQUIRE(err.has_value());
    REQUIRE(err.value().find("Invalid kerb width") != std::string::npos);
}

TEST_CASE("Valid track passes validation", "[track][validation]") {
    std::vector<TrackPoint> points;
    // Square track
    points.push_back({{0.0, 0.0}, 12.0, 1.5, 0.0, 0.0, Surface::Asphalt, Surface::Asphalt});
    points.push_back({{100.0, 0.0}, 12.0, 1.5, 0.0, 0.0, Surface::Asphalt, Surface::Asphalt});
    points.push_back({{100.0, 100.0}, 12.0, 1.5, 0.0, 0.0, Surface::Asphalt, Surface::Asphalt});
    points.push_back({{0.0, 100.0}, 12.0, 1.5, 0.0, 0.0, Surface::Asphalt, Surface::Asphalt});

    Track track(std::move(points));
    auto err = track.validate();
    REQUIRE(!err.has_value()); // Should be valid (no error)
}

TEST_CASE("Load example track files from data/tracks", "[track][io]") {
    // Test loading the demo track
    Track demo = Track::load_from_file("data/tracks/demo.track.json");
    REQUIRE(demo.length() > 0.0);
    REQUIRE(demo.metadata().name == "Demo Circuit");
    REQUIRE(demo.points().size() > 10);

    // Test loading the figure-8 track
    Track figure8 = Track::load_from_file("data/tracks/figure8.track.json");
    REQUIRE(figure8.length() > 0.0);
    REQUIRE(figure8.metadata().name == "Figure-8 Circuit");
    REQUIRE(figure8.points().size() > 10);

    // Test loading the hill track
    Track hill = Track::load_from_file("data/tracks/hill.track.json");
    REQUIRE(hill.length() > 0.0);
    REQUIRE(hill.metadata().name == "Hill Circuit");
    REQUIRE(hill.points().size() > 10);
    // Verify elevation changes
    bool has_elevation = false;
    for (const auto& p : hill.points()) {
        if (p.elevation != 0.0) {
            has_elevation = true;
            break;
        }
    }
    REQUIRE(has_elevation);
}