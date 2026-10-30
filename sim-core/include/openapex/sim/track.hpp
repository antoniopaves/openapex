// SPDX-FileCopyrightText: 2026 OpenApex contributors
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <cstddef>
#include <optional>
#include <string>
#include <vector>

namespace openapex::sim {

struct Vec2 {
    double x = 0.0;
    double y = 0.0;
};

struct Vec3 {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
};

enum class Surface { Asphalt, Kerb, Grass };

/// Multiplicador de agarre de cada superficie (se aplica al pico D del neumático).
[[nodiscard]] double surface_grip(Surface surface) noexcept;

struct TrackQuery {
    double lateral = 0.0; ///< Distancia al eje de la pista en m (positivo = izquierda)
    double along = 0.0;   ///< Distancia recorrida desde la línea de meta en m
    double heading = 0.0; ///< Dirección de la pista en rad
    Surface surface = Surface::Grass;
    double elevation = 0.0;   ///< Elevación en el punto (m)
    double banking = 0.0;     ///< Inclinación lateral en rad
};

// Estructura para un punto de la línea central con propiedades por segmento
struct TrackPoint {
    Vec2 pos;             // Posición 2D (x, y)
    double width = 12.0;  // Ancho total de la pista en este punto
    double kerb_width = 1.5; // Ancho del piano
    double elevation = 0.0;  // Elevación (m)
    double banking = 0.0;    // Inclinación lateral (rad)
    Surface surface_left = Surface::Asphalt;
    Surface surface_right = Surface::Asphalt;
};

// Metadatos de la pista
struct TrackMetadata {
    std::string name;
    std::string author;
    std::string description;
    int version = 1;
};

// Objeto estático en la pista (conos, barreras, etc.)
struct TrackObject {
    enum class Type { Cone, TireBarrier, ArmcoBarrier, Grandstand, Building, Tree } type = Type::Cone;
    Vec2 position;
    double rotation = 0.0; // rad
    double scale = 1.0;
};

// Información del pit lane
struct PitLaneInfo {
    double entry_along = 0.0;   // Distancia along de la entrada
    double exit_along = 0.0;    // Distancia along de la salida
    double speed_limit = 80.0 / 3.6; // m/s (80 km/h por defecto)
};

/// Pista cerrada definida por la línea central con propiedades por segmento.
/// Soporta carga/guardado desde archivo JSON.
class Track {
public:
    // Constructor desde puntos con propiedades por segmento
    explicit Track(std::vector<TrackPoint> points);

    // Constructor legacy (compatibilidad)
    Track(std::vector<Vec2> centerline, double width, double kerb_width = 0.8);

    /// Circuito de demostración generado por código.
    [[nodiscard]] static Track make_demo();

    /// Carga una pista desde archivo JSON.
    /// Lanza std::runtime_error si hay error de formato o validación.
    [[nodiscard]] static Track load_from_file(const std::string& path);

    /// Guarda la pista en archivo JSON.
    void save_to_file(const std::string& path) const;

    [[nodiscard]] TrackQuery query(double x, double y) const noexcept;

    [[nodiscard]] double length() const noexcept { return length_; }
    [[nodiscard]] double width_at(std::size_t index) const noexcept;
    [[nodiscard]] double kerb_width_at(std::size_t index) const noexcept;
    [[nodiscard]] const std::vector<TrackPoint>& points() const noexcept { return pts_; }
    [[nodiscard]] const std::vector<Vec2>& centerline() const noexcept;
    [[nodiscard]] const TrackMetadata& metadata() const noexcept { return meta_; }
    [[nodiscard]] const std::vector<TrackObject>& objects() const noexcept { return objects_; }
    [[nodiscard]] const PitLaneInfo& pit_lane() const noexcept { return pit_lane_; }

    [[nodiscard]] Vec2 start_position() const;
    [[nodiscard]] double start_heading() const;

    // Valida la pista (continuidad, bucle cerrado, etc.)
    // Devuelve vacío si es válida, o mensaje de error si no
    [[nodiscard]] std::optional<std::string> validate() const noexcept;

private:
    std::vector<TrackPoint> pts_;
    std::vector<double> cumulative_; // Distancia acumulada en cada punto
    std::vector<double> segment_lengths_; // Longitud de cada segmento
    TrackMetadata meta_;
    std::vector<TrackObject> objects_;
    PitLaneInfo pit_lane_;
    std::vector<Vec2> centerline_; // cache de pts_[i].pos
    double length_ = 0.0;

    void rebuild_cumulative() noexcept;
    [[nodiscard]] Surface surface_at(double lateral, std::size_t segment_idx) const noexcept;
};

} // namespace openapex::sim