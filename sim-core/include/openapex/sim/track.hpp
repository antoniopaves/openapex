// SPDX-FileCopyrightText: 2026 OpenApex contributors
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <cstddef>
#include <vector>

namespace openapex::sim {

struct Vec2 {
    double x = 0.0;
    double y = 0.0;
};

enum class Surface { Asphalt, Kerb, Grass };

/// Multiplicador de agarre de cada superficie (se aplica al pico D del neumático).
[[nodiscard]] double surface_grip(Surface surface) noexcept;

struct TrackQuery {
    double lateral = 0.0; ///< Distancia al eje de la pista en m (positivo = izquierda)
    double along = 0.0;   ///< Distancia recorrida desde la línea de meta en m
    double heading = 0.0; ///< Dirección de la pista en rad
    Surface surface = Surface::Grass;
};

/// Pista cerrada definida por la línea central y un ancho. Más adelante se cargará de archivo.
class Track {
public:
    Track(std::vector<Vec2> centerline, double width, double kerb_width = 0.8);

    /// Circuito de demostración generado por código.
    [[nodiscard]] static Track make_demo();

    [[nodiscard]] TrackQuery query(double x, double y) const noexcept;

    [[nodiscard]] double length() const noexcept { return length_; }
    [[nodiscard]] double width() const noexcept { return width_; }
    [[nodiscard]] double kerb_width() const noexcept { return kerb_; }
    [[nodiscard]] const std::vector<Vec2>& centerline() const noexcept { return pts_; }

    [[nodiscard]] Vec2 start_position() const;
    [[nodiscard]] double start_heading() const;

private:
    std::vector<Vec2> pts_;
    std::vector<double> cumulative_;
    double width_;
    double kerb_;
    double length_ = 0.0;
};

} // namespace openapex::sim