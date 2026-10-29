// SPDX-FileCopyrightText: 2026 OpenApex contributors
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

namespace openapex::sim {

/// Coeficientes de la Magic Formula de Pacejka (forma simplificada).
struct PacejkaCoefficients {
    double B = 10.0; ///< Rigidez
    double C = 1.9;  ///< Forma
    double D = 1.0;  ///< Pico (coeficiente de fricción efectivo)
    double E = 0.97; ///< Curvatura
};

/// Fuerza normalizada (adimensional, en [-D, D]) para un deslizamiento dado.
[[nodiscard]] double pacejka_normalized(const PacejkaCoefficients& c, double slip) noexcept;

/// Fuerza lateral en newtons. slip_angle en radianes, normal_load en newtons.
[[nodiscard]] double lateral_force(const PacejkaCoefficients& c, double slip_angle,
                                   double normal_load) noexcept;

} // namespace openapex::sim
