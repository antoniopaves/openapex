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

/// Fuerzas del neumático en su propio sistema: x hacia delante, y a la izquierda.
struct TireForce {
    double fx = 0.0;
    double fy = 0.0;
};

/// Fuerza con slip combinado. kappa = slip ratio, tan_alpha = tangente del slip angle.
/// La magnitud total nunca supera D * normal_load (círculo de fricción).
[[nodiscard]] TireForce combined_slip_force(const PacejkaCoefficients& c, double kappa,
                                            double tan_alpha, double normal_load) noexcept;

} // namespace openapex::sim
