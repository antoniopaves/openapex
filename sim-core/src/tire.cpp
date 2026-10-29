// SPDX-FileCopyrightText: 2026 OpenApex contributors
// SPDX-License-Identifier: GPL-3.0-or-later
#include "openapex/sim/tire.hpp"

#include <cmath>

namespace openapex::sim {

double pacejka_normalized(const PacejkaCoefficients& c, double slip) noexcept {
    const double bx = c.B * slip;
    return c.D * std::sin(c.C * std::atan(bx - c.E * (bx - std::atan(bx))));
}

double lateral_force(const PacejkaCoefficients& c, double slip_angle,
                     double normal_load) noexcept {
    return normal_load * pacejka_normalized(c, slip_angle);
}

} // namespace openapex::sim
