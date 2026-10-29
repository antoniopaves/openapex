#!/usr/bin/env bash
set -euo pipefail

cat > vcpkg.json <<'EOF'
{
  "name": "openapex",
  "version": "0.1.0",
  "dependencies": ["fmt", "glm", "catch2"]
}
EOF

cat > .gitignore <<'EOF'
build/
.cache/
vcpkg_installed/
compile_commands.json
*.user
.idea/
.vscode/
EOF

cat > .editorconfig <<'EOF'
root = true
[*]
charset = utf-8
end_of_line = lf
indent_style = space
indent_size = 4
insert_final_newline = true
trim_trailing_whitespace = true
[*.{json,yml,yaml,toml}]
indent_size = 2
EOF

cat > .clang-format <<'EOF'
BasedOnStyle: LLVM
IndentWidth: 4
ColumnLimit: 100
BreakBeforeBraces: Attach
AllowShortFunctionsOnASingleLine: Inline
PointerAlignment: Left
EOF

cat > REUSE.toml <<'EOF'
version = 1

[[annotations]]
path = ["vcpkg.json", "CMakePresets.json", ".gitignore", ".editorconfig", ".clang-format", "README.md", "CONTRIBUTING.md", "LICENSE", "bootstrap.sh"]
precedence = "aggregate"
SPDX-FileCopyrightText = "2026 OpenApex contributors"
SPDX-License-Identifier = "GPL-3.0-or-later"
EOF

cat > CMakeLists.txt <<'EOF'
# SPDX-FileCopyrightText: 2026 OpenApex contributors
# SPDX-License-Identifier: GPL-3.0-or-later
cmake_minimum_required(VERSION 3.29)
project(openapex VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

option(OPENAPEX_BUILD_TESTS "Build unit tests" ON)

add_compile_options(-Wall -Wextra -Wpedantic -Wconversion -Wshadow)

if(OPENAPEX_BUILD_TESTS)
    enable_testing()
endif()

add_subdirectory(sim-core)
EOF

cat > CMakePresets.json <<'EOF'
{
  "version": 6,
  "configurePresets": [
    {
      "name": "base",
      "hidden": true,
      "generator": "Ninja",
      "binaryDir": "${sourceDir}/build/${presetName}",
      "toolchainFile": "$env{VCPKG_ROOT}/scripts/buildsystems/vcpkg.cmake",
      "cacheVariables": {
        "CMAKE_CXX_COMPILER": "clang++",
        "CMAKE_CXX_COMPILER_LAUNCHER": "ccache",
        "CMAKE_LINKER_TYPE": "MOLD"
      }
    },
    { "name": "debug", "inherits": "base",
      "cacheVariables": { "CMAKE_BUILD_TYPE": "Debug" } },
    { "name": "release", "inherits": "base",
      "cacheVariables": { "CMAKE_BUILD_TYPE": "RelWithDebInfo" } },
    { "name": "asan", "inherits": "base",
      "cacheVariables": {
        "CMAKE_BUILD_TYPE": "Debug",
        "CMAKE_CXX_FLAGS": "-fsanitize=address,undefined -fno-omit-frame-pointer"
      } }
  ],
  "buildPresets": [
    { "name": "debug", "configurePreset": "debug" },
    { "name": "release", "configurePreset": "release" },
    { "name": "asan", "configurePreset": "asan" }
  ],
  "testPresets": [
    { "name": "debug", "configurePreset": "debug",
      "output": { "outputOnFailure": true } },
    { "name": "release", "configurePreset": "release",
      "output": { "outputOnFailure": true } },
    { "name": "asan", "configurePreset": "asan",
      "output": { "outputOnFailure": true } }
  ]
}
EOF

cat > sim-core/CMakeLists.txt <<'EOF'
# SPDX-FileCopyrightText: 2026 OpenApex contributors
# SPDX-License-Identifier: GPL-3.0-or-later
find_package(fmt CONFIG REQUIRED)
find_package(glm CONFIG REQUIRED)

add_library(sim-core src/tire.cpp)
add_library(openapex::sim-core ALIAS sim-core)
target_include_directories(sim-core PUBLIC include)
target_link_libraries(sim-core PUBLIC glm::glm PRIVATE fmt::fmt)

if(OPENAPEX_BUILD_TESTS)
    find_package(Catch2 3 CONFIG REQUIRED)
    add_executable(sim-core-tests tests/test_tire.cpp)
    target_link_libraries(sim-core-tests PRIVATE sim-core Catch2::Catch2WithMain)
    include(CTest)
    include(Catch)
    catch_discover_tests(sim-core-tests)
endif()
EOF

cat > sim-core/include/openapex/sim/tire.hpp <<'EOF'
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
EOF

cat > sim-core/src/tire.cpp <<'EOF'
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
EOF

cat > sim-core/tests/test_tire.cpp <<'EOF'
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
EOF

cat > README.md <<'EOF'
# OpenApex

Simulador de carreras open source (GPLv3) con física de primer nivel, modding abierto y servidores libres.
Estado: prototipo temprano (sim-core).

## Compilar

    cmake --preset debug
    cmake --build --preset debug
    ctest --preset debug

Requiere clang, CMake >= 3.29, Ninja y vcpkg (`VCPKG_ROOT` definido).
EOF

cat > CONTRIBUTING.md <<'EOF'
# Contribuir

- Licencia: GPL-3.0-or-later. Cada archivo lleva cabecera SPDX (`reuse lint`).
- Firma tus commits con `git commit -s` (Developer Certificate of Origin).
- Formato: `clang-format`. Todo cambio en física necesita un test.
EOF

echo "Archivos creados."
