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
add_subdirectory(app)
