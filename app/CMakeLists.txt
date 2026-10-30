# SPDX-FileCopyrightText: 2026 OpenApex contributors
# SPDX-License-Identifier: GPL-3.0-or-later
find_package(SDL3 CONFIG REQUIRED)

add_executable(openapex main.cpp)
target_link_libraries(openapex PRIVATE openapex::sim-core SDL3::SDL3)
