# SPDX-FileCopyrightText: 2026 OpenApex contributors
# SPDX-License-Identifier: GPL-3.0-or-later
find_package(fmt CONFIG REQUIRED)
find_package(glm CONFIG REQUIRED)
find_package(nlohmann_json CONFIG REQUIRED)

add_library(sim-core src/tire.cpp src/vehicle.cpp src/track.cpp)
add_library(openapex::sim-core ALIAS sim-core)
target_include_directories(sim-core PUBLIC include)
target_link_libraries(sim-core PUBLIC glm::glm PRIVATE fmt::fmt nlohmann_json::nlohmann_json)

if(OPENAPEX_BUILD_TESTS)
    find_package(Catch2 3 CONFIG REQUIRED)
    add_executable(sim-core-tests tests/test_tire.cpp tests/test_vehicle.cpp tests/test_track.cpp)
    target_link_libraries(sim-core-tests PRIVATE sim-core Catch2::Catch2WithMain)
    include(CTest)
    include(Catch)
    catch_discover_tests(sim-core-tests WORKING_DIRECTORY ${PROJECT_SOURCE_DIR})
endif()
