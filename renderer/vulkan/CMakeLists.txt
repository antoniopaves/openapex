# SPDX-FileCopyrightText: 2026 OpenApex contributors
# SPDX-License-Identifier: GPL-3.0-or-later

cmake_minimum_required(VERSION 3.25)
project(openapex-vulkan LANGUAGES CXX)

# Vulkan dependencies
find_package(Vulkan REQUIRED)
find_package(imgui REQUIRED)
find_package(fmt CONFIG REQUIRED)

# tinygltf is header-only, use include directory
find_path(TINYGLTF_INCLUDE_DIRS tiny_gltf.h
    PATHS ${CMAKE_BINARY_DIR}/vcpkg_installed/x64-linux/include
    PATH_SUFFIXES tinygltf
)

# Renderer library
add_library(openapex-vulkan STATIC
    context.cpp
    renderer.cpp
    track_renderer.cpp
    car_renderer.cpp
    ui.cpp
    camera.cpp
    pipeline.cpp
    model.cpp
)

# Create alias for namespace-style access
add_library(openapex::vulkan ALIAS openapex-vulkan)

target_include_directories(openapex-vulkan PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/..
    ${CMAKE_SOURCE_DIR}/sim-core/include
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
)

target_include_directories(openapex-vulkan PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/shaders
    ${CMAKE_CURRENT_SOURCE_DIR}/backends
)

if(TINYGLTF_INCLUDE_DIRS)
    target_include_directories(openapex-vulkan PRIVATE ${TINYGLTF_INCLUDE_DIRS})
endif()

# Find dependencies
find_package(glm CONFIG REQUIRED)

target_link_libraries(openapex-vulkan PUBLIC
    Vulkan::Vulkan
    imgui::imgui
    openapex::sim-core
    glm::glm
    fmt::fmt
)

target_compile_features(openapex-vulkan PUBLIC cxx_std_20)

# Shader compilation with glslang (optional, for SPIR-V)
# For now, we'll pre-compile shaders or include them as strings

# Install headers
install(FILES
    context.hpp
    renderer.hpp
    track_renderer.hpp
    car_renderer.hpp
    ui.hpp
    camera.hpp
    pipeline.hpp
    model.hpp
    DESTINATION include/openapex/renderer/vulkan
)

# Shader files (will be handled by build system)
file(GLOB SHADER_FILES shaders/*.vert shaders/*.frag)
if(SHADER_FILES)
    install(FILES ${SHADER_FILES} DESTINATION share/openapex/shaders)
endif()