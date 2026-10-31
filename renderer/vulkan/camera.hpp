// SPDX-FileCopyrightText: 2026 OpenApex contributors
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

namespace openapex::renderer::vulkan {

struct Camera {
    glm::vec3 position{0.0f, 5.0f, 10.0f};
    glm::vec3 target{0.0f, 0.0f, 0.0f};
    glm::vec3 up{0.0f, 1.0f, 0.0f};

    float fov = glm::radians(60.0f);
    float aspect = 16.0f / 9.0f;
    float near_plane = 0.1f;
    float far_plane = 1000.0f;

    // Camera modes
    enum class Mode { Free, Follow, Chase } mode = Mode::Follow;

    // Follow camera parameters
    float follow_distance = 15.0f;
    float follow_height = 5.0f;
    float follow_smoothness = 0.1f;

    // View matrix
    [[nodiscard]] glm::mat4 view_matrix() const noexcept {
        return glm::lookAt(position, target, up);
    }

    // Projection matrix
    [[nodiscard]] glm::mat4 projection_matrix() const noexcept {
        return glm::perspective(fov, aspect, near_plane, far_plane);
    }

    // View-projection matrix
    [[nodiscard]] glm::mat4 view_projection_matrix() const noexcept {
        return projection_matrix() * view_matrix();
    }

    // Update camera for follow mode
    void update_follow(const glm::vec3& vehicle_pos, const glm::vec3& vehicle_forward, float dt);
};

} // namespace openapex::renderer::vulkan