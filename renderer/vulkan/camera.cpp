// SPDX-FileCopyrightText: 2026 OpenApex contributors
// SPDX-License-Identifier: GPL-3.0-or-later
#include "camera.hpp"

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/quaternion.hpp>
#include <glm/gtx/vector_angle.hpp>

namespace openapex::renderer::vulkan {

void Camera::update_follow(const glm::vec3& vehicle_pos, const glm::vec3& vehicle_forward, float dt) {
    if (mode != Mode::Follow) return;

    // Desired camera position behind and above the vehicle
    glm::vec3 desired_pos = vehicle_pos - vehicle_forward * follow_distance + glm::vec3(0.0f, follow_height, 0.0f);

    // Smoothly interpolate camera position
    position = glm::mix(position, desired_pos, follow_smoothness * dt * 60.0f);

    // Look at a point slightly ahead of the vehicle
    target = vehicle_pos + vehicle_forward * 10.0f;
}

} // namespace openapex::renderer::vulkan