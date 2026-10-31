// SPDX-FileCopyrightText: 2026 OpenApex contributors
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include "context.hpp"
#include "pipeline.hpp"
#include "model.hpp"
#include "camera.hpp"

#include <openapex/sim/vehicle.hpp>

#include <vulkan/vulkan.hpp>

#include <memory>

namespace openapex::renderer::vulkan {

class CarRenderer {
public:
    CarRenderer() = default;
    ~CarRenderer();

    CarRenderer(const CarRenderer&) = delete;
    CarRenderer& operator=(const CarRenderer&) = delete;
    CarRenderer(CarRenderer&&) noexcept = default;
    CarRenderer& operator=(CarRenderer&&) noexcept = default;

    // Initialize car renderer
    bool init(Context& context, Pipeline&& car_pipeline);

    // Load car model
    bool load_model(const std::string& model_path);

    // Render car at given transform
    void render(vk::CommandBuffer cmd, const Pipeline& pipeline,
                const Camera& camera, const glm::mat4& model_matrix);

    // Cleanup
    void cleanup();

private:
    Context* context_ = nullptr;
    std::unique_ptr<Model> model_;
    Pipeline car_pipeline_;
};

} // namespace openapex::renderer::vulkan