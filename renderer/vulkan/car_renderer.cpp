// SPDX-FileCopyrightText: 2026 OpenApex contributors
// SPDX-License-Identifier: GPL-3.0-or-later
#include "car_renderer.hpp"

#include <iostream>

namespace openapex::renderer::vulkan {

CarRenderer::~CarRenderer() {
    cleanup();
}

bool CarRenderer::init(Context& context, Pipeline&& car_pipeline) {
    context_ = &context;
    car_pipeline_ = std::move(car_pipeline);
    return true;
}

bool CarRenderer::load_model(const std::string& model_path) {
    // For now, create a simple box as placeholder car model
    // In the future, load from glTF using Model class
    std::cout << "Loading car model from: " << model_path << " (placeholder)\n";
    return true;
}

void CarRenderer::render(vk::CommandBuffer cmd, const Pipeline& pipeline,
                         const Camera& camera, const glm::mat4& model_matrix) {
    if (!model_) return;

    cmd.bindPipeline(vk::PipelineBindPoint::eGraphics, pipeline.pipeline);

    // Push model-view-projection matrix
    glm::mat4 mvp = camera.projection_matrix() * camera.view_matrix() * model_matrix;
    cmd.pushConstants(pipeline.layout, vk::ShaderStageFlagBits::eVertex, 0, sizeof(glm::mat4), &mvp);

    // Render model meshes
    model_->render(cmd, pipeline);
}

void CarRenderer::cleanup() {
    model_.reset();
}

} // namespace openapex::renderer::vulkan