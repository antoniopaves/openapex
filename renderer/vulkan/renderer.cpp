// SPDX-FileCopyrightText: 2026 OpenApex contributors
// SPDX-License-Identifier: GPL-3.0-or-later
#include "renderer.hpp"

#include <openapex/sim/track.hpp>
#include <openapex/sim/vehicle.hpp>

#include <iostream>

namespace openapex::renderer::vulkan {

Renderer::~Renderer() {
    wait_idle();
}

bool Renderer::init(SDL_Window* window) {
    // Initialize Vulkan context
    if (!context_.init(window)) {
        return false;
    }

    // Create track pipeline
    Pipeline::Config track_config;
    track_config.vert_shader_path = "shaders/track.vert.spv";
    track_config.frag_shader_path = "shaders/track.frag.spv";
    track_config.vertex_bindings = {
        vk::VertexInputBindingDescription{0, sizeof(TrackVertex), vk::VertexInputRate::eVertex}
    };
    track_config.vertex_attributes = {
        vk::VertexInputAttributeDescription{0, 0, vk::Format::eR32G32B32Sfloat, offsetof(TrackVertex, position)},
        vk::VertexInputAttributeDescription{1, 0, vk::Format::eR32G32B32Sfloat, offsetof(TrackVertex, normal)},
        vk::VertexInputAttributeDescription{2, 0, vk::Format::eR32G32Sfloat, offsetof(TrackVertex, uv)},
        vk::VertexInputAttributeDescription{3, 0, vk::Format::eR32Uint, offsetof(TrackVertex, surface_type)}
    };
    track_config.topology = vk::PrimitiveTopology::eTriangleList;

    if (!track_pipeline_.create(context_, context_.render_pass(), track_config)) {
        return false;
    }

    // Create car pipeline
    Pipeline::Config car_config;
    car_config.vert_shader_path = "shaders/car.vert.spv";
    car_config.frag_shader_path = "shaders/car.frag.spv";
    car_config.vertex_bindings = {
        vk::VertexInputBindingDescription{0, sizeof(ModelVertex), vk::VertexInputRate::eVertex}
    };
    car_config.vertex_attributes = {
        vk::VertexInputAttributeDescription{0, 0, vk::Format::eR32G32B32Sfloat, offsetof(ModelVertex, position)},
        vk::VertexInputAttributeDescription{1, 0, vk::Format::eR32G32B32Sfloat, offsetof(ModelVertex, normal)},
        vk::VertexInputAttributeDescription{2, 0, vk::Format::eR32G32Sfloat, offsetof(ModelVertex, uv)}
    };
    car_config.topology = vk::PrimitiveTopology::eTriangleList;

    if (!car_pipeline_.create(context_, context_.render_pass(), car_config)) {
        return false;
    }

    // Initialize sub-renderers
    if (!track_renderer_.init(context_, track_pipeline_)) {
        return false;
    }
    if (!car_renderer_.init(context_, std::move(car_pipeline_))) {
        return false;
    }
    if (!ui_renderer_.init(context_, window)) {
        return false;
    }

    // Build track mesh (placeholder - will be called with actual track later)
    // track_renderer_.build_track_mesh(track);

    // Load car model (placeholder)
    // car_renderer_.load_model("models/car.gltf");

    std::cout << "Vulkan renderer initialized successfully\n";
    return true;
}

bool Renderer::render_frame(
    const openapex::sim::Track& track,
    const openapex::sim::VehicleState& vehicle_state,
    const openapex::sim::VehicleParams& vehicle_params,
    const Camera& camera,
    const UIState& ui_state) {

    // Begin frame
    if (!context_.begin_frame()) {
        // Swapchain needs recreation
        return false;
    }

    vk::CommandBuffer cmd = context_.command_buffer();

    // Begin render pass
    vk::RenderPassBeginInfo render_pass_info{};
    render_pass_info.renderPass = context_.render_pass();
    render_pass_info.framebuffer = context_.get().swapchain.framebuffers[context_.current_frame()];
    render_pass_info.renderArea.offset = vk::Offset2D{0, 0};
    render_pass_info.renderArea.extent = context_.extent();

    vk::ClearValue clear_color{};
    clear_color.color = vk::ClearColorValue(std::array<float, 4>{0.1f, 0.1f, 0.15f, 1.0f});
    render_pass_info.clearValueCount = 1;
    render_pass_info.pClearValues = &clear_color;

    cmd.beginRenderPass(render_pass_info, vk::SubpassContents::eInline);

    // Set viewport and scissor
    vk::Viewport viewport{};
    viewport.x = 0.0f;
    viewport.y = 0.0f;
    viewport.width = static_cast<float>(context_.extent().width);
    viewport.height = static_cast<float>(context_.extent().height);
    viewport.minDepth = 0.0f;
    viewport.maxDepth = 1.0f;
    cmd.setViewport(0, 1, &viewport);

    vk::Rect2D scissor{};
    scissor.offset = vk::Offset2D{0, 0};
    scissor.extent = context_.extent();
    cmd.setScissor(0, 1, &scissor);

    // Render track
    track_renderer_.render(cmd, track_pipeline_, camera);

    // Render car
    // Calculate car model matrix from vehicle state
    glm::mat4 car_model = glm::mat4(1.0f);
    car_model = glm::translate(car_model, glm::vec3(vehicle_state.x, 0.5f, vehicle_state.y));
    car_model = glm::rotate(car_model, static_cast<float>(vehicle_state.yaw), glm::vec3(0.0f, 1.0f, 0.0f));
    car_renderer_.render(cmd, car_pipeline_, camera, car_model);

    // Render UI
    ui_renderer_.render(cmd, ui_state);

    // End render pass
    cmd.endRenderPass();

    // End frame
    context_.end_frame();

    return true;
}

void Renderer::on_resize(SDL_Window* window) {
    context_.recreate_swapchain(window);
    // Recreate framebuffers for new swapchain size
    // This would need to be implemented
}

void Renderer::wait_idle() const {
    context_.wait_idle();
}

} // namespace openapex::renderer::vulkan