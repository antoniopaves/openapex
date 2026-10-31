// SPDX-FileCopyrightText: 2026 OpenApex contributors
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include "context.hpp"
#include "pipeline.hpp"

#include <vulkan/vulkan.hpp>

#include <SDL3/SDL.h>

#include <memory>

namespace openapex::renderer::vulkan {

struct UIState {
    // Telemetry data for UI
    float speed_kmh = 0.0f;
    float rpm = 0.0f;
    int gear = 1;
    bool abs_enabled = true;
    bool tc_enabled = true;
    int diff_type = 0; // 0=Open, 1=Viscous, 2=Clutch, 3=Torsen
    bool abs_active[4] = {false, false, false, false};
    bool tc_active = false;
    float diff_lock_torque = 0.0f;
    float drive_torque_rl = 0.0f;
    float drive_torque_rr = 0.0f;
    float wheel_slip[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    float throttle = 0.0f;
    float brake = 0.0f;
    float steer = 0.0f;
    float lap_time = 0.0f;
    float best_lap = 0.0f;
};

class UIRenderer {
public:
    UIRenderer() = default;
    ~UIRenderer();

    UIRenderer(const UIRenderer&) = delete;
    UIRenderer& operator=(const UIRenderer&) = delete;
    UIRenderer(UIRenderer&&) noexcept = default;
    UIRenderer& operator=(UIRenderer&&) noexcept = default;

    // Initialize UI renderer (ImGui + Vulkan backend)
    bool init(Context& context, SDL_Window* window);

    // Render UI
    void render(vk::CommandBuffer cmd, const UIState& state);

    // Handle SDL events for ImGui
    void process_event(const SDL_Event& event);

    // Cleanup
    void cleanup();

private:
    Context* context_ = nullptr;
    vk::DescriptorPool imgui_descriptor_pool_;
    // ImGui context is global, managed by ImGui itself
};

} // namespace openapex::renderer::vulkan