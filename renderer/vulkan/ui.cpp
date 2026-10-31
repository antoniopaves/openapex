// SPDX-FileCopyrightText: 2026 OpenApex contributors
// SPDX-License-Identifier: GPL-3.0-or-later
#include "ui.hpp"

#include <imgui.h>
#include "backends/imgui_impl_sdl3.h"
#include "backends/imgui_impl_vulkan.h"

#include <vulkan/vulkan.hpp>

#include <iostream>

namespace openapex::renderer::vulkan {

UIRenderer::~UIRenderer() {
    cleanup();
}

bool UIRenderer::init(Context& context, SDL_Window* window) {
    context_ = &context;

    // Setup Dear ImGui context
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;

    // Setup ImGui style
    ImGui::StyleColorsDark();

    // Setup Platform/Renderer backends
    if (!ImGui_ImplSDL3_InitForVulkan(window)) {
        std::cerr << "Failed to initialize ImGui SDL3 backend\n";
        return false;
    }

    // Create descriptor pool for ImGui
    vk::DescriptorPoolSize pool_sizes[] = {
        {vk::DescriptorType::eSampler, 1000},
        {vk::DescriptorType::eCombinedImageSampler, 1000},
        {vk::DescriptorType::eSampledImage, 1000},
        {vk::DescriptorType::eStorageImage, 1000},
        {vk::DescriptorType::eUniformTexelBuffer, 1000},
        {vk::DescriptorType::eStorageTexelBuffer, 1000},
        {vk::DescriptorType::eUniformBuffer, 1000},
        {vk::DescriptorType::eStorageBuffer, 1000},
        {vk::DescriptorType::eUniformBufferDynamic, 1000},
        {vk::DescriptorType::eStorageBufferDynamic, 1000},
        {vk::DescriptorType::eInputAttachment, 1000}
    };

    vk::DescriptorPoolCreateInfo pool_info{};
    pool_info.flags = vk::DescriptorPoolCreateFlagBits::eFreeDescriptorSet;
    pool_info.maxSets = 1000;
    pool_info.poolSizeCount = static_cast<uint32_t>(std::size(pool_sizes));
    pool_info.pPoolSizes = pool_sizes;

    try {
        imgui_descriptor_pool_ = context.get().device.createDescriptorPool(pool_info);
    } catch (const vk::SystemError& err) {
        std::cerr << "Failed to create ImGui descriptor pool: " << err.what() << "\n";
        return false;
    }

    // Setup ImGui Vulkan backend
    ImGui_ImplVulkan_InitInfo init_info{};
    init_info.ApiVersion = VK_API_VERSION_1_3;
    init_info.Instance = context.get().instance;
    init_info.PhysicalDevice = context.get().physical_device;
    init_info.Device = context.get().device;
    init_info.QueueFamily = context.get().queue_indices.graphics_family.value();
    init_info.Queue = context.get().graphics_queue;
    init_info.DescriptorPool = imgui_descriptor_pool_;
    init_info.MinImageCount = context.get().swapchain.image_count;
    init_info.ImageCount = context.get().swapchain.image_count;
    init_info.UseDynamicRendering = false;

    // Pipeline info (replaces old RenderPass, Subpass, MSAASamples)
    init_info.PipelineInfoMain.RenderPass = context.get().render_pass;
    init_info.PipelineInfoMain.Subpass = 0;
    init_info.PipelineInfoMain.MSAASamples = VK_SAMPLE_COUNT_1_BIT;

    if (!ImGui_ImplVulkan_Init(&init_info)) {
        std::cerr << "Failed to initialize ImGui Vulkan backend\n";
        return false;
    }

    // Font texture is created automatically in newer ImGui versions
    // No need for ImGui_ImplVulkan_CreateFontsTexture

    std::cout << "UI renderer initialized\n";
    return true;
}

void UIRenderer::render(vk::CommandBuffer cmd, const UIState& state) {
    ImGui_ImplVulkan_NewFrame();
    ImGui_ImplSDL3_NewFrame();
    ImGui::NewFrame();

    // Telemetry window
    ImGui::Begin("Telemetry", nullptr, ImGuiWindowFlags_AlwaysAutoResize);

    ImGui::Text("Speed: %.1f km/h", state.speed_kmh);
    ImGui::Text("RPM: %.0f", state.rpm);
    ImGui::Text("Gear: %d", state.gear);

    ImGui::Separator();

    ImGui::Text("ABS: %s", state.abs_enabled ? "ON" : "OFF");
    ImGui::Text("TC: %s", state.tc_enabled ? "ON" : "OFF");
    const char* diff_names[] = {"Open", "Viscous", "Clutch", "Torsen"};
    ImGui::Text("LSD: %s", diff_names[state.diff_type]);

    ImGui::Separator();

    ImGui::Text("ABS Active: FL:%d FR:%d RL:%d RR:%d",
                state.abs_active[0], state.abs_active[1], state.abs_active[2], state.abs_active[3]);
    ImGui::Text("TC Active: %d", state.tc_active);
    ImGui::Text("Diff Lock Torque: %.1f Nm", state.diff_lock_torque);
    ImGui::Text("Drive Torque RL: %.1f RR: %.1f", state.drive_torque_rl, state.drive_torque_rr);

    ImGui::Separator();

    ImGui::Text("Wheel Slip: FL:%.3f FR:%.3f RL:%.3f RR:%.3f",
                state.wheel_slip[0], state.wheel_slip[1], state.wheel_slip[2], state.wheel_slip[3]);

    ImGui::Separator();

    ImGui::Text("Input: T:%.2f B:%.2f S:%.2f", state.throttle, state.brake, state.steer);

    ImGui::Separator();

    ImGui::Text("Lap Time: %.2f", state.lap_time);
    ImGui::Text("Best Lap: %.2f", state.best_lap);

    ImGui::End();

    // Controls help window
    ImGui::Begin("Controls", nullptr, ImGuiWindowFlags_AlwaysAutoResize);
    ImGui::Text("WASD / Arrows: Drive");
    ImGui::Text("A: Toggle ABS");
    ImGui::Text("C: Toggle TC");
    ImGui::Text("D: Cycle LSD Type");
    ImGui::Text("ESC: Quit");
    ImGui::End();

    // Render ImGui
    ImGui::Render();
    ImGui_ImplVulkan_RenderDrawData(ImGui::GetDrawData(), cmd);
}

void UIRenderer::process_event(const SDL_Event& event) {
    ImGui_ImplSDL3_ProcessEvent(&event);
}

void UIRenderer::cleanup() {
    if (context_) {
        ImGui_ImplVulkan_Shutdown();
        ImGui_ImplSDL3_Shutdown();
        ImGui::DestroyContext();

        context_->get().device.destroyDescriptorPool(imgui_descriptor_pool_);
    }
}

} // namespace openapex::renderer::vulkan