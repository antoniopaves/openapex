// SPDX-FileCopyrightText: 2026 OpenApex contributors
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <vulkan/vulkan.hpp>
#include <SDL3/SDL_vulkan.h>

#include <string>
#include <vector>
#include <optional>

namespace openapex::renderer::vulkan {

struct SwapchainInfo {
    vk::Extent2D extent;
    vk::Format format;
    vk::ColorSpaceKHR color_space;
    vk::PresentModeKHR present_mode;
    uint32_t image_count;
    std::vector<vk::Image> images;
    std::vector<vk::ImageView> image_views;
    std::vector<vk::Framebuffer> framebuffers;
};

struct QueueFamilyIndices {
    std::optional<uint32_t> graphics_family;
    std::optional<uint32_t> present_family;

    [[nodiscard]] bool is_complete() const noexcept {
        return graphics_family.has_value() && present_family.has_value();
    }
};

struct VulkanContext {
    vk::Instance instance;
    vk::PhysicalDevice physical_device;
    vk::Device device;
    vk::Queue graphics_queue;
    vk::Queue present_queue;
    vk::SurfaceKHR surface;
    SwapchainInfo swapchain;
    vk::RenderPass render_pass;
    vk::CommandPool command_pool;
    std::vector<vk::CommandBuffer> command_buffers;
    std::vector<vk::Semaphore> image_available_semaphores;
    std::vector<vk::Semaphore> render_finished_semaphores;
    std::vector<vk::Fence> in_flight_fences;
    uint32_t current_frame = 0;
    QueueFamilyIndices queue_indices;

    // Debug messenger (only in debug builds)
#ifdef NDEBUG
    static constexpr bool enable_validation_layers = false;
#else
    static constexpr bool enable_validation_layers = true;
#endif

    static constexpr uint32_t max_frames_in_flight = 2;

    // Required extensions
    static constexpr const char* device_extensions[] = {
        VK_KHR_SWAPCHAIN_EXTENSION_NAME
    };

    static constexpr const char* validation_layers[] = {
        "VK_LAYER_KHRONOS_validation"
    };
};

class Context {
public:
    Context() = default;
    ~Context();

    Context(const Context&) = delete;
    Context& operator=(const Context&) = delete;
    Context(Context&&) noexcept = default;
    Context& operator=(Context&&) noexcept = default;

    // Initialize Vulkan context with SDL window
    bool init(SDL_Window* window, const std::string& app_name = "OpenApex");

    // Recreate swapchain (e.g., on window resize)
    bool recreate_swapchain(SDL_Window* window);

    // Begin frame - acquire next image
    [[nodiscard]] bool begin_frame();

    // End frame - present
    [[nodiscard]] bool end_frame();

    // Wait for device idle
    void wait_idle() const;

    // Getters
    [[nodiscard]] const VulkanContext& get() const noexcept { return ctx_; }
    [[nodiscard]] VulkanContext& get() noexcept { return ctx_; }
    [[nodiscard]] const vk::Extent2D& extent() const noexcept { return ctx_.swapchain.extent; }
    [[nodiscard]] uint32_t current_frame() const noexcept { return ctx_.current_frame; }
    [[nodiscard]] const vk::CommandBuffer& command_buffer() const noexcept {
        return ctx_.command_buffers[ctx_.current_frame];
    }
    [[nodiscard]] const vk::RenderPass& render_pass() const noexcept { return ctx_.render_pass; }

private:
    VulkanContext ctx_;

    // Debug messenger and swapchain handle (not in VulkanContext to avoid copy issues)
    vk::DebugUtilsMessengerEXT debug_messenger_{VK_NULL_HANDLE};
    vk::SwapchainKHR swapchain_{VK_NULL_HANDLE};

    // Initialization helpers
    bool create_instance(const std::string& app_name);
    bool setup_debug_messenger();
    bool create_surface(SDL_Window* window);
    bool pick_physical_device();
    bool create_logical_device();
    bool create_swapchain(SDL_Window* window);
    bool create_image_views();
    bool create_render_pass();
    bool create_command_pool();
    bool create_command_buffers();
    bool create_sync_objects();

    // Cleanup helpers
    void cleanup_swapchain();
    void cleanup();

    // Validation
    static VKAPI_ATTR VkBool32 VKAPI_CALL debug_callback(
        VkDebugUtilsMessageSeverityFlagBitsEXT severity,
        VkDebugUtilsMessageTypeFlagsEXT type,
        const VkDebugUtilsMessengerCallbackDataEXT* callback_data,
        void* user_data);

    [[nodiscard]] bool check_device_extension_support(vk::PhysicalDevice device) const;
    [[nodiscard]] QueueFamilyIndices find_queue_families(vk::PhysicalDevice device) const;
    [[nodiscard]] SwapchainInfo query_swapchain_support(vk::PhysicalDevice device, SDL_Window* window) const;
    [[nodiscard]] vk::SurfaceFormatKHR choose_swap_surface_format(const std::vector<vk::SurfaceFormatKHR>& formats) const;
    [[nodiscard]] vk::PresentModeKHR choose_swap_present_mode(const std::vector<vk::PresentModeKHR>& modes) const;
    [[nodiscard]] vk::Extent2D choose_swap_extent(const vk::SurfaceCapabilitiesKHR& capabilities, SDL_Window* window) const;
};

} // namespace openapex::renderer::vulkan