// SPDX-FileCopyrightText: 2026 OpenApex contributors
// SPDX-License-Identifier: GPL-3.0-or-later
#include "context.hpp"

#include <vulkan/vulkan.hpp>
#include <SDL3/SDL_vulkan.h>

#include <algorithm>
#include <array>
#include <cstring>
#include <iostream>
#include <set>
#include <stdexcept>

namespace openapex::renderer::vulkan {

Context::~Context() {
    cleanup();
}

bool Context::init(SDL_Window* window, const std::string& app_name) {
    if (!create_instance(app_name)) return false;
    if (!setup_debug_messenger()) return false;
    if (!create_surface(window)) return false;
    if (!pick_physical_device()) return false;
    if (!create_logical_device()) return false;
    if (!create_swapchain(window)) return false;
    if (!create_image_views()) return false;
    if (!create_render_pass()) return false;
    if (!create_command_pool()) return false;
    if (!create_command_buffers()) return false;
    if (!create_sync_objects()) return false;

    std::cout << "Vulkan context initialized successfully\n";
    std::cout << "  Physical Device: " << ctx_.physical_device.getProperties().deviceName << "\n";
    std::cout << "  Swapchain Extent: " << ctx_.swapchain.extent.width << "x" << ctx_.swapchain.extent.height << "\n";
    std::cout << "  Swapchain Images: " << ctx_.swapchain.image_count << "\n";
    std::cout << "  Present Mode: " << static_cast<int>(ctx_.swapchain.present_mode) << "\n";

    return true;
}

bool Context::recreate_swapchain(SDL_Window* window) {
    ctx_.device.waitIdle();
    cleanup_swapchain();

    if (!create_swapchain(window)) return false;
    if (!create_image_views()) return false;
    // Note: render_pass, framebuffers, command buffers need recreation too
    // For now, we'll recreate them in the renderer
    return true;
}

bool Context::begin_frame() {
    ctx_.device.waitForFences(1, &ctx_.in_flight_fences[ctx_.current_frame], VK_TRUE, UINT64_MAX);

    auto [result, image_index] = ctx_.device.acquireNextImageKHR(
        swapchain_,
        UINT64_MAX,
        ctx_.image_available_semaphores[ctx_.current_frame],
        nullptr
    );

    if (result == vk::Result::eErrorOutOfDateKHR) {
        return false; // Need to recreate swapchain
    }
    if (result != vk::Result::eSuccess && result != vk::Result::eSuboptimalKHR) {
        throw std::runtime_error("Failed to acquire swapchain image!");
    }

    ctx_.current_frame = image_index;
    ctx_.device.resetFences(1, &ctx_.in_flight_fences[ctx_.current_frame]);

    auto& cmd = ctx_.command_buffers[ctx_.current_frame];
    cmd.reset();

    vk::CommandBufferBeginInfo begin_info{};
    begin_info.flags = vk::CommandBufferUsageFlagBits::eOneTimeSubmit;
    cmd.begin(begin_info);

    return true;
}

bool Context::end_frame() {
    auto& cmd = ctx_.command_buffers[ctx_.current_frame];
    cmd.end();

    vk::SubmitInfo submit_info{};
    vk::Semaphore wait_semaphores[] = { ctx_.image_available_semaphores[ctx_.current_frame] };
    vk::PipelineStageFlags wait_stages[] = { vk::PipelineStageFlagBits::eColorAttachmentOutput };
    submit_info.waitSemaphoreCount = 1;
    submit_info.pWaitSemaphores = wait_semaphores;
    submit_info.pWaitDstStageMask = wait_stages;
    submit_info.commandBufferCount = 1;
    submit_info.pCommandBuffers = &cmd;
    vk::Semaphore signal_semaphores[] = { ctx_.render_finished_semaphores[ctx_.current_frame] };
    submit_info.signalSemaphoreCount = 1;
    submit_info.pSignalSemaphores = signal_semaphores;

    ctx_.graphics_queue.submit(1, &submit_info, ctx_.in_flight_fences[ctx_.current_frame]);

    vk::PresentInfoKHR present_info{};
    present_info.waitSemaphoreCount = 1;
    present_info.pWaitSemaphores = signal_semaphores;
    present_info.swapchainCount = 1;
    present_info.pSwapchains = &swapchain_;
    present_info.pImageIndices = &ctx_.current_frame;
    present_info.pResults = nullptr;

    return true;
}

void Context::wait_idle() const {
    ctx_.device.waitIdle();
}

// Instance creation
bool Context::create_instance(const std::string& app_name) {
    vk::ApplicationInfo app_info{};
    app_info.pApplicationName = app_name.c_str();
    app_info.applicationVersion = VK_MAKE_VERSION(0, 1, 0);
    app_info.pEngineName = "OpenApex";
    app_info.engineVersion = VK_MAKE_VERSION(0, 1, 0);
    app_info.apiVersion = VK_API_VERSION_1_3;

    // Get required extensions from SDL
    uint32_t sdl_ext_count = 0;
    const char* const* sdl_extensions = SDL_Vulkan_GetInstanceExtensions(&sdl_ext_count);
    std::vector<const char*> extensions(sdl_extensions, sdl_extensions + sdl_ext_count);

    if (VulkanContext::enable_validation_layers) {
        extensions.push_back(VK_EXT_DEBUG_UTILS_EXTENSION_NAME);
    }

    // Check validation layer support
    if (VulkanContext::enable_validation_layers) {
        auto available_layers = vk::enumerateInstanceLayerProperties();
        for (const char* layer_name : VulkanContext::validation_layers) {
            bool found = false;
            for (const auto& layer : available_layers) {
                if (strcmp(layer.layerName, layer_name) == 0) {
                    found = true;
                    break;
                }
            }
            if (!found) {
                std::cerr << "Validation layer " << layer_name << " not available!\n";
                return false;
            }
        }
    }

    vk::InstanceCreateInfo create_info{};
    create_info.pApplicationInfo = &app_info;
    create_info.enabledExtensionCount = static_cast<uint32_t>(extensions.size());
    create_info.ppEnabledExtensionNames = extensions.data();

    VkDebugUtilsMessengerCreateInfoEXT debug_create_info{};
    if (VulkanContext::enable_validation_layers) {
        create_info.enabledLayerCount = static_cast<uint32_t>(std::size(VulkanContext::validation_layers));
        create_info.ppEnabledLayerNames = VulkanContext::validation_layers;

        debug_create_info.sType = VK_STRUCTURE_TYPE_DEBUG_UTILS_MESSENGER_CREATE_INFO_EXT;
        debug_create_info.messageSeverity =
            VK_DEBUG_UTILS_MESSAGE_SEVERITY_VERBOSE_BIT_EXT |
            VK_DEBUG_UTILS_MESSAGE_SEVERITY_WARNING_BIT_EXT |
            VK_DEBUG_UTILS_MESSAGE_SEVERITY_ERROR_BIT_EXT;
        debug_create_info.messageType =
            VK_DEBUG_UTILS_MESSAGE_TYPE_GENERAL_BIT_EXT |
            VK_DEBUG_UTILS_MESSAGE_TYPE_VALIDATION_BIT_EXT |
            VK_DEBUG_UTILS_MESSAGE_TYPE_PERFORMANCE_BIT_EXT;
        debug_create_info.pfnUserCallback = debug_callback;
        debug_create_info.pUserData = nullptr;
        create_info.pNext = &debug_create_info;
    } else {
        create_info.enabledLayerCount = 0;
    }

    try {
        ctx_.instance = vk::createInstance(create_info);
    } catch (const vk::SystemError& err) {
        std::cerr << "Failed to create Vulkan instance: " << err.what() << "\n";
        return false;
    }

    return true;
}

bool Context::setup_debug_messenger() {
    if (!VulkanContext::enable_validation_layers) return true;

    VkDebugUtilsMessengerCreateInfoEXT create_info{};
    create_info.sType = VK_STRUCTURE_TYPE_DEBUG_UTILS_MESSENGER_CREATE_INFO_EXT;
    create_info.messageSeverity =
        VK_DEBUG_UTILS_MESSAGE_SEVERITY_VERBOSE_BIT_EXT |
        VK_DEBUG_UTILS_MESSAGE_SEVERITY_WARNING_BIT_EXT |
        VK_DEBUG_UTILS_MESSAGE_SEVERITY_ERROR_BIT_EXT;
    create_info.messageType =
        VK_DEBUG_UTILS_MESSAGE_TYPE_GENERAL_BIT_EXT |
        VK_DEBUG_UTILS_MESSAGE_TYPE_VALIDATION_BIT_EXT |
        VK_DEBUG_UTILS_MESSAGE_TYPE_PERFORMANCE_BIT_EXT;
    create_info.pfnUserCallback = debug_callback;
    create_info.pUserData = nullptr;

    try {
        auto func = reinterpret_cast<PFN_vkCreateDebugUtilsMessengerEXT>(
            ctx_.instance.getProcAddr("vkCreateDebugUtilsMessengerEXT"));
        if (func) {
            VkDebugUtilsMessengerEXT raw_messenger = VK_NULL_HANDLE;
            VkResult result = func(ctx_.instance, &create_info, nullptr, &raw_messenger);
            if (result != VK_SUCCESS) {
                std::cerr << "Failed to create debug messenger: " << result << "\n";
                return false;
            }
            debug_messenger_ = raw_messenger;
        }
    } catch (const vk::SystemError& err) {
        std::cerr << "Failed to set up debug messenger: " << err.what() << "\n";
        return false;
    }

    return true;
}

bool Context::create_surface(SDL_Window* window) {
    VkSurfaceKHR surface;
    if (!SDL_Vulkan_CreateSurface(window, ctx_.instance, nullptr, &surface)) {
        std::cerr << "Failed to create window surface: " << SDL_GetError() << "\n";
        return false;
    }
    ctx_.surface = surface;
    return true;
}

bool Context::pick_physical_device() {
    auto devices = ctx_.instance.enumeratePhysicalDevices();
    if (devices.empty()) {
        std::cerr << "Failed to find GPUs with Vulkan support!\n";
        return false;
    }

    for (const auto& device : devices) {
        if (check_device_extension_support(device)) {
            QueueFamilyIndices indices = find_queue_families(device);
            if (indices.is_complete()) {
                ctx_.physical_device = device;
                ctx_.queue_indices = indices;
                return true;
            }
        }
    }

    std::cerr << "Failed to find a suitable GPU!\n";
    return false;
}

bool Context::create_logical_device() {
    QueueFamilyIndices indices = ctx_.queue_indices;

    std::vector<vk::DeviceQueueCreateInfo> queue_create_infos;
    std::set<uint32_t> unique_queue_families = {
        indices.graphics_family.value(),
        indices.present_family.value()
    };

    float queue_priority = 1.0f;
    for (uint32_t queue_family : unique_queue_families) {
        vk::DeviceQueueCreateInfo queue_create_info{};
        queue_create_info.queueFamilyIndex = queue_family;
        queue_create_info.queueCount = 1;
        queue_create_info.pQueuePriorities = &queue_priority;
        queue_create_infos.push_back(queue_create_info);
    }

    vk::PhysicalDeviceFeatures device_features{};
    device_features.samplerAnisotropy = VK_TRUE;

    vk::DeviceCreateInfo create_info{};
    create_info.queueCreateInfoCount = static_cast<uint32_t>(queue_create_infos.size());
    create_info.pQueueCreateInfos = queue_create_infos.data();
    create_info.pEnabledFeatures = &device_features;
    create_info.enabledExtensionCount = static_cast<uint32_t>(std::size(VulkanContext::device_extensions));
    create_info.ppEnabledExtensionNames = VulkanContext::device_extensions;

    if (VulkanContext::enable_validation_layers) {
        create_info.enabledLayerCount = static_cast<uint32_t>(std::size(VulkanContext::validation_layers));
        create_info.ppEnabledLayerNames = VulkanContext::validation_layers;
    } else {
        create_info.enabledLayerCount = 0;
    }

    try {
        ctx_.device = ctx_.physical_device.createDevice(create_info);
    } catch (const vk::SystemError& err) {
        std::cerr << "Failed to create logical device: " << err.what() << "\n";
        return false;
    }

    ctx_.graphics_queue = ctx_.device.getQueue(indices.graphics_family.value(), 0);
    ctx_.present_queue = ctx_.device.getQueue(indices.present_family.value(), 0);

    return true;
}

bool Context::create_swapchain(SDL_Window* window) {
    SwapchainInfo swapchain_info = query_swapchain_support(ctx_.physical_device, window);

    vk::SurfaceFormatKHR surface_format = choose_swap_surface_format(
        ctx_.physical_device.getSurfaceFormatsKHR(ctx_.surface));
    vk::PresentModeKHR present_mode = choose_swap_present_mode(
        ctx_.physical_device.getSurfacePresentModesKHR(ctx_.surface));
    vk::Extent2D extent = choose_swap_extent(
        ctx_.physical_device.getSurfaceCapabilitiesKHR(ctx_.surface), window);

    uint32_t image_count = swapchain_info.image_count;
    if (image_count < swapchain_info.image_count) {
        image_count = swapchain_info.image_count;
    }

    vk::SwapchainCreateInfoKHR create_info{};
    create_info.surface = ctx_.surface;
    create_info.minImageCount = image_count;
    create_info.imageFormat = surface_format.format;
    create_info.imageColorSpace = surface_format.colorSpace;
    create_info.imageExtent = extent;
    create_info.imageArrayLayers = 1;
    create_info.imageUsage = vk::ImageUsageFlagBits::eColorAttachment;

    QueueFamilyIndices indices = ctx_.queue_indices;
    uint32_t queue_family_indices[] = {
        indices.graphics_family.value(),
        indices.present_family.value()
    };

    if (indices.graphics_family != indices.present_family) {
        create_info.imageSharingMode = vk::SharingMode::eConcurrent;
        create_info.queueFamilyIndexCount = 2;
        create_info.pQueueFamilyIndices = queue_family_indices;
    } else {
        create_info.imageSharingMode = vk::SharingMode::eExclusive;
    }

    create_info.preTransform = ctx_.physical_device.getSurfaceCapabilitiesKHR(ctx_.surface).currentTransform;
    create_info.compositeAlpha = vk::CompositeAlphaFlagBitsKHR::eOpaque;
    create_info.presentMode = present_mode;
    create_info.clipped = VK_TRUE;
    create_info.oldSwapchain = swapchain_;

    try {
        auto swapchain = ctx_.device.createSwapchainKHR(create_info);
        // Store swapchain - we'll need to add this to VulkanContext
        // For now, we'll just create and get images
        auto images = ctx_.device.getSwapchainImagesKHR(swapchain);

        ctx_.swapchain.extent = extent;
        ctx_.swapchain.format = surface_format.format;
        ctx_.swapchain.color_space = surface_format.colorSpace;
        ctx_.swapchain.present_mode = present_mode;
        ctx_.swapchain.image_count = static_cast<uint32_t>(images.size());
        ctx_.swapchain.images = images;

        // Store swapchain handle in a new member - for now we'll just track it externally
        swapchain_ = swapchain;

    } catch (const vk::SystemError& err) {
        std::cerr << "Failed to create swapchain: " << err.what() << "\n";
        return false;
    }

    return true;
}

bool Context::create_image_views() {
    ctx_.swapchain.image_views.resize(ctx_.swapchain.images.size());
    for (size_t i = 0; i < ctx_.swapchain.images.size(); ++i) {
        vk::ImageViewCreateInfo create_info{};
        create_info.image = ctx_.swapchain.images[i];
        create_info.viewType = vk::ImageViewType::e2D;
        create_info.format = ctx_.swapchain.format;
        create_info.components.r = vk::ComponentSwizzle::eIdentity;
        create_info.components.g = vk::ComponentSwizzle::eIdentity;
        create_info.components.b = vk::ComponentSwizzle::eIdentity;
        create_info.components.a = vk::ComponentSwizzle::eIdentity;
        create_info.subresourceRange.aspectMask = vk::ImageAspectFlagBits::eColor;
        create_info.subresourceRange.baseMipLevel = 0;
        create_info.subresourceRange.levelCount = 1;
        create_info.subresourceRange.baseArrayLayer = 0;
        create_info.subresourceRange.layerCount = 1;

        try {
            ctx_.swapchain.image_views[i] = ctx_.device.createImageView(create_info);
        } catch (const vk::SystemError& err) {
            std::cerr << "Failed to create image views: " << err.what() << "\n";
            return false;
        }
    }
    return true;
}

bool Context::create_render_pass() {
    vk::AttachmentDescription color_attachment{};
    color_attachment.format = ctx_.swapchain.format;
    color_attachment.samples = vk::SampleCountFlagBits::e1;
    color_attachment.loadOp = vk::AttachmentLoadOp::eClear;
    color_attachment.storeOp = vk::AttachmentStoreOp::eStore;
    color_attachment.stencilLoadOp = vk::AttachmentLoadOp::eDontCare;
    color_attachment.stencilStoreOp = vk::AttachmentStoreOp::eDontCare;
    color_attachment.initialLayout = vk::ImageLayout::eUndefined;
    color_attachment.finalLayout = vk::ImageLayout::ePresentSrcKHR;

    vk::AttachmentReference color_attachment_ref{};
    color_attachment_ref.attachment = 0;
    color_attachment_ref.layout = vk::ImageLayout::eColorAttachmentOptimal;

    vk::SubpassDescription subpass{};
    subpass.pipelineBindPoint = vk::PipelineBindPoint::eGraphics;
    subpass.colorAttachmentCount = 1;
    subpass.pColorAttachments = &color_attachment_ref;

    vk::SubpassDependency dependency{};
    dependency.srcSubpass = VK_SUBPASS_EXTERNAL;
    dependency.dstSubpass = 0;
    dependency.srcStageMask = vk::PipelineStageFlagBits::eColorAttachmentOutput;
    dependency.srcAccessMask = vk::AccessFlagBits::eNone;
    dependency.dstStageMask = vk::PipelineStageFlagBits::eColorAttachmentOutput;
    dependency.dstAccessMask = vk::AccessFlagBits::eColorAttachmentWrite;

    vk::RenderPassCreateInfo render_pass_info{};
    render_pass_info.attachmentCount = 1;
    render_pass_info.pAttachments = &color_attachment;
    render_pass_info.subpassCount = 1;
    render_pass_info.pSubpasses = &subpass;
    render_pass_info.dependencyCount = 1;
    render_pass_info.pDependencies = &dependency;

    try {
        ctx_.render_pass = ctx_.device.createRenderPass(render_pass_info);
    } catch (const vk::SystemError& err) {
        std::cerr << "Failed to create render pass: " << err.what() << "\n";
        return false;
    }

    return true;
}

bool Context::create_command_pool() {
    vk::CommandPoolCreateInfo pool_info{};
    pool_info.flags = vk::CommandPoolCreateFlagBits::eResetCommandBuffer;
    pool_info.queueFamilyIndex = ctx_.queue_indices.graphics_family.value();

    try {
        ctx_.command_pool = ctx_.device.createCommandPool(pool_info);
    } catch (const vk::SystemError& err) {
        std::cerr << "Failed to create command pool: " << err.what() << "\n";
        return false;
    }

    return true;
}

bool Context::create_command_buffers() {
    ctx_.command_buffers.resize(VulkanContext::max_frames_in_flight);

    vk::CommandBufferAllocateInfo alloc_info{};
    alloc_info.commandPool = ctx_.command_pool;
    alloc_info.level = vk::CommandBufferLevel::ePrimary;
    alloc_info.commandBufferCount = VulkanContext::max_frames_in_flight;

    try {
        ctx_.command_buffers = ctx_.device.allocateCommandBuffers(alloc_info);
    } catch (const vk::SystemError& err) {
        std::cerr << "Failed to allocate command buffers: " << err.what() << "\n";
        return false;
    }

    return true;
}

bool Context::create_sync_objects() {
    ctx_.image_available_semaphores.resize(VulkanContext::max_frames_in_flight);
    ctx_.render_finished_semaphores.resize(VulkanContext::max_frames_in_flight);
    ctx_.in_flight_fences.resize(VulkanContext::max_frames_in_flight);

    vk::SemaphoreCreateInfo semaphore_info{};
    vk::FenceCreateInfo fence_info{};
    fence_info.flags = vk::FenceCreateFlagBits::eSignaled;

    for (size_t i = 0; i < VulkanContext::max_frames_in_flight; ++i) {
        try {
            ctx_.image_available_semaphores[i] = ctx_.device.createSemaphore(semaphore_info);
            ctx_.render_finished_semaphores[i] = ctx_.device.createSemaphore(semaphore_info);
            ctx_.in_flight_fences[i] = ctx_.device.createFence(fence_info);
        } catch (const vk::SystemError& err) {
            std::cerr << "Failed to create synchronization objects: " << err.what() << "\n";
            return false;
        }
    }

    return true;
}

void Context::cleanup_swapchain() {
    for (auto& image_view : ctx_.swapchain.image_views) {
        ctx_.device.destroyImageView(image_view);
    }
    ctx_.swapchain.image_views.clear();
    ctx_.swapchain.images.clear();

    if (swapchain_) {
        ctx_.device.destroySwapchainKHR(swapchain_);
        swapchain_ = VK_NULL_HANDLE;
    }
}

void Context::cleanup() {
    cleanup_swapchain();

    for (size_t i = 0; i < VulkanContext::max_frames_in_flight; ++i) {
        ctx_.device.destroySemaphore(ctx_.image_available_semaphores[i]);
        ctx_.device.destroySemaphore(ctx_.render_finished_semaphores[i]);
        ctx_.device.destroyFence(ctx_.in_flight_fences[i]);
    }

    ctx_.device.destroyCommandPool(ctx_.command_pool);
    ctx_.device.destroyRenderPass(ctx_.render_pass);
    ctx_.instance.destroySurfaceKHR(ctx_.surface);
    ctx_.device.destroy();

    if (VulkanContext::enable_validation_layers && debug_messenger_) {
        auto func = reinterpret_cast<PFN_vkDestroyDebugUtilsMessengerEXT>(
            ctx_.instance.getProcAddr("vkDestroyDebugUtilsMessengerEXT"));
        if (func) {
            func(ctx_.instance, debug_messenger_, nullptr);
        }
    }

    ctx_.instance.destroy();
}

VKAPI_ATTR VkBool32 VKAPI_CALL Context::debug_callback(
    VkDebugUtilsMessageSeverityFlagBitsEXT severity,
    VkDebugUtilsMessageTypeFlagsEXT type,
    const VkDebugUtilsMessengerCallbackDataEXT* callback_data,
    void* user_data) {

    std::cerr << "Validation Layer: " << callback_data->pMessage << "\n";
    return VK_FALSE;
}

bool Context::check_device_extension_support(vk::PhysicalDevice device) const {
    auto extensions = device.enumerateDeviceExtensionProperties();
    std::set<std::string> required_extensions(VulkanContext::device_extensions, VulkanContext::device_extensions + std::size(VulkanContext::device_extensions));

    for (const auto& extension : extensions) {
        required_extensions.erase(extension.extensionName);
    }

    return required_extensions.empty();
}

QueueFamilyIndices Context::find_queue_families(vk::PhysicalDevice device) const {
    QueueFamilyIndices indices;
    auto queue_families = device.getQueueFamilyProperties();

    for (uint32_t i = 0; i < queue_families.size(); ++i) {
        if (queue_families[i].queueFlags & vk::QueueFlagBits::eGraphics) {
            indices.graphics_family = i;
        }

        if (device.getSurfaceSupportKHR(i, ctx_.surface)) {
            indices.present_family = i;
        }

        if (indices.is_complete()) break;
    }

    return indices;
}

SwapchainInfo Context::query_swapchain_support(vk::PhysicalDevice device, SDL_Window* window) const {
    SwapchainInfo info;
    info.extent = choose_swap_extent(device.getSurfaceCapabilitiesKHR(ctx_.surface), window);
    info.format = choose_swap_surface_format(device.getSurfaceFormatsKHR(ctx_.surface)).format;
    info.color_space = choose_swap_surface_format(device.getSurfaceFormatsKHR(ctx_.surface)).colorSpace;
    info.present_mode = choose_swap_present_mode(device.getSurfacePresentModesKHR(ctx_.surface));
    info.image_count = device.getSurfaceCapabilitiesKHR(ctx_.surface).minImageCount + 1;
    return info;
}

vk::SurfaceFormatKHR Context::choose_swap_surface_format(const std::vector<vk::SurfaceFormatKHR>& formats) const {
    for (const auto& format : formats) {
        if (format.format == vk::Format::eB8G8R8A8Srgb &&
            format.colorSpace == vk::ColorSpaceKHR::eSrgbNonlinear) {
            return format;
        }
    }
    return formats[0];
}

vk::PresentModeKHR Context::choose_swap_present_mode(const std::vector<vk::PresentModeKHR>& modes) const {
    for (const auto& mode : modes) {
        if (mode == vk::PresentModeKHR::eMailbox) {
            return mode;
        }
    }
    return vk::PresentModeKHR::eFifo;
}

vk::Extent2D Context::choose_swap_extent(const vk::SurfaceCapabilitiesKHR& capabilities, SDL_Window* window) const {
    if (capabilities.currentExtent.width != UINT32_MAX) {
        return capabilities.currentExtent;
    }

    int width, height;
    SDL_GetWindowSizeInPixels(window, &width, &height);

    vk::Extent2D actual_extent = {
        static_cast<uint32_t>(width),
        static_cast<uint32_t>(height)
    };

    actual_extent.width = std::clamp(actual_extent.width,
        capabilities.minImageExtent.width, capabilities.maxImageExtent.width);
    actual_extent.height = std::clamp(actual_extent.height,
        capabilities.minImageExtent.height, capabilities.maxImageExtent.height);

    return actual_extent;
}

} // namespace openapex::renderer::vulkan