// SPDX-FileCopyrightText: 2026 OpenApex contributors
// SPDX-License-Identifier: GPL-3.0-or-later
#include "track_renderer.hpp"

#include <openapex/sim/track.hpp>

#include <vulkan/vulkan.hpp>
#include <glm/glm.hpp>

#include <algorithm>
#include <iostream>

namespace openapex::renderer::vulkan {

TrackRenderer::~TrackRenderer() {
    cleanup();
}

bool TrackRenderer::init(Context& context, [[maybe_unused]] const Pipeline& track_pipeline) {
    context_ = &context;
    return true;
}

bool TrackRenderer::build_track_mesh(const openapex::sim::Track& track) {
    vertices_.clear();
    indices_.clear();

    // Get track points (TrackPoint has banking, width, etc.)
    const auto& points = track.points();
    if (points.size() < 2) {
        std::cerr << "Track has insufficient points for mesh generation\n";
        return false;
    }

    // Generate track mesh as triangle strips along the centerline
    // For each segment, create a quad (two triangles) representing the track surface
    const float track_width = 12.0f; // Default width, could come from track data
    const float kerb_width = 1.5f;

    for (size_t i = 0; i < points.size(); ++i) {
        const auto& curr = points[i];
        const auto& next = points[(i + 1) % points.size()];

        // Calculate segment direction
        glm::vec2 dir(next.pos.x - curr.pos.x, next.pos.y - curr.pos.y);
        float len = glm::length(dir);
        if (len < 0.001f) continue;
        dir /= len;

        // Perpendicular vector (left side)
        glm::vec2 perp(-dir.y, dir.x);

        // Current segment vertices
        float half_width = track_width * 0.5f;
        float y_pos = static_cast<float>(curr.banking); // Banking as height offset

        // Main track surface (asphalt)
        TrackVertex v1;
        v1.position = glm::vec3(curr.pos.x - perp.x * half_width, y_pos, curr.pos.y - perp.y * half_width);
        v1.normal = glm::vec3(0.0f, 1.0f, 0.0f);
        v1.uv = glm::vec2(0.0f, static_cast<float>(i));
        v1.surface_type = 0; // asphalt
        vertices_.push_back(v1);

        TrackVertex v2;
        v2.position = glm::vec3(curr.pos.x + perp.x * half_width, y_pos, curr.pos.y + perp.y * half_width);
        v2.normal = glm::vec3(0.0f, 1.0f, 0.0f);
        v2.uv = glm::vec2(1.0f, static_cast<float>(i));
        v2.surface_type = 0; // asphalt
        vertices_.push_back(v2);

        // Left kerb
        TrackVertex v3;
        v3.position = glm::vec3(curr.pos.x - perp.x * (half_width + kerb_width), y_pos - 0.05f, curr.pos.y - perp.y * (half_width + kerb_width));
        v3.normal = glm::vec3(0.0f, 1.0f, 0.0f);
        v3.uv = glm::vec2(-0.1f, static_cast<float>(i));
        v3.surface_type = 1; // kerb
        vertices_.push_back(v3);

        // Right kerb
        TrackVertex v4;
        v4.position = glm::vec3(curr.pos.x + perp.x * (half_width + kerb_width), y_pos - 0.05f, curr.pos.y + perp.y * (half_width + kerb_width));
        v4.normal = glm::vec3(0.0f, 1.0f, 0.0f);
        v4.uv = glm::vec2(1.1f, static_cast<float>(i));
        v4.surface_type = 1; // kerb
        vertices_.push_back(v4);
    }

    // Generate indices for triangle strips
    for (size_t i = 0; i + 3 < vertices_.size(); i += 4) {
        // Two quads per segment (track + kerbs), each quad = 2 triangles
        // Track surface quad (v1, v2, next_v1, next_v2)
        indices_.push_back(static_cast<uint32_t>(i));       // v1
        indices_.push_back(static_cast<uint32_t>(i + 1));   // v2
        indices_.push_back(static_cast<uint32_t>(i + 4));   // next v1
        indices_.push_back(static_cast<uint32_t>(i + 1));   // v2
        indices_.push_back(static_cast<uint32_t>(i + 5));   // next v2
        indices_.push_back(static_cast<uint32_t>(i + 4));   // next v1

        // Left kerb quad
        indices_.push_back(static_cast<uint32_t>(i + 2));   // left kerb
        indices_.push_back(static_cast<uint32_t>(i));       // track edge
        indices_.push_back(static_cast<uint32_t>(i + 6));   // next left kerb
        indices_.push_back(static_cast<uint32_t>(i));       // track edge
        indices_.push_back(static_cast<uint32_t>(i + 4));   // next track edge
        indices_.push_back(static_cast<uint32_t>(i + 6));   // next left kerb

        // Right kerb quad
        indices_.push_back(static_cast<uint32_t>(i + 1));   // track edge
        indices_.push_back(static_cast<uint32_t>(i + 3));   // right kerb
        indices_.push_back(static_cast<uint32_t>(i + 5));   // next track edge
        indices_.push_back(static_cast<uint32_t>(i + 3));   // right kerb
        indices_.push_back(static_cast<uint32_t>(i + 7));   // next right kerb
        indices_.push_back(static_cast<uint32_t>(i + 5));   // next track edge
    }

    // Create vertex and index buffers
    vk::Device device = context_->get().device;

    // Vertex buffer
    vk::DeviceSize vertex_buffer_size = vertices_.size() * sizeof(TrackVertex);
    if (!create_buffer(vertex_buffer_size,
                       vk::BufferUsageFlagBits::eVertexBuffer,
                       vk::MemoryPropertyFlagBits::eHostVisible | vk::MemoryPropertyFlagBits::eHostCoherent,
                       mesh_.vertex_buffer, mesh_.vertex_memory)) {
        return false;
    }

    void* data = device.mapMemory(mesh_.vertex_memory, 0, vertex_buffer_size);
    std::memcpy(data, vertices_.data(), vertex_buffer_size);
    device.unmapMemory(mesh_.vertex_memory);

    // Index buffer
    mesh_.index_count = static_cast<uint32_t>(indices_.size());
    vk::DeviceSize index_buffer_size = indices_.size() * sizeof(uint32_t);
    if (!create_buffer(index_buffer_size,
                       vk::BufferUsageFlagBits::eIndexBuffer,
                       vk::MemoryPropertyFlagBits::eHostVisible | vk::MemoryPropertyFlagBits::eHostCoherent,
                       mesh_.index_buffer, mesh_.index_memory)) {
        return false;
    }

    data = device.mapMemory(mesh_.index_memory, 0, index_buffer_size);
    std::memcpy(data, indices_.data(), index_buffer_size);
    device.unmapMemory(mesh_.index_memory);

    std::cout << "Track mesh built: " << vertices_.size() << " vertices, " << indices_.size() << " indices\n";
    return true;
}

void TrackRenderer::render(vk::CommandBuffer cmd, const Pipeline& pipeline, const Camera& camera) {
    if (mesh_.index_count == 0) return;

    // Bind pipeline
    cmd.bindPipeline(vk::PipelineBindPoint::eGraphics, pipeline.pipeline);

    // Bind vertex buffer
    vk::DeviceSize offsets[] = {0};
    cmd.bindVertexBuffers(0, 1, &mesh_.vertex_buffer, offsets);

    // Bind index buffer
    cmd.bindIndexBuffer(mesh_.index_buffer, 0, vk::IndexType::eUint32);

    // Push constants for MVP matrix
    glm::mat4 mvp = camera.view_projection_matrix();
    cmd.pushConstants(pipeline.layout, vk::ShaderStageFlagBits::eVertex, 0, sizeof(glm::mat4), &mvp);

    // Draw indexed
    cmd.drawIndexed(mesh_.index_count, 1, 0, 0, 0);
}

void TrackRenderer::cleanup() {
    if (context_) {
        vk::Device device = context_->get().device;
        device.destroyBuffer(mesh_.vertex_buffer);
        device.freeMemory(mesh_.vertex_memory);
        device.destroyBuffer(mesh_.index_buffer);
        device.freeMemory(mesh_.index_memory);
        mesh_ = {};
    }
    vertices_.clear();
    indices_.clear();
}

bool TrackRenderer::create_buffer(vk::DeviceSize size, vk::BufferUsageFlags usage,
                                  vk::MemoryPropertyFlags properties,
                                  vk::Buffer& buffer, vk::DeviceMemory& memory) {
    vk::Device device = context_->get().device;

    vk::BufferCreateInfo buffer_info{};
    buffer_info.size = size;
    buffer_info.usage = usage;
    buffer_info.sharingMode = vk::SharingMode::eExclusive;

    try {
        buffer = device.createBuffer(buffer_info);
    } catch (const vk::SystemError& err) {
        std::cerr << "Failed to create buffer: " << err.what() << "\n";
        return false;
    }

    vk::MemoryRequirements mem_reqs = device.getBufferMemoryRequirements(buffer);
    vk::MemoryAllocateInfo alloc_info{};
    alloc_info.allocationSize = mem_reqs.size;
    alloc_info.memoryTypeIndex = find_memory_type(mem_reqs.memoryTypeBits, properties);

    try {
        memory = device.allocateMemory(alloc_info);
        device.bindBufferMemory(buffer, memory, 0);
    } catch (const vk::SystemError& err) {
        std::cerr << "Failed to allocate buffer memory: " << err.what() << "\n";
        return false;
    }

    return true;
}

void TrackRenderer::copy_buffer(vk::Buffer src, vk::Buffer dst, vk::DeviceSize size) {
    vk::CommandBuffer cmd = context_->command_buffer();

    vk::CommandBufferBeginInfo begin_info{};
    begin_info.flags = vk::CommandBufferUsageFlagBits::eOneTimeSubmit;
    cmd.begin(begin_info);

    vk::BufferCopy copy_region{};
    copy_region.size = size;
    cmd.copyBuffer(src, dst, 1, &copy_region);

    cmd.end();

    vk::SubmitInfo submit_info{};
    submit_info.commandBufferCount = 1;
    submit_info.pCommandBuffers = &cmd;
    context_->get().graphics_queue.submit(1, &submit_info, nullptr);
    context_->get().graphics_queue.waitIdle();
}

uint32_t TrackRenderer::find_memory_type(uint32_t type_filter, vk::MemoryPropertyFlags properties) {
    vk::PhysicalDeviceMemoryProperties mem_props = context_->get().physical_device.getMemoryProperties();

    for (uint32_t i = 0; i < mem_props.memoryTypeCount; ++i) {
        if ((type_filter & (1 << i)) &&
            (mem_props.memoryTypes[i].propertyFlags & properties) == properties) {
            return i;
        }
    }

    throw std::runtime_error("Failed to find suitable memory type!");
}

} // namespace openapex::renderer::vulkan