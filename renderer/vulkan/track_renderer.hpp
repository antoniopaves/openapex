// SPDX-FileCopyrightText: 2026 OpenApex contributors
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include "context.hpp"
#include "pipeline.hpp"
#include "camera.hpp"

#include <openapex/sim/track.hpp>

#include <vulkan/vulkan.hpp>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include <vector>
#include <memory>

namespace openapex::renderer::vulkan {

struct TrackVertex {
    glm::vec3 position;
    glm::vec3 normal;
    glm::vec2 uv;
    uint32_t surface_type; // 0=asphalt, 1=kerb, 2=grass, etc.
};

class TrackRenderer {
public:
    TrackRenderer() = default;
    ~TrackRenderer();

    TrackRenderer(const TrackRenderer&) = delete;
    TrackRenderer& operator=(const TrackRenderer&) = delete;
    TrackRenderer(TrackRenderer&&) noexcept = default;
    TrackRenderer& operator=(TrackRenderer&&) noexcept = default;

    // Initialize track renderer with Vulkan context
    bool init(Context& context, const Pipeline& track_pipeline);

    // Build track mesh from track data
    bool build_track_mesh(const openapex::sim::Track& track);

    // Render track
    void render(vk::CommandBuffer cmd, const Pipeline& pipeline, const Camera& camera);

    // Cleanup
    void cleanup();

private:
    struct MeshBuffer {
        vk::Buffer vertex_buffer;
        vk::DeviceMemory vertex_memory;
        vk::Buffer index_buffer;
        vk::DeviceMemory index_memory;
        uint32_t index_count = 0;
    };

    Context* context_ = nullptr;
    MeshBuffer mesh_;
    std::vector<TrackVertex> vertices_;
    std::vector<uint32_t> indices_;

    // Helper to create buffers
    bool create_buffer(vk::DeviceSize size, vk::BufferUsageFlags usage,
                       vk::MemoryPropertyFlags properties,
                       vk::Buffer& buffer, vk::DeviceMemory& memory);
    void copy_buffer(vk::Buffer src, vk::Buffer dst, vk::DeviceSize size);
    uint32_t find_memory_type(uint32_t type_filter, vk::MemoryPropertyFlags properties);
};

} // namespace openapex::renderer::vulkan