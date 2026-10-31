// SPDX-FileCopyrightText: 2026 OpenApex contributors
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include "context.hpp"
#include "pipeline.hpp"

#include <vulkan/vulkan.hpp>

#include <glm/glm.hpp>

#include <string>
#include <vector>
#include <memory>

namespace openapex::renderer::vulkan {

struct ModelVertex {
    glm::vec3 position;
    glm::vec3 normal;
    glm::vec2 uv;
};

struct ModelMesh {
    std::vector<ModelVertex> vertices;
    std::vector<uint32_t> indices;
    std::string name;

    // Buffers
    vk::Buffer vertex_buffer;
    vk::DeviceMemory vertex_memory;
    vk::Buffer index_buffer;
    vk::DeviceMemory index_memory;
};

struct ModelTexture {
    vk::Image image;
    vk::DeviceMemory memory;
    vk::ImageView view;
    vk::Sampler sampler;
    uint32_t width = 0;
    uint32_t height = 0;
};

class Model {
public:
    Model() = default;
    ~Model();

    Model(const Model&) = delete;
    Model& operator=(const Model&) = delete;
    Model(Model&&) noexcept = default;
    Model& operator=(Model&&) noexcept = default;

    // Load model from file (glTF)
    bool load(Context& context, const std::string& path);

    // Render all meshes
    void render(vk::CommandBuffer cmd, const Pipeline& pipeline) const;

    // Cleanup
    void cleanup(Context& context);

    [[nodiscard]] const std::vector<ModelMesh>& meshes() const noexcept { return meshes_; }

private:
    std::vector<ModelMesh> meshes_;
    std::vector<ModelTexture> textures_;
    Context* context_ = nullptr;

    // Helper to create buffer
    bool create_buffer(vk::DeviceSize size, vk::BufferUsageFlags usage,
                       vk::MemoryPropertyFlags properties,
                       vk::Buffer& buffer, vk::DeviceMemory& memory);
    void copy_buffer(vk::Buffer src, vk::Buffer dst, vk::DeviceSize size);
    uint32_t find_memory_type(uint32_t type_filter, vk::MemoryPropertyFlags properties);
};

} // namespace openapex::renderer::vulkan