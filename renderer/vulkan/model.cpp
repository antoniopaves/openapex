// SPDX-FileCopyrightText: 2026 OpenApex contributors
// SPDX-License-Identifier: GPL-3.0-or-later
#include "model.hpp"
#include "pipeline.hpp"

#include <vulkan/vulkan.hpp>
#include <tiny_gltf.h>

#include <iostream>
#include <algorithm>

namespace openapex::renderer::vulkan {

Model::~Model() {
    cleanup(*context_);
}

bool Model::load(Context& context, const std::string& path) {
    context_ = &context;
    tinygltf::Model gltf_model;
    tinygltf::TinyGLTF loader;
    std::string err, warn;

    bool ret = loader.LoadBinaryFromFile(&gltf_model, &err, &warn, path);
    if (!ret) {
        ret = loader.LoadASCIIFromFile(&gltf_model, &err, &warn, path);
    }

    if (!warn.empty()) {
        std::cout << "glTF warning: " << warn << "\n";
    }
    if (!err.empty()) {
        std::cerr << "glTF error: " << err << "\n";
    }
    if (!ret) {
        std::cerr << "Failed to load glTF model: " << path << "\n";
        return false;
    }

    // Process meshes
    for (const auto& gltf_mesh : gltf_model.meshes) {
        for (const auto& gltf_primitive : gltf_mesh.primitives) {
            ModelMesh mesh;
            mesh.name = gltf_mesh.name;

            // Get accessors
            const auto& pos_accessor = gltf_model.accessors[gltf_primitive.attributes.at("POSITION")];
            const auto& pos_view = gltf_model.bufferViews[pos_accessor.bufferView];
            const auto& pos_buffer = gltf_model.buffers[pos_view.buffer];

            const float* positions = reinterpret_cast<const float*>(
                pos_buffer.data.data() + pos_view.byteOffset + pos_accessor.byteOffset);

            const auto normal_accessor_it = gltf_primitive.attributes.find("NORMAL");
            const float* normals = nullptr;
            if (normal_accessor_it != gltf_primitive.attributes.end()) {
                const auto& norm_accessor = gltf_model.accessors[normal_accessor_it->second];
                const auto& norm_view = gltf_model.bufferViews[norm_accessor.bufferView];
                const auto& norm_buffer = gltf_model.buffers[norm_view.buffer];
                normals = reinterpret_cast<const float*>(
                    norm_buffer.data.data() + norm_view.byteOffset + norm_accessor.byteOffset);
            }

            const auto uv_accessor_it = gltf_primitive.attributes.find("TEXCOORD_0");
            const float* uvs = nullptr;
            if (uv_accessor_it != gltf_primitive.attributes.end()) {
                const auto& uv_accessor = gltf_model.accessors[uv_accessor_it->second];
                const auto& uv_view = gltf_model.bufferViews[uv_accessor.bufferView];
                const auto& uv_buffer = gltf_model.buffers[uv_view.buffer];
                uvs = reinterpret_cast<const float*>(
                    uv_buffer.data.data() + uv_view.byteOffset + uv_accessor.byteOffset);
            }

            // Vertex count
            size_t vertex_count = pos_accessor.count;

            // Build vertices
            mesh.vertices.resize(vertex_count);
            for (size_t i = 0; i < vertex_count; ++i) {
                mesh.vertices[i].position = glm::vec3(
                    positions[i * 3 + 0],
                    positions[i * 3 + 1],
                    positions[i * 3 + 2]);

                if (normals) {
                    mesh.vertices[i].normal = glm::vec3(
                        normals[i * 3 + 0],
                        normals[i * 3 + 1],
                        normals[i * 3 + 2]);
                } else {
                    mesh.vertices[i].normal = glm::vec3(0.0f, 1.0f, 0.0f);
                }

                if (uvs) {
                    mesh.vertices[i].uv = glm::vec2(uvs[i * 2 + 0], uvs[i * 2 + 1]);
                } else {
                    mesh.vertices[i].uv = glm::vec2(0.0f, 0.0f);
                }
            }

            // Indices
            if (gltf_primitive.indices >= 0) {
                const auto& idx_accessor = gltf_model.accessors[gltf_primitive.indices];
                const auto& idx_view = gltf_model.bufferViews[idx_accessor.bufferView];
                const auto& idx_buffer = gltf_model.buffers[idx_view.buffer];

                mesh.indices.resize(idx_accessor.count);
                if (idx_accessor.componentType == TINYGLTF_COMPONENT_TYPE_UNSIGNED_SHORT) {
                    const uint16_t* indices = reinterpret_cast<const uint16_t*>(
                        idx_buffer.data.data() + idx_view.byteOffset + idx_accessor.byteOffset);
                    for (size_t i = 0; i < idx_accessor.count; ++i) {
                        mesh.indices[i] = indices[i];
                    }
                } else if (idx_accessor.componentType == TINYGLTF_COMPONENT_TYPE_UNSIGNED_INT) {
                    const uint32_t* indices = reinterpret_cast<const uint32_t*>(
                        idx_buffer.data.data() + idx_view.byteOffset + idx_accessor.byteOffset);
                    for (size_t i = 0; i < idx_accessor.count; ++i) {
                        mesh.indices[i] = indices[i];
                    }
                }
            } else {
                // No indices - generate sequential
                mesh.indices.resize(vertex_count);
                for (size_t i = 0; i < vertex_count; ++i) {
                    mesh.indices[i] = static_cast<uint32_t>(i);
                }
            }

            // Create buffers
            vk::Device device = context.get().device;

            // Vertex buffer
            vk::DeviceSize vertex_buffer_size = mesh.vertices.size() * sizeof(ModelVertex);
            if (!create_buffer(vertex_buffer_size,
                               vk::BufferUsageFlagBits::eVertexBuffer,
                               vk::MemoryPropertyFlagBits::eHostVisible | vk::MemoryPropertyFlagBits::eHostCoherent,
                               mesh.vertex_buffer, mesh.vertex_memory)) {
                return false;
            }

            void* data = device.mapMemory(mesh.vertex_memory, 0, vertex_buffer_size);
            std::memcpy(data, mesh.vertices.data(), vertex_buffer_size);
            device.unmapMemory(mesh.vertex_memory);

            // Index buffer
            vk::DeviceSize index_buffer_size = mesh.indices.size() * sizeof(uint32_t);
            if (!create_buffer(index_buffer_size,
                               vk::BufferUsageFlagBits::eIndexBuffer,
                               vk::MemoryPropertyFlagBits::eHostVisible | vk::MemoryPropertyFlagBits::eHostCoherent,
                               mesh.index_buffer, mesh.index_memory)) {
                return false;
            }

            data = device.mapMemory(mesh.index_memory, 0, index_buffer_size);
            std::memcpy(data, mesh.indices.data(), index_buffer_size);
            device.unmapMemory(mesh.index_memory);

            meshes_.push_back(std::move(mesh));
        }
    }

    std::cout << "Model loaded: " << path << " (" << meshes_.size() << " meshes)\n";
    return true;
}

void Model::render(vk::CommandBuffer cmd, const Pipeline& pipeline) const {
    cmd.bindPipeline(vk::PipelineBindPoint::eGraphics, pipeline.pipeline);

    for (const auto& mesh : meshes_) {
        vk::DeviceSize offsets[] = {0};
        cmd.bindVertexBuffers(0, 1, &mesh.vertex_buffer, offsets);
        cmd.bindIndexBuffer(mesh.index_buffer, 0, vk::IndexType::eUint32);
        cmd.drawIndexed(static_cast<uint32_t>(mesh.indices.size()), 1, 0, 0, 0);
    }
}

void Model::cleanup(Context& context) {
    vk::Device device = context.get().device;
    for (auto& mesh : meshes_) {
        device.destroyBuffer(mesh.vertex_buffer);
        device.freeMemory(mesh.vertex_memory);
        device.destroyBuffer(mesh.index_buffer);
        device.freeMemory(mesh.index_memory);
    }
    meshes_.clear();

    for (auto& texture : textures_) {
        device.destroyImageView(texture.view);
        device.destroySampler(texture.sampler);
        device.destroyImage(texture.image);
        device.freeMemory(texture.memory);
    }
    textures_.clear();
}

bool Model::create_buffer(vk::DeviceSize size, vk::BufferUsageFlags usage,
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

void Model::copy_buffer(vk::Buffer src, vk::Buffer dst, vk::DeviceSize size) {
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

uint32_t Model::find_memory_type(uint32_t type_filter, vk::MemoryPropertyFlags properties) {
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