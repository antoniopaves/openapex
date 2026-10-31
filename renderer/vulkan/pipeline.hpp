// SPDX-FileCopyrightText: 2026 OpenApex contributors
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include "context.hpp"

#include <vulkan/vulkan.hpp>

#include <string>
#include <vector>

namespace openapex::renderer::vulkan {

struct Pipeline {
    vk::Pipeline pipeline;
    vk::PipelineLayout layout;
    vk::DescriptorSetLayout descriptor_set_layout;
    vk::DescriptorPool descriptor_pool;
    std::vector<vk::DescriptorSet> descriptor_sets;

    // Pipeline configuration
    struct Config {
        std::string vert_shader_path;
        std::string frag_shader_path;
        std::vector<vk::VertexInputBindingDescription> vertex_bindings;
        std::vector<vk::VertexInputAttributeDescription> vertex_attributes;
        vk::PrimitiveTopology topology = vk::PrimitiveTopology::eTriangleList;
        vk::PolygonMode polygon_mode = vk::PolygonMode::eFill;
        vk::CullModeFlags cull_mode = vk::CullModeFlagBits::eBack;
        vk::FrontFace front_face = vk::FrontFace::eCounterClockwise;
        bool depth_test = true;
        bool depth_write = true;
        vk::CompareOp depth_compare_op = vk::CompareOp::eLessOrEqual;
        std::vector<vk::DynamicState> dynamic_states = {
            vk::DynamicState::eViewport,
            vk::DynamicState::eScissor
        };
    };

    Pipeline() = default;
    ~Pipeline() = default;

    Pipeline(const Pipeline&) = delete;
    Pipeline& operator=(const Pipeline&) = delete;
    Pipeline(Pipeline&&) noexcept = default;
    Pipeline& operator=(Pipeline&&) noexcept = default;

    // Create pipeline from config
    bool create(Context& context, const vk::RenderPass& render_pass, const Config& config);

    // Cleanup
    void cleanup(Context& context);

private:
    // Helper to read shader file
    static std::vector<char> read_shader_file(const std::string& path);

    // Create shader module
    vk::ShaderModule create_shader_module(const std::vector<char>& code, vk::Device device) const;
};

} // namespace openapex::renderer::vulkan