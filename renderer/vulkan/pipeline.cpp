// SPDX-FileCopyrightText: 2026 OpenApex contributors
// SPDX-License-Identifier: GPL-3.0-or-later
#include "pipeline.hpp"

#include <vulkan/vulkan.hpp>
#include <glm/glm.hpp>
#include <fstream>
#include <iostream>

namespace openapex::renderer::vulkan {

bool Pipeline::create(Context& context, const vk::RenderPass& render_pass, const Config& config) {
    vk::Device device = context.get().device;

    // Read shader files
    auto vert_code = read_shader_file(config.vert_shader_path);
    auto frag_code = read_shader_file(config.frag_shader_path);

    if (vert_code.empty() || frag_code.empty()) {
        std::cerr << "Failed to read shader files\n";
        return false;
    }

    // Create shader modules
    vk::ShaderModule vert_module = create_shader_module(vert_code, device);
    vk::ShaderModule frag_module = create_shader_module(frag_code, device);

    // Shader stages
    vk::PipelineShaderStageCreateInfo vert_stage_info{};
    vert_stage_info.stage = vk::ShaderStageFlagBits::eVertex;
    vert_stage_info.module = vert_module;
    vert_stage_info.pName = "main";

    vk::PipelineShaderStageCreateInfo frag_stage_info{};
    frag_stage_info.stage = vk::ShaderStageFlagBits::eFragment;
    frag_stage_info.module = frag_module;
    frag_stage_info.pName = "main";

    vk::PipelineShaderStageCreateInfo shader_stages[] = {vert_stage_info, frag_stage_info};

    // Vertex input
    vk::PipelineVertexInputStateCreateInfo vertex_input_info{};
    vertex_input_info.vertexBindingDescriptionCount = static_cast<uint32_t>(config.vertex_bindings.size());
    vertex_input_info.pVertexBindingDescriptions = config.vertex_bindings.data();
    vertex_input_info.vertexAttributeDescriptionCount = static_cast<uint32_t>(config.vertex_attributes.size());
    vertex_input_info.pVertexAttributeDescriptions = config.vertex_attributes.data();

    // Input assembly
    vk::PipelineInputAssemblyStateCreateInfo input_assembly{};
    input_assembly.topology = config.topology;
    input_assembly.primitiveRestartEnable = VK_FALSE;

    // Viewport and scissor (dynamic)
    vk::PipelineViewportStateCreateInfo viewport_state{};
    viewport_state.viewportCount = 1;
    viewport_state.scissorCount = 1;

    // Rasterizer
    vk::PipelineRasterizationStateCreateInfo rasterizer{};
    rasterizer.depthClampEnable = VK_FALSE;
    rasterizer.rasterizerDiscardEnable = VK_FALSE;
    rasterizer.polygonMode = config.polygon_mode;
    rasterizer.lineWidth = 1.0f;
    rasterizer.cullMode = config.cull_mode;
    rasterizer.frontFace = config.front_face;
    rasterizer.depthBiasEnable = VK_FALSE;

    // Multisampling
    vk::PipelineMultisampleStateCreateInfo multisampling{};
    multisampling.sampleShadingEnable = VK_FALSE;
    multisampling.rasterizationSamples = vk::SampleCountFlagBits::e1;

    // Depth and stencil
    vk::PipelineDepthStencilStateCreateInfo depth_stencil{};
    depth_stencil.depthTestEnable = config.depth_test;
    depth_stencil.depthWriteEnable = config.depth_write;
    depth_stencil.depthCompareOp = config.depth_compare_op;
    depth_stencil.depthBoundsTestEnable = VK_FALSE;
    depth_stencil.stencilTestEnable = VK_FALSE;

    // Color blending
    vk::PipelineColorBlendAttachmentState color_blend_attachment{};
    color_blend_attachment.colorWriteMask =
        vk::ColorComponentFlagBits::eR | vk::ColorComponentFlagBits::eG |
        vk::ColorComponentFlagBits::eB | vk::ColorComponentFlagBits::eA;
    color_blend_attachment.blendEnable = VK_FALSE;

    vk::PipelineColorBlendStateCreateInfo color_blending{};
    color_blending.logicOpEnable = VK_FALSE;
    color_blending.logicOp = vk::LogicOp::eCopy;
    color_blending.attachmentCount = 1;
    color_blending.pAttachments = &color_blend_attachment;
    color_blending.blendConstants[0] = 0.0f;
    color_blending.blendConstants[1] = 0.0f;
    color_blending.blendConstants[2] = 0.0f;
    color_blending.blendConstants[3] = 0.0f;

    // Dynamic states
    vk::PipelineDynamicStateCreateInfo dynamic_state{};
    dynamic_state.dynamicStateCount = static_cast<uint32_t>(config.dynamic_states.size());
    dynamic_state.pDynamicStates = config.dynamic_states.data();

    // Descriptor set layout
    vk::DescriptorSetLayoutBinding ubo_layout_binding{};
    ubo_layout_binding.binding = 0;
    ubo_layout_binding.descriptorType = vk::DescriptorType::eUniformBuffer;
    ubo_layout_binding.descriptorCount = 1;
    ubo_layout_binding.stageFlags = vk::ShaderStageFlagBits::eVertex;
    ubo_layout_binding.pImmutableSamplers = nullptr;

    vk::DescriptorSetLayoutCreateInfo layout_info{};
    layout_info.bindingCount = 1;
    layout_info.pBindings = &ubo_layout_binding;

    try {
        descriptor_set_layout = device.createDescriptorSetLayout(layout_info);
    } catch (const vk::SystemError& err) {
        std::cerr << "Failed to create descriptor set layout: " << err.what() << "\n";
        return false;
    }

    // Pipeline layout
    vk::PipelineLayoutCreateInfo pipeline_layout_info{};
    pipeline_layout_info.setLayoutCount = 1;
    pipeline_layout_info.pSetLayouts = &descriptor_set_layout;
    pipeline_layout_info.pushConstantRangeCount = 1;
    vk::PushConstantRange push_constant{};
    push_constant.stageFlags = vk::ShaderStageFlagBits::eVertex;
    push_constant.offset = 0;
    push_constant.size = sizeof(glm::mat4); // MVP matrix
    pipeline_layout_info.pPushConstantRanges = &push_constant;

    try {
        layout = device.createPipelineLayout(pipeline_layout_info);
    } catch (const vk::SystemError& err) {
        std::cerr << "Failed to create pipeline layout: " << err.what() << "\n";
        return false;
    }

    // Graphics pipeline
    vk::GraphicsPipelineCreateInfo pipeline_info{};
    pipeline_info.stageCount = 2;
    pipeline_info.pStages = shader_stages;
    pipeline_info.pVertexInputState = &vertex_input_info;
    pipeline_info.pInputAssemblyState = &input_assembly;
    pipeline_info.pViewportState = &viewport_state;
    pipeline_info.pRasterizationState = &rasterizer;
    pipeline_info.pMultisampleState = &multisampling;
    pipeline_info.pDepthStencilState = &depth_stencil;
    pipeline_info.pColorBlendState = &color_blending;
    pipeline_info.pDynamicState = &dynamic_state;
    pipeline_info.layout = layout;
    pipeline_info.renderPass = render_pass;
    pipeline_info.subpass = 0;
    pipeline_info.basePipelineHandle = VK_NULL_HANDLE;

    try {
        pipeline = device.createGraphicsPipeline(VK_NULL_HANDLE, pipeline_info).value;
    } catch (const vk::SystemError& err) {
        std::cerr << "Failed to create graphics pipeline: " << err.what() << "\n";
        return false;
    }

    // Cleanup shader modules
    device.destroyShaderModule(vert_module);
    device.destroyShaderModule(frag_module);

    std::cout << "Pipeline created: " << config.vert_shader_path << " + " << config.frag_shader_path << "\n";
    return true;
}

void Pipeline::cleanup(Context& context) {
    vk::Device device = context.get().device;
    device.destroyPipeline(pipeline);
    device.destroyPipelineLayout(layout);
    device.destroyDescriptorSetLayout(descriptor_set_layout);
    device.destroyDescriptorPool(descriptor_pool);
    pipeline = VK_NULL_HANDLE;
    layout = VK_NULL_HANDLE;
    descriptor_set_layout = VK_NULL_HANDLE;
    descriptor_pool = VK_NULL_HANDLE;
    descriptor_sets.clear();
}

std::vector<char> Pipeline::read_shader_file(const std::string& path) {
    std::ifstream file(path, std::ios::ate | std::ios::binary);
    if (!file.is_open()) {
        std::cerr << "Failed to open shader file: " << path << "\n";
        return {};
    }

    size_t file_size = static_cast<size_t>(file.tellg());
    std::vector<char> buffer(file_size);
    file.seekg(0);
    file.read(buffer.data(), file_size);
    file.close();

    return buffer;
}

vk::ShaderModule Pipeline::create_shader_module(const std::vector<char>& code, vk::Device device) const {
    vk::ShaderModuleCreateInfo create_info{};
    create_info.codeSize = code.size();
    create_info.pCode = reinterpret_cast<const uint32_t*>(code.data());

    try {
        return device.createShaderModule(create_info);
    } catch (const vk::SystemError& err) {
        std::cerr << "Failed to create shader module: " << err.what() << "\n";
        return VK_NULL_HANDLE;
    }
}

} // namespace openapex::renderer::vulkan