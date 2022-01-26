#include "depth_only_pass.h"
#include "Utils.h"
#include "common/vk_initializers.h"
#include "scene_graph/components/image.h"
#include "scene_graph/components/material.h"
#include "scene_graph/components/pbr_material.h"
#include "scene_graph/components/texture.h"

using namespace vkb;
using namespace vkb::sg;

depth_only_pass::depth_only_pass(vkb::RenderContext &render_context, vkb::ShaderSource &&vertex_shader, vkb::ShaderSource &&fragment_shader) :
    Subpass{render_context, std::move(vertex_shader), std::move(fragment_shader)}
{
}

depth_only_pass::~depth_only_pass()
{
}

void depth_only_pass::prepare()
{
	auto &device = render_context.get_device();

	ColorBlendState           depthOnlyColorState;
	ColorBlendAttachmentState depthOnlyAttaState;
	depthOnlyAttaState.color_write_mask = 0;
	depthOnlyColorState.attachments.push_back(depthOnlyAttaState);
	pipeline_state.set_color_blend_state(depthOnlyColorState);

	std::vector<ShaderModule *> modules;
	modules.push_back(&device.get_resource_cache().request_shader_module(VK_SHADER_STAGE_VERTEX_BIT, get_vertex_shader()));
	modules.push_back(&device.get_resource_cache().request_shader_module(VK_SHADER_STAGE_FRAGMENT_BIT, get_fragment_shader()));

	auto &layout = device.get_resource_cache().request_pipeline_layout(modules);
	pipeline_state.set_pipeline_layout(layout);
}

void depth_only_pass::set_as_shadow_pipeline()
{
	isShadowPass = true;

	DepthStencilState shadowDepthState;
	shadowDepthState.depth_compare_op = VK_COMPARE_OP_LESS;
	pipeline_state.set_depth_stencil_state(shadowDepthState);

	RasterizationState rasterState;
	rasterState.depth_bias_enable = true;
	pipeline_state.set_rasterization_state(rasterState);
}

void depth_only_pass::draw(vkb::CommandBuffer & command_buffer, vkb::sg::Camera * camera, std::multimap<float, std::pair<vkb::sg::Node*, vkb::sg::SubMesh*>>* submeshs) {
    render_camera = camera;
    draw_meshs = submeshs;
    draw(command_buffer);
}

void depth_only_pass::draw(vkb::CommandBuffer &command_buffer)
{
	Device &device = render_context.get_device();

	if (isShadowPass) {
        command_buffer.set_depth_bias(2.0f, 0.0f, 2.0f);
	}

	// update and bind buffer
	bind_pipeline_state(command_buffer, pipeline_state);
	for (auto iter = draw_meshs->begin(); iter != draw_meshs->end(); iter++)
	{
		auto node    = iter->second.first;
		auto submesh = iter->second.second;
		update_global_uniform_buffers(command_buffer, node);
		auto &layout = const_cast<PipelineLayout &>(pipeline_state.get_pipeline_layout());
		command_buffer.bind_pipeline_layout(layout);
		if (bind_vertex_input(command_buffer, submesh))
		{
			command_buffer.draw_indexed(submesh->vertex_indices, 1, 0, 0, 0);
		}
		else
		{
			command_buffer.draw(submesh->vertices_count, 1, 0, 0);
		}
	}
}

void depth_only_pass::bind_pipeline_state(vkb::CommandBuffer &command_buffer, vkb::PipelineState &pipeline)
{
	command_buffer.set_color_blend_state(pipeline.get_color_blend_state());
	command_buffer.set_depth_stencil_state(pipeline.get_depth_stencil_state());
	command_buffer.set_input_assembly_state(pipeline.get_input_assembly_state());
	command_buffer.set_rasterization_state(pipeline.get_rasterization_state());
	command_buffer.set_viewport_state(pipeline.get_viewport_state());
	command_buffer.set_multisample_state(pipeline.get_multisample_state());
}

void depth_only_pass::update_global_uniform_buffers(vkb::CommandBuffer &commandBuffer, vkb::sg::Node *node)
{
	auto &render_frame = get_render_context().get_active_frame();
	auto  allocation   = render_frame.allocate_buffer(VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT, sizeof(GlobalUniform), 0);
	auto &transform    = node->get_transform();

	GlobalUniform globalUniform;
	globalUniform.model           = transform.get_world_matrix();
	globalUniform.view_project    = vkb::vulkan_style_projection(render_camera->get_projection()) * render_camera->get_view();
	globalUniform.camera_position = render_camera->get_node()->get_component<vkb::sg::Transform>().get_world_translation();
	allocation.update(globalUniform);
	commandBuffer.bind_buffer(allocation.get_buffer(), allocation.get_offset(), allocation.get_size(), 0, 0, 0);
}

bool depth_only_pass::bind_vertex_input(vkb::CommandBuffer &command_buffer, vkb::sg::SubMesh *submesh)
{
	auto vertex_input_resources = pipeline_state.get_pipeline_layout().get_resources(ShaderResourceType::Input, VK_SHADER_STAGE_VERTEX_BIT);

	VertexInputState vertex_input_state;

	for (auto &input_resource : vertex_input_resources)
	{
		sg::VertexAttribute attribute;

		if (!submesh->get_attribute(input_resource.name, attribute))
		{
			continue;
		}

		VkVertexInputAttributeDescription vertex_attribute{};
		vertex_attribute.binding  = input_resource.location;
		vertex_attribute.format   = attribute.format;
		vertex_attribute.location = input_resource.location;
		vertex_attribute.offset   = attribute.offset;

		vertex_input_state.attributes.push_back(vertex_attribute);

		VkVertexInputBindingDescription vertex_binding{};
		vertex_binding.binding = input_resource.location;
		vertex_binding.stride  = attribute.stride;

		vertex_input_state.bindings.push_back(vertex_binding);
	}

	command_buffer.set_vertex_input_state(vertex_input_state);

	// Find submesh vertex buffers matching the shader input attribute names
	for (auto &input_resource : vertex_input_resources)
	{
		const auto &buffer_iter = submesh->vertex_buffers.find(input_resource.name);

		if (buffer_iter != submesh->vertex_buffers.end())
		{
			std::vector<std::reference_wrapper<const core::Buffer>> buffers;
			buffers.emplace_back(std::ref(buffer_iter->second));

			// Bind vertex buffers only for the attribute locations defined
			command_buffer.bind_vertex_buffers(input_resource.location, std::move(buffers), {0});
		}
	}

	// Draw submesh indexed if indices exists
	if (submesh->vertex_indices != 0)
	{
		// Bind index buffer of submesh
		command_buffer.bind_index_buffer(*submesh->index_buffer, submesh->index_offset, submesh->index_type);
		return true;
	}
	return false;
}
