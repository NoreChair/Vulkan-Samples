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

void depth_only_pass::prepare(vkb::RenderTarget *render_target)
{
	this->render_target = render_target;

	auto &device = render_context.get_device();

	std::vector<LoadStoreInfo> loadStoreInfos;
	loadStoreInfos.emplace_back(LoadStoreInfo{VK_ATTACHMENT_LOAD_OP_DONT_CARE, VK_ATTACHMENT_STORE_OP_DONT_CARE});
	loadStoreInfos.emplace_back(LoadStoreInfo{VK_ATTACHMENT_LOAD_OP_CLEAR, VK_ATTACHMENT_STORE_OP_STORE});

	std::vector<SubpassInfo> subPassInfos;
	subPassInfos.emplace_back(SubpassInfo{{}, {}, {}, false, 0, VK_RESOLVE_MODE_NONE});

	render_pass  = &device.get_resource_cache().request_render_pass(render_target->get_attachments(), loadStoreInfos, subPassInfos);
	frame_buffer = &device.get_resource_cache().request_framebuffer(*render_target, *render_pass);

	DepthStencilState defaultDepthState;
	pipeline_state.set_depth_stencil_state(defaultDepthState);

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

void depth_only_pass::draw(vkb::CommandBuffer &comman_buffer, std::multimap<float, std::pair<vkb::sg::Node *, vkb::sg::SubMesh *>> &submeshs)
{
	Device &device = render_context.get_device();

	std::vector<VkClearValue> clearValue{initializers::clear_color_value(0.0, 0.0, 0.0, 0.0), initializers::clear_depth_stencil_value(0.0, 0)};
	comman_buffer.begin_render_pass(*render_target, *render_pass, *frame_buffer, clearValue);
	// update and bind buffer
	bind_pipeline_state(comman_buffer, pipeline_state);
	for (auto iter = submeshs.begin(); iter != submeshs.end(); iter++)
	{
		auto node    = iter->second.first;
		auto submesh = iter->second.second;
		update_global_uniform_buffers(comman_buffer, node);
		auto &layout = const_cast<PipelineLayout &>(pipeline_state.get_pipeline_layout());
		comman_buffer.bind_pipeline_layout(layout);
		if (bind_vertex_input(comman_buffer, submesh))
		{
			comman_buffer.draw_indexed(submesh->vertex_indices, 1, 0, 0, 0);
		}
		else
		{
			comman_buffer.draw(submesh->vertices_count, 1, 0, 0);
		}
	}

	comman_buffer.end_render_pass();
}

void depth_only_pass::bind_pipeline_state(vkb::CommandBuffer &comman_buffer, vkb::PipelineState &pipeline)
{
	comman_buffer.set_color_blend_state(pipeline.get_color_blend_state());
	comman_buffer.set_depth_stencil_state(pipeline.get_depth_stencil_state());
	comman_buffer.set_input_assembly_state(pipeline.get_input_assembly_state());
	comman_buffer.set_rasterization_state(pipeline.get_rasterization_state());
	comman_buffer.set_viewport_state(pipeline.get_viewport_state());
	comman_buffer.set_multisample_state(pipeline.get_multisample_state());
}

void depth_only_pass::update_global_uniform_buffers(vkb::CommandBuffer &commandBuffer, vkb::sg::Node *node)
{
	auto &render_frame = get_render_context().get_active_frame();
	auto  allocation   = render_frame.allocate_buffer(VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT, sizeof(GlobalUniform), 0);
	auto &transform    = node->get_transform();

	GlobalUniform globalUniform;
	globalUniform.model           = transform.get_world_matrix();
	globalUniform.view_project    = vkb::vulkan_style_projection(render_camera->get_projection()) * render_camera->get_view();
	globalUniform.camera_position = render_camera->get_node()->get_component<vkb::sg::Transform>().get_translation();
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
