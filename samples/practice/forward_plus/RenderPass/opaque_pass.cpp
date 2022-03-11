#include "opaque_pass.h"

#include "Utils.h"
#include "ShaderProgram.h"
#include "GraphicContext.h"

#include "common/vk_initializers.h"
#include "scene_graph/components/image.h"
#include "scene_graph/components/material.h"
#include "scene_graph/components/pbr_material.h"
#include "scene_graph/components/texture.h"

using namespace vkb;
using namespace vkb::sg;

opaque_pass::opaque_pass(RenderContext &render_context, ShaderSource &&vertex_shader, ShaderSource &&fragment_shader, VkExtent2D extent) :
    Subpass{render_context, std::move(vertex_shader), std::move(fragment_shader)},
    render_extent(extent)
{
}

opaque_pass::~opaque_pass()
{
}

void opaque_pass::prepare()
{
	DepthStencilState defaultDepthState;
	defaultDepthState.depth_compare_op   = VK_COMPARE_OP_GREATER_OR_EQUAL;
	pipeline_state.set_depth_stencil_state(defaultDepthState);
	defaultDepthState.depth_write_enable = false;
    sky_pipeline_state.set_depth_stencil_state(defaultDepthState);

	RasterizationState rasterState;
	//rasterState.depth_bias_enable = true;
	//pipeline_state.set_rasterization_state(rasterState);

 //   rasterState.depth_bias_enable = false;
    rasterState.cull_mode = VK_CULL_MODE_FRONT_BIT;
    sky_pipeline_state.set_rasterization_state(rasterState);

	ColorBlendState           defaultColorState;
	ColorBlendAttachmentState defaultAttaState;
	defaultColorState.attachments.push_back(defaultAttaState);
	pipeline_state.set_color_blend_state(defaultColorState);
    sky_pipeline_state.set_color_blend_state(defaultColorState);

	auto samplerCreateInfo         = initializers::sampler_create_info();
	samplerCreateInfo.magFilter    = VK_FILTER_LINEAR;
	samplerCreateInfo.minFilter    = VK_FILTER_LINEAR;
	samplerCreateInfo.mipmapMode   = VK_SAMPLER_MIPMAP_MODE_LINEAR;
	samplerCreateInfo.addressModeU = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
	samplerCreateInfo.addressModeV = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
	samplerCreateInfo.addressModeW = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
	samplerCreateInfo.minLod       = 0;
	samplerCreateInfo.maxLod       = VK_LOD_CLAMP_NONE;
	linear_clamp_sampler           = std::make_shared<core::Sampler>(render_context.get_device(), samplerCreateInfo);
}

void opaque_pass::draw(vkb::CommandBuffer & command_buffer, vkb::sg::Camera * camera, std::multimap<float, std::pair<vkb::sg::Node*, vkb::sg::SubMesh*>>* submeshs) {
    this->render_camera = camera;
    this->draw_meshes = submeshs;
    draw(command_buffer);
}

void opaque_pass::draw(vkb::CommandBuffer &command_buffer)
{
	Device &device = render_context.get_device();
	bind_pipeline_state(command_buffer, pipeline_state);
	//command_buffer.set_depth_bias(0.1f, 0.0f, 0.1f);
	for (auto iter = draw_meshes->begin(); iter != draw_meshes->end(); iter++)
	{
		auto node    = iter->second.first;
		auto submesh = iter->second.second;
		auto variant = submesh->get_shader_variant();

		std::vector<ShaderModule *> modules;
		modules.push_back(&device.get_resource_cache().request_shader_module(VK_SHADER_STAGE_VERTEX_BIT, get_vertex_shader(), variant));
		modules.push_back(&device.get_resource_cache().request_shader_module(VK_SHADER_STAGE_FRAGMENT_BIT, get_fragment_shader(), variant));

		PipelineLayout &layout = device.get_resource_cache().request_pipeline_layout(modules);
		command_buffer.bind_pipeline_layout(layout);
		update_global_uniform_buffers(command_buffer, node);
		bind_descriptor(command_buffer, layout, submesh);
		if (bind_vertex_input(command_buffer, layout, submesh))
		{
			command_buffer.draw_indexed(submesh->vertex_indices, 1, 0, 0, 0);
		}
		else
		{
			command_buffer.draw(submesh->vertices_count, 1, 0, 0);
		}
	}
}

void opaque_pass::draw_sky(vkb::CommandBuffer & command_buffer, vkb::sg::SubMesh * sphere) {
    Device &device = render_context.get_device();
    bind_pipeline_state(command_buffer, sky_pipeline_state);

    auto program = ShaderProgram::Find(std::string("sky"));
    PipelineLayout &layout = device.get_resource_cache().request_pipeline_layout(program->GetShaderModules());
    command_buffer.bind_pipeline_layout(layout);

    struct {
        glm::mat4 viewProj;
        glm::vec4 sunDir;
        //glm::vec4 cameraPos;
        //glm::vec4 projOrigin;
        //glm::vec4 projPlaneU;
        //glm::vec4 projPlaneV;
        //glm::vec4 viewport;
    }ubo;

    auto &render_frame = get_render_context().get_active_frame();
    auto  allocation = render_frame.allocate_buffer(VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT, sizeof(ubo), 0);

    glm::mat4 v = render_camera->get_view();
    v[3] = glm::vec4(0.0, 0.0, 0.0, 1.0);
    ubo.viewProj = vkb::vulkan_style_projection(render_camera->get_projection()) * v;
    ubo.sunDir = sunDirection;
    //ubo.cameraPos = glm::vec4(transform.get_translation(), 1.0);

    allocation.update(ubo);
    command_buffer.bind_buffer(allocation.get_buffer(), allocation.get_offset(), allocation.get_size(), 0, 0, 0);

    if (bind_vertex_input(command_buffer, layout, sphere)) {
        command_buffer.draw_indexed(sphere->vertex_indices, 1, 0, 0, 0);
    } else {
        command_buffer.draw(sphere->vertices_count, 1, 0, 0);
    }
}

void opaque_pass::bind_pipeline_state(vkb::CommandBuffer &comman_buffer, vkb::PipelineState &pipeline)
{
	comman_buffer.set_color_blend_state(pipeline.get_color_blend_state());
	comman_buffer.set_depth_stencil_state(pipeline.get_depth_stencil_state());
	comman_buffer.set_input_assembly_state(pipeline.get_input_assembly_state());
	comman_buffer.set_rasterization_state(pipeline.get_rasterization_state());
	comman_buffer.set_viewport_state(pipeline.get_viewport_state());
	comman_buffer.set_multisample_state(pipeline.get_multisample_state());
}

void opaque_pass::update_global_uniform_buffers(vkb::CommandBuffer &commandBuffer, vkb::sg::Node *node)
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

void opaque_pass::bind_descriptor(vkb::CommandBuffer &command_buffer, vkb::PipelineLayout &pipeline_layout, vkb::sg::SubMesh *submesh)
{
	struct
	{
		glm::vec4  direction_light;
		glm::vec4  direction_light_color;
		VkExtent2D viewPort;
		float      inv_tile_dim;
		uint32_t   tile_count_x;
	} lightInfos{sunDirection, sunColor, render_extent, 1.0f / 16.0f, (uint32_t) glm::ceil(render_extent.width / 16.0f)};

	auto allocation = get_render_context().get_active_frame().allocate_buffer(VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT, sizeof(lightInfos), 0);
	allocation.update(lightInfos);
	command_buffer.bind_buffer(allocation.get_buffer(), allocation.get_offset(), allocation.get_size(), 0, 1, 0);

	command_buffer.bind_buffer(*GraphicContext::lightBuffer, 0, GraphicContext::lightBuffer->get_size(), 0, 2, 0);
    command_buffer.bind_buffer(*GraphicContext::lightGridBuffer, 0, GraphicContext::lightGridBuffer ->get_size(), 0, 3, 0);

	DescriptorSetLayout &descriptorSetLayout = pipeline_layout.get_descriptor_set_layout(1);
	for (auto &texture : submesh->get_material()->textures)
	{
		auto layoutBinding = descriptorSetLayout.get_layout_binding(texture.first);
		if (layoutBinding != nullptr)
		{
			command_buffer.bind_image(texture.second->get_image()->get_vk_image_view(),
			                          texture.second->get_sampler()->vk_sampler,
			                          1, layoutBinding->binding, 0);
		}
	}

	command_buffer.bind_image(*screenShadow, *linear_clamp_sampler, 1, 3, 0);

	struct MaterialUniform
	{
		glm::vec4 base_color_factor;
		float     metallic_factor;
		float     roughness_factor;
	};
	auto            pbr_material = dynamic_cast<const sg::PBRMaterial *>(submesh->get_material());
	MaterialUniform material_uniform{};
	material_uniform.base_color_factor = pbr_material->base_color_factor;
	material_uniform.metallic_factor   = pbr_material->metallic_factor;
	material_uniform.roughness_factor  = pbr_material->roughness_factor;

	auto data = to_bytes(material_uniform);

	if (!data.empty())
	{
		command_buffer.push_constants(data);
	}
}

bool opaque_pass::bind_vertex_input(vkb::CommandBuffer &command_buffer, vkb::PipelineLayout &pipeline_layout, vkb::sg::SubMesh *submesh)
{
	auto vertex_input_resources = pipeline_layout.get_resources(ShaderResourceType::Input, VK_SHADER_STAGE_VERTEX_BIT);

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
