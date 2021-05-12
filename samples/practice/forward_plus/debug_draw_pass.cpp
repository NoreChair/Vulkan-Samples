#include "debug_draw_pass.h"
#include "gltf_loader.h"

using namespace vkb;
using namespace vkb::sg;

debug_draw_pass::debug_draw_pass(RenderContext &render_context, ShaderSource &&vertex_shader, ShaderSource &&fragment_shader) :
    Subpass{render_context, std::move(vertex_shader), std::move(fragment_shader)}
{
}

debug_draw_pass::~debug_draw_pass()
{
}

void debug_draw_pass::prepare()
{
	auto &device = render_context.get_device();

	RasterizationState rasterState;
	rasterState.polygon_mode = VK_POLYGON_MODE_LINE;
	rasterState.cull_mode    = VK_CULL_MODE_NONE;
	pipeline_state.set_rasterization_state(rasterState);

	DepthStencilState depthState;
	depthState.depth_write_enable = false;
	pipeline_state.set_depth_stencil_state(depthState);

	ColorBlendState           defaultColorState;
	ColorBlendAttachmentState defaultAttaState;
	defaultColorState.attachments.push_back(defaultAttaState);
	pipeline_state.set_color_blend_state(defaultColorState);

	VertexInputState vertexInputState;
	vertexInputState.bindings.emplace_back(VkVertexInputBindingDescription{0, sizeof(float) * 3, VK_VERTEX_INPUT_RATE_VERTEX});
	vertexInputState.attributes.emplace_back(VkVertexInputAttributeDescription{0, 0, VK_FORMAT_R32G32B32_SFLOAT, 0});
	vertexInputState.bindings.emplace_back(VkVertexInputBindingDescription{1, sizeof(instance_attribute), VK_VERTEX_INPUT_RATE_INSTANCE});
	vertexInputState.attributes.emplace_back(VkVertexInputAttributeDescription{1, 1, VK_FORMAT_R32G32B32_SFLOAT, 0});
	vertexInputState.attributes.emplace_back(VkVertexInputAttributeDescription{2, 1, VK_FORMAT_R32G32B32_SFLOAT, sizeof(glm::vec3)});
	vertexInputState.attributes.emplace_back(VkVertexInputAttributeDescription{3, 1, VK_FORMAT_R32G32B32_SFLOAT, sizeof(glm::vec3) * 2});
	pipeline_state.set_vertex_input_state(vertexInputState);

	std::vector<ShaderModule *> modules;
	modules.push_back(&device.get_resource_cache().request_shader_module(VK_SHADER_STAGE_VERTEX_BIT, get_vertex_shader()));
	modules.push_back(&device.get_resource_cache().request_shader_module(VK_SHADER_STAGE_FRAGMENT_BIT, get_fragment_shader()));

	auto &layout = device.get_resource_cache().request_pipeline_layout(modules);
	pipeline_state.set_pipeline_layout(layout);
}

void debug_draw_pass::set_up(vkb::sg::SubMesh *sphere, vkb::sg::SubMesh *cube, vkb::sg::Camera *camera)
{
	render_camera = camera;
	sphere_mesh   = sphere;
	cube_mesh     = cube;
}

void debug_draw_pass::draw(vkb::CommandBuffer &command_buffer)
{
	int cube_count   = (int) bounding_cube.size();
	int sphere_count = (int) bounding_sphere.size();
	int count        = cube_count + sphere_count;
	if (count == 0 || (!draw_sphere && !draw_box))
	{
		return;
	}

	bind_pipeline_state(command_buffer, pipeline_state);
	auto &layout = const_cast<PipelineLayout &>(pipeline_state.get_pipeline_layout());
	command_buffer.bind_pipeline_layout(layout);

	std::vector<std::reference_wrapper<const core::Buffer>> buffers;

	auto &instance_buffer = render_context.get_active_frame().allocate_buffer(VK_BUFFER_USAGE_VERTEX_BUFFER_BIT, sizeof(instance_attribute) * count);
	instance_buffer.update((uint8_t *) bounding_cube.data(), sizeof(instance_attribute) * cube_count, 0);
	instance_buffer.update((uint8_t *) bounding_sphere.data(), sizeof(instance_attribute) * sphere_count, sizeof(instance_attribute) * cube_count);

	BufferAllocation &uniform_buffer = render_context.get_active_frame().allocate_buffer(VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT, sizeof(glm::mat4));
	uniform_buffer.update(vkb::vulkan_style_projection(render_camera->get_projection()) * render_camera->get_view());
	command_buffer.bind_buffer(uniform_buffer.get_buffer(), uniform_buffer.get_offset(), uniform_buffer.get_size(), 0, 0, 0);

	command_buffer.set_vertex_input_state(pipeline_state.get_vertex_input_state());
	if (cube_count && draw_box)
	{
		// Draw Cube
		auto vert_iter = cube_mesh->vertex_buffers.find(std::string("position"));
		buffers.emplace_back(std::reference_wrapper<const core::Buffer>(std::ref(vert_iter->second)));
		buffers.emplace_back(std::reference_wrapper<const core::Buffer>(std::ref(instance_buffer.get_buffer())));
		command_buffer.bind_vertex_buffers(0, buffers, {0, 0});
		if (cube_mesh->vertex_indices != 0)
		{
			command_buffer.bind_index_buffer(*cube_mesh->index_buffer, cube_mesh->index_offset, cube_mesh->index_type);
			command_buffer.draw_indexed(cube_mesh->vertex_indices, cube_count, 0, 0, 0);
		}
		else
		{
			command_buffer.draw(cube_mesh->vertices_count, cube_count, 0, 0);
		}
		buffers.clear();
	}

	if (sphere_count && draw_sphere)
	{
		// Draw Sphere
		auto vert_iter = sphere_mesh->vertex_buffers.find(std::string("position"));
		buffers.emplace_back(std::reference_wrapper<const core::Buffer>(std::ref(vert_iter->second)));
		buffers.emplace_back(std::reference_wrapper<const core::Buffer>(std::ref(instance_buffer.get_buffer())));
		command_buffer.bind_vertex_buffers(0, buffers, {0, sizeof(instance_attribute) * cube_count});
		if (sphere_mesh->vertex_indices != 0)
		{
			command_buffer.bind_index_buffer(*sphere_mesh->index_buffer, sphere_mesh->index_offset, sphere_mesh->index_type);
			command_buffer.draw_indexed(sphere_mesh->vertex_indices, sphere_count, 0, 0, 0);
		}
		else
		{
			command_buffer.draw(sphere_mesh->vertices_count, sphere_count, 0, 0);
		}
	}
}

void debug_draw_pass::bind_pipeline_state(vkb::CommandBuffer &comman_buffer, vkb::PipelineState &pipeline)
{
	comman_buffer.set_color_blend_state(pipeline.get_color_blend_state());
	comman_buffer.set_depth_stencil_state(pipeline.get_depth_stencil_state());
	comman_buffer.set_input_assembly_state(pipeline.get_input_assembly_state());
	comman_buffer.set_rasterization_state(pipeline.get_rasterization_state());
	comman_buffer.set_viewport_state(pipeline.get_viewport_state());
	comman_buffer.set_multisample_state(pipeline.get_multisample_state());
}

void debug_draw_pass::add_bounding_sphere(std::vector<glm::vec3> &&center, std::vector<float> &&radius, std::vector<glm::vec3> &&color)
{
	assert(center.size() == radius.size());
	const float f        = 1.0f / RAND_MAX;
	size_t      src_size = bounding_sphere.size();
	bounding_sphere.resize(src_size + center.size());
	for (size_t i = 0; i < center.size(); i++)
	{
		bounding_sphere[src_size + i].center = center[i];
		bounding_sphere[src_size + i].scale  = glm::vec3(radius[i]);
		bounding_sphere[src_size + i].color  = color.size() == 0 ? glm::vec3(rand() * f, rand() * f, rand() * f) : color[i];
	}
}

void debug_draw_pass::add_bounding_box(std::vector<glm::vec3> &&center, std::vector<glm::vec3> &&extent)
{
	assert(center.size() == extent.size());
	const float f        = 1.0f / RAND_MAX;
	size_t      src_size = bounding_cube.size();
	bounding_cube.resize(src_size + center.size());
	for (size_t i = 0; i < center.size(); i++)
	{
		bounding_cube[src_size + i].center = center[i];
		bounding_cube[src_size + i].scale  = extent[i];
		bounding_cube[src_size + i].color  = glm::vec3(rand() * f, rand() * f, rand() * f);
	}
}