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

void debug_draw_pass::prepare(vkb::sg::Camera *camera, vkb::RenderTarget *render_target)
{
	this->render_camera = camera;
	this->render_target = render_target;

	auto &device = render_context.get_device();

	std::vector<LoadStoreInfo> loadStoreInfos;
	loadStoreInfos.emplace_back(LoadStoreInfo{VK_ATTACHMENT_LOAD_OP_LOAD, VK_ATTACHMENT_STORE_OP_STORE});
	loadStoreInfos.emplace_back(LoadStoreInfo{VK_ATTACHMENT_LOAD_OP_LOAD, VK_ATTACHMENT_STORE_OP_STORE});

	std::vector<SubpassInfo> subPassInfos;
	subPassInfos.emplace_back(SubpassInfo{{}, {0}, {}, false, 0, VK_RESOLVE_MODE_NONE});
	render_pass  = &device.get_resource_cache().request_render_pass(render_target->get_attachments(), loadStoreInfos, subPassInfos);
	frame_buffer = &device.get_resource_cache().request_framebuffer(*render_target, *render_pass);

	RasterizationState rasterState;
	rasterState.polygon_mode = VK_POLYGON_MODE_LINE;
	pipeline_state.set_rasterization_state(rasterState);

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

void debug_draw_pass::set_up(vkb::sg::SubMesh *sphere, vkb::sg::SubMesh *cube)
{
	sphere_mesh = sphere;
	cube_mesh   = cube;
}

void debug_draw_pass::draw(vkb::CommandBuffer &command_buffer)
{
	command_buffer.begin_render_pass(*render_target, *render_pass, *frame_buffer, {});
	bind_pipeline_state(command_buffer, pipeline_state);
	auto &layout = const_cast<PipelineLayout &>(pipeline_state.get_pipeline_layout());
	command_buffer.bind_pipeline_layout(layout);

	std::vector<std::reference_wrapper<const core::Buffer>> buffers;

	int   count           = bounding_cube.size() + bounding_sphere.size();
	auto &instance_buffer = render_context.get_active_frame().allocate_buffer(VK_BUFFER_USAGE_VERTEX_BUFFER_BIT, sizeof(instance_attribute) * count);
	instance_buffer.update((uint8_t *) bounding_cube.data(), sizeof(instance_attribute) * bounding_cube.size(), 0);
	instance_buffer.update((uint8_t *) bounding_sphere.data(), sizeof(instance_attribute) * bounding_sphere.size(), sizeof(instance_attribute) * bounding_cube.size());

	{
		// Draw Cube
		auto vert_iter = cube_mesh->vertex_buffers.find(std::string("position"));
		buffers.emplace_back(std::reference_wrapper<const core::Buffer>(std::ref(vert_iter->second)));
		buffers.emplace_back(std::reference_wrapper<const core::Buffer>(std::ref(instance_buffer.get_buffer())));
		command_buffer.bind_vertex_buffers(0, buffers, {0, 0});
		if (cube_mesh->vertex_indices != 0)
		{
			command_buffer.bind_index_buffer(*cube_mesh->index_buffer, cube_mesh->index_offset, cube_mesh->index_type);
			command_buffer.draw_indexed(cube_mesh->vertex_indices, bounding_cube.size(), 0, 0, 0);
		}
		else
		{
			command_buffer.draw(cube_mesh->vertices_count, bounding_cube.size(), 0, 0);
		}
	}

	buffers.clear();
	{
		// Draw Sphere
		auto vert_iter = sphere_mesh->vertex_buffers.find(std::string("position"));
		buffers.emplace_back(std::reference_wrapper<const core::Buffer>(std::ref(vert_iter->second)));
		buffers.emplace_back(std::reference_wrapper<const core::Buffer>(std::ref(instance_buffer.get_buffer())));
		command_buffer.bind_vertex_buffers(0, buffers, {0, sizeof(instance_attribute) * bounding_cube.size()});
		if (sphere_mesh->vertex_indices != 0)
		{
			command_buffer.bind_index_buffer(*sphere_mesh->index_buffer, sphere_mesh->index_offset, sphere_mesh->index_type);
			command_buffer.draw_indexed(sphere_mesh->vertex_indices, bounding_sphere.size(), 0, 0, 0);
		}
		else
		{
			command_buffer.draw(sphere_mesh->vertices_count, bounding_sphere.size(), 0, 0);
		}
	}
	command_buffer.end_render_pass();
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

void debug_draw_pass::add_bounding_sphere(std::vector<glm::vec3> &&center, std::vector<float> &&radius)
{
	assert(center.size() == radius.size());
	size_t src_size = bounding_sphere.size();
	bounding_sphere.resize(src_size + center.size());
	for (size_t i = 0; i < center.size(); i++)
	{
		bounding_sphere[src_size + i].center = center[i];
		bounding_sphere[src_size + i].scale  = glm::vec3(radius[i]);
		bounding_sphere[src_size + i].color  = glm::vec3(0.0, 1.0, 0.0);
	}
}

void debug_draw_pass::add_bounding_box(std::vector<glm::vec3> &&center, std::vector<glm::vec3> &&extent)
{
	assert(center.size() == extent.size());
	size_t src_size = bounding_cube.size();
	bounding_cube.resize(src_size + center.size());
	for (size_t i = 0; i < center.size(); i++)
	{
		bounding_cube[src_size + i].center = center[i];
		bounding_cube[src_size + i].scale  = extent[i];
		bounding_cube[src_size + i].color  = glm::vec3(0.0, 0.0, 1.0);
	}
}