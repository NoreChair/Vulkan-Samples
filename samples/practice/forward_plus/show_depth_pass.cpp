#include "show_depth_pass.h"
#include "common/vk_initializers.h"
#include "scene_graph/components/image.h"
#include "rendering/render_context.h"

using namespace vkb;
using namespace vkb::sg;

show_depth_pass::show_depth_pass(RenderContext &render_context, ShaderSource &&vertex_shader, ShaderSource &&fragment_shader) :
	Subpass{ render_context, std::move(vertex_shader), std::move(fragment_shader) }
{
}

show_depth_pass::~show_depth_pass()
{
}

void show_depth_pass::prepare(vkb::RenderTarget *render_target)
{
	this->render_target = render_target;
	auto &device = render_context.get_device();

	std::vector<LoadStoreInfo> loadStoreInfos;
	loadStoreInfos.emplace_back(LoadStoreInfo{ VK_ATTACHMENT_LOAD_OP_CLEAR, VK_ATTACHMENT_STORE_OP_STORE });
	loadStoreInfos.emplace_back(LoadStoreInfo{ VK_ATTACHMENT_LOAD_OP_DONT_CARE, VK_ATTACHMENT_STORE_OP_DONT_CARE });

	std::vector<SubpassInfo> subPassInfos;
	subPassInfos.emplace_back(SubpassInfo{ {}, {0}, {}, false, 0, VK_RESOLVE_MODE_NONE });

	render_pass = &device.get_resource_cache().request_render_pass(render_target->get_attachments(), loadStoreInfos, subPassInfos);
	frame_buffer = &device.get_resource_cache().request_framebuffer(*render_target, *render_pass);

	DepthStencilState postProcessDepthState;        // depth test/write off
	postProcessDepthState.depth_test_enable = false;
	postProcessDepthState.depth_write_enable = false;
	postProcessDepthState.depth_compare_op = VK_COMPARE_OP_ALWAYS;
	pipeline_state.set_depth_stencil_state(postProcessDepthState);

	ColorBlendState           defaultColorState;
	ColorBlendAttachmentState defaultAttaState;
	defaultColorState.attachments.push_back(defaultAttaState);
	pipeline_state.set_color_blend_state(defaultColorState);

	VertexInputState vertexInputState;
	vertexInputState.bindings.emplace_back(VkVertexInputBindingDescription{ 0, sizeof(float) * 3, VK_VERTEX_INPUT_RATE_VERTEX });
	vertexInputState.attributes.emplace_back(VkVertexInputAttributeDescription{ 0, 0, VK_FORMAT_R32G32B32_SFLOAT, 0 });
	pipeline_state.set_vertex_input_state(vertexInputState);

	std::vector<ShaderModule *> modules;
	modules.push_back(&device.get_resource_cache().request_shader_module(VK_SHADER_STAGE_VERTEX_BIT, get_vertex_shader()));
	modules.push_back(&device.get_resource_cache().request_shader_module(VK_SHADER_STAGE_FRAGMENT_BIT, get_fragment_shader()));

	auto &layout = device.get_resource_cache().request_pipeline_layout(modules);
	pipeline_state.set_pipeline_layout(layout);

	auto samplerCreateInfo = initializers::sampler_create_info();
	samplerCreateInfo.magFilter = VK_FILTER_LINEAR;
	samplerCreateInfo.minFilter = VK_FILTER_LINEAR;
	samplerCreateInfo.mipmapMode = VK_SAMPLER_MIPMAP_MODE_LINEAR;
	samplerCreateInfo.addressModeU = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
	samplerCreateInfo.addressModeV = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
	samplerCreateInfo.addressModeW = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
	samplerCreateInfo.minLod = 0;
	samplerCreateInfo.maxLod = VK_LOD_CLAMP_NONE;
	linear_clamp_sampler = std::make_shared<core::Sampler>(device, samplerCreateInfo);
}

void show_depth_pass::set_up(vkb::core::Buffer *vertex_buffer, vkb::core::ImageView *image_view)
{
	screen_quad = vertex_buffer;
	depth_image_view = image_view;
}

void show_depth_pass::draw(vkb::CommandBuffer &command_buffer)
{
	std::vector<VkClearValue> clearValue{ initializers::clear_color_value(0.0, 0.0, 0.0, 0.0), initializers::clear_depth_stencil_value(0.0, 0) };
	command_buffer.begin_render_pass(*render_target, *render_pass, *frame_buffer, clearValue);
	bind_pipeline_state(command_buffer, pipeline_state);
	const PipelineLayout &layout = pipeline_state.get_pipeline_layout();
	command_buffer.bind_pipeline_layout(const_cast<PipelineLayout &>(layout));
	command_buffer.bind_image(*depth_image_view, *linear_clamp_sampler, 0, 0, 0);
	command_buffer.set_vertex_input_state(pipeline_state.get_vertex_input_state());
	command_buffer.bind_vertex_buffers(0, { *screen_quad }, { 0 });
	command_buffer.draw(3, 1, 0, 0);
	command_buffer.end_render_pass();
}

void show_depth_pass::bind_pipeline_state(vkb::CommandBuffer &comman_buffer, vkb::PipelineState &pipeline)
{
	comman_buffer.set_color_blend_state(pipeline.get_color_blend_state());
	comman_buffer.set_depth_stencil_state(pipeline.get_depth_stencil_state());
	comman_buffer.set_input_assembly_state(pipeline.get_input_assembly_state());
	comman_buffer.set_rasterization_state(pipeline.get_rasterization_state());
	comman_buffer.set_viewport_state(pipeline.get_viewport_state());
	comman_buffer.set_multisample_state(pipeline.get_multisample_state());
}
