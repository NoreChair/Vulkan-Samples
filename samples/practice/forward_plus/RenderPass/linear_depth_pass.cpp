#include "linear_depth_pass.h"
#include "buffer_pool.h"
#include "GraphicContext.h"

linear_depth_pass::linear_depth_pass(vkb::Device *d, vkb::RenderContext *ctx) :
    compute_pass(d),
    context{ctx}
{
}

void linear_depth_pass::dispatch(vkb::CommandBuffer & command_buffer, vkb::core::ImageView* src, vkb::core::ImageView* dest, vkb::sg::Camera * camera) {
    render_camera = camera;
    scene_depth = src;
    linear_depth = dest;
    dispatch(command_buffer);
}

void linear_depth_pass::dispatch(vkb::CommandBuffer &command_buffer)
{
	VkExtent3D extent = scene_depth->get_image().get_extent();

	bind_pipeline(command_buffer);
	command_buffer.bind_input(*scene_depth, 0, 0, 0);
	command_buffer.bind_input(*linear_depth, 0, 1, 0);
	struct
	{
		float    nearPlane;
		float    farPlane;
		uint32_t width;
		uint32_t height;
	} uniforms{render_camera->get_near_plane(), render_camera->get_far_plane(), extent.width, extent.height};

	vkb::BufferAllocation allocation = context->get_active_frame().allocate_buffer(VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT, sizeof(uniforms));
	allocation.update(uniforms);
	command_buffer.bind_buffer(allocation.get_buffer(), allocation.get_offset(), allocation.get_offset(), 0, 2, 0);
	command_buffer.dispatch((uint32_t)glm::ceil(extent.width / 16.0f), (uint32_t)glm::ceil(extent.height / 16.0f), 1);
}
