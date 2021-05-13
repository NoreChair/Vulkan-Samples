#include "light_grid_pass.h"
#include "rendering/subpass.h"

light_grid_pass::light_grid_pass(vkb::Device *d, vkb::RenderContext *ctx) :
    compute_pass(d),
    context(ctx)
{
}

void light_grid_pass::set_up(vkb::core::Buffer *data, vkb::core::Buffer *grid, vkb::core::ImageView *depth, vkb::sg::Camera *camera)
{
	light_data    = data;
	light_grid    = grid;
	linear_depth  = depth;
	render_camera = camera;
}

void light_grid_pass::dispatch(vkb::CommandBuffer &command_buffer)
{
	bind_pipeline(command_buffer);

	VkExtent3D extent        = linear_depth->get_image().get_extent();
	VkExtent2D dispatchCount = {(uint32_t) glm::ceil(extent.width / 16.0f), (uint32_t) glm::ceil(extent.height / 16.0f)};
	float      near_plane    = render_camera->get_near_plane();
	float      far_plane     = render_camera->get_far_plane();

	struct
	{
		glm::mat4  viewMatrix;
		glm::mat4  projMatrix;
		VkExtent2D viewport;
		uint32_t   tileCountX;
		uint32_t   lightBufferCount;
		float      invTileDim;
		float      rcpZMagic;
	} uniforms{
	    render_camera->get_view(),
	    vkb::vulkan_style_projection(render_camera->get_projection()),
	    {extent.width, extent.height},
	    dispatchCount.width,
	    light_count,
	    1.0f / 16.0f,
	    near_plane / (far_plane - near_plane)};

	vkb::BufferAllocation allocation = context->get_active_frame().allocate_buffer(VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT, sizeof(uniforms));
	allocation.update(uniforms);
	command_buffer.bind_buffer(allocation.get_buffer(), allocation.get_offset(), allocation.get_offset(), 0, 0, 0);
	command_buffer.bind_buffer(*light_data, 0, light_data->get_size(), 0, 1, 0);
	command_buffer.bind_buffer(*light_grid, 0, light_grid->get_size(), 0, 2, 0);
	//command_buffer.bind_buffer(*lightMaskBuffer, 0, lightMaskBuffer->get_size(), 0, 3, 0);
	command_buffer.bind_image(*linear_depth, 0, 4, 0);
	command_buffer.dispatch(dispatchCount.width, dispatchCount.height, 1);
}