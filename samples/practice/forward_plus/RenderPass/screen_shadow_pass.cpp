#include "screen_shadow_pass.h"

screen_shadow_pass::screen_shadow_pass(vkb::Device *d, vkb::RenderContext *ctx) :
    compute_pass(d),
    context(ctx)
{
}

void screen_shadow_pass::set_up(vkb::core::ImageView *depth, vkb::core::ImageView *shadowMap, vkb::core::ImageView *target)
{
	uavs[0] = depth;
	uavs[1] = shadowMap;
	uavs[2] = target;
}

void screen_shadow_pass::set_camera(vkb::sg::Camera *camera, shadow_camera *light_camera)
{
	main_camera        = camera;
	this->light_caemra = light_caemra;
}

void screen_shadow_pass::dispatch(vkb::CommandBuffer &command_buffer)
{
	bind_pipeline(command_buffer);


}
