#pragma once
#include "Components/shadow_camera.h"
#include "compute_pass.h"
#include "core/image_view.h"
#include "rendering/render_context.h"

class screen_shadow_pass : public compute_pass
{
  public:
	screen_shadow_pass(vkb::Device *d, vkb::RenderContext *ctx);

	virtual ~screen_shadow_pass() = default;

	void set_up(vkb::core::ImageView *depth, vkb::core::ImageView *shadowMap, vkb::core::ImageView *target);

	void set_camera(vkb::sg::Camera *camera, shadow_camera *light_camera);

	virtual void dispatch(vkb::CommandBuffer &command_buffer);

  private:
	vkb::core::ImageView *uavs[3];
	vkb::RenderContext *  context{nullptr};
	vkb::sg::Camera *     main_camera{nullptr};
	shadow_camera *       light_caemra{nullptr};
};