#pragma once
#include "compute_pass.h"
#include "core/image_view.h"
#include "rendering/render_context.h"
#include "scene_graph/components/camera.h"

class linear_depth_pass : public compute_pass
{
  public:
	linear_depth_pass(vkb::Device *d, vkb::RenderContext *ctx);

	virtual ~linear_depth_pass() = default;

	void set_up(vkb::core::ImageView *src, vkb::core::ImageView *dest, vkb::sg::Camera *camera);

	void dispatch(vkb::CommandBuffer &command_buffer) override;

  private:
	vkb::RenderContext   *context{nullptr};
	vkb::sg::Camera		 *render_camera{nullptr};
	vkb::core::ImageView *scene_depth{nullptr};
	vkb::core::ImageView *linear_depth{nullptr};
};
