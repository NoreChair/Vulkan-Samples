#pragma once
#include "compute_pass.h"
#include "core/buffer.h"
#include "core/image_view.h"
#include "rendering/render_context.h"
#include "scene_graph/components/camera.h"

class light_grid_pass : public compute_pass
{
  public:
	light_grid_pass(vkb::Device *d, vkb::RenderContext *ctx);

	virtual ~light_grid_pass() = default;

	void set_up(vkb::core::Buffer *data, vkb::core::Buffer *grid, vkb::core::ImageView *depth, vkb::sg::Camera *camera);

	void dispatch(vkb::CommandBuffer &command_buffer) override;

  private:
	const uint32_t light_count = 128;

	vkb::RenderContext   *context{nullptr};
	vkb::sg::Camera      *render_camera{nullptr};
	vkb::core::ImageView *linear_depth{nullptr};
	vkb::core::Buffer    *light_data{nullptr};
	vkb::core::Buffer    *light_grid{nullptr};
};
