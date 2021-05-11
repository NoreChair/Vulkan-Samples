#pragma once
#include "common/vk_common.h"
#include "platform/platform.h"
#include "rendering/subpass.h"

class show_depth_pass : public vkb::Subpass
{
  public:
	show_depth_pass(vkb::RenderContext &render_context, vkb::ShaderSource &&vertex_shader, vkb::ShaderSource &&fragment_shader);
	virtual ~show_depth_pass();

	void prepare();
	void set_up(vkb::core::Buffer *vertex_buffer, vkb::core::ImageView *image_view);
	void draw(vkb::CommandBuffer &command_buffer);

  private:
	void bind_pipeline_state(vkb::CommandBuffer &comman_buffer, vkb::PipelineState &pipeline);

	vkb::PipelineState pipeline_state;

	vkb::core::Buffer    *screen_quad{nullptr};
	vkb::core::ImageView *depth_image_view{nullptr};

	std::shared_ptr<vkb::core::Sampler> linear_clamp_sampler{nullptr};
};
