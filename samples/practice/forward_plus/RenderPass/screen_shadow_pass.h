#pragma once
#include "Components/shadow_camera.h"
#include "compute_pass.h"
#include "core/image_view.h"
#include "core/sampler.h"
#include "rendering/render_context.h"

class screen_shadow_pass : public compute_pass
{
  public:
	screen_shadow_pass(vkb::Device *d, vkb::RenderContext *ctx);

	virtual ~screen_shadow_pass() = default;

	virtual void prepare(std::vector<vkb::ShaderModule *> &shader_module) override;

	void set_up(vkb::core::ImageView *depth, vkb::core::ImageView *shadowMap, vkb::core::ImageView *target);

	void set_camera(vkb::sg::Camera *camera, shadow_camera *light_camera);

	virtual void dispatch(vkb::CommandBuffer &command_buffer) override;

  private:
	std::unique_ptr<vkb::core::Sampler> linearClampSampler;
	std::unique_ptr<vkb::core::Sampler> shadowSampler;
	vkb::core::ImageView *              uavs[3];
	vkb::RenderContext *                context{nullptr};
	vkb::sg::Camera *                   main_camera{nullptr};
	shadow_camera *                     light_camera{nullptr};
};