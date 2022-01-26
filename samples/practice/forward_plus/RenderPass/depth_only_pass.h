#pragma once
#include "common/vk_common.h"
#include "platform/platform.h"
#include "rendering/subpass.h"
#include "scene_graph/components/camera.h"

class depth_only_pass : public vkb::Subpass
{
  public:
	depth_only_pass(vkb::RenderContext &render_context, vkb::ShaderSource &&vertex_shader, vkb::ShaderSource &&fragment_shader);

	virtual ~depth_only_pass();

	void prepare();

	void set_as_shadow_pipeline();

	void draw(vkb::CommandBuffer &comman_buffer, vkb::sg::Camera *camera, std::multimap<float, std::pair<vkb::sg::Node *, vkb::sg::SubMesh *>> *submeshs);

	void draw(vkb::CommandBuffer &comman_buffer);

  private:
	void bind_pipeline_state(vkb::CommandBuffer &comman_buffer, vkb::PipelineState &pipeline);
	void update_global_uniform_buffers(vkb::CommandBuffer &commandBuffer, vkb::sg::Node *node);
	bool bind_vertex_input(vkb::CommandBuffer &command_buffer, vkb::sg::SubMesh *submesh);

	bool isShadowPass = false;

	vkb::PipelineState pipeline_state;
	vkb::sg::Camera *  render_camera{nullptr};

	std::multimap<float, std::pair<vkb::sg::Node *, vkb::sg::SubMesh *>> *draw_meshs;
};
