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

	void prepare(vkb::sg::Camera *camera, vkb::RenderTarget *render_target);
	void draw(vkb::CommandBuffer &comman_buffer, std::multimap<float, std::pair<vkb::sg::Node *, vkb::sg::SubMesh *>> &submeshs);

  private:
	void prepare(){};
	void draw(vkb::CommandBuffer &comman_buffer){};
	void bind_pipeline_state(vkb::CommandBuffer &comman_buffer, vkb::PipelineState &pipeline);
	void update_global_uniform_buffers(vkb::CommandBuffer &commandBuffer, vkb::sg::Node *node);
	bool bind_vertex_input(vkb::CommandBuffer &command_buffer, vkb::sg::SubMesh *submesh);

	vkb::PipelineState pipeline_state;
	vkb::sg::Camera *  render_camera{nullptr};
	vkb::RenderPass *  render_pass{nullptr};
	vkb::Framebuffer * frame_buffer{nullptr};
	vkb::RenderTarget *render_target{nullptr};
};
