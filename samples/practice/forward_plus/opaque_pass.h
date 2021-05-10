#pragma once
#include "common/vk_common.h"
#include "platform/platform.h"
#include "rendering/subpass.h"
#include "scene_graph/components/camera.h"
#include <map>

class opaque_pass : public vkb::Subpass
{
  public:
	opaque_pass(vkb::RenderContext &render_context, vkb::ShaderSource &&vertex_shader, vkb::ShaderSource &&fragment_shader);
	virtual ~opaque_pass();

	void prepare(vkb::RenderTarget *render_target);
	void set_up(vkb::core::Buffer *light_grid, vkb::core::Buffer *light_data, vkb::sg::Camera *camera);
	void draw(vkb::CommandBuffer &comman_buffer, std::multimap<float, std::pair<vkb::sg::Node *, vkb::sg::SubMesh *>> &submeshs);

  private:
	void prepare(){};
	void draw(vkb::CommandBuffer &comman_buffer){};
	void bind_pipeline_state(vkb::CommandBuffer &comman_buffer, vkb::PipelineState &pipeline);
	void update_global_uniform_buffers(vkb::CommandBuffer &commandBuffer, vkb::sg::Node *node);
	void bind_descriptor(vkb::CommandBuffer &command_buffer, vkb::PipelineLayout &pipeline_layout, vkb::sg::SubMesh *submesh);
	bool bind_vertex_input(vkb::CommandBuffer &command_buffer, vkb::PipelineLayout &pipeline_layout, vkb::sg::SubMesh *submesh);

	vkb::PipelineState pipeline_state;
	vkb::sg::Camera   *render_camera{nullptr};
	vkb::RenderPass   *render_pass{nullptr};
	vkb::Framebuffer  *frame_buffer{nullptr};
	vkb::RenderTarget *render_target{nullptr};
	vkb::core::Buffer *light_grid_buffer{nullptr};
	vkb::core::Buffer *light_data_buffer{nullptr};

	glm::vec4 sunDirection = glm::normalize(glm::vec4(-1.0, -1.0, 0.0, 0.0));
	glm::vec4 sunColor     = glm::vec4(1.0, 1.0, 1.0, 1.0);
};