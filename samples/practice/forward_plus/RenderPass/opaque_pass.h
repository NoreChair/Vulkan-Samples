#pragma once
#include "common/vk_common.h"
#include "platform/platform.h"
#include "rendering/subpass.h"
#include "scene_graph/components/camera.h"
#include <map>

class opaque_pass : public vkb::Subpass
{
  public:
	opaque_pass(vkb::RenderContext &render_context, vkb::ShaderSource &&vertex_shader, vkb::ShaderSource &&fragment_shader, VkExtent2D extent);
	virtual ~opaque_pass();

	void prepare();
	void set_up(vkb::core::Buffer *light_grid, vkb::core::Buffer *light_data, vkb::sg::Camera *camera, std::multimap<float, std::pair<vkb::sg::Node *, vkb::sg::SubMesh *>> *submeshs);
	void draw(vkb::CommandBuffer &comman_buffer);
    void draw_sky(vkb::CommandBuffer &command_buffer, vkb::sg::SubMesh* sphere);
	vkb::core::ImageView *screenShadow{nullptr};

	glm::vec4 sunDirection = glm::vec4(0.0, -1.0, 0.0, 0.0);
	glm::vec4 sunColor     = glm::vec4(1.0, 1.0, 1.0, 1.0);

  private:
	void bind_pipeline_state(vkb::CommandBuffer &comman_buffer, vkb::PipelineState &pipeline);
	void update_global_uniform_buffers(vkb::CommandBuffer &commandBuffer, vkb::sg::Node *node);
	void bind_descriptor(vkb::CommandBuffer &command_buffer, vkb::PipelineLayout &pipeline_layout, vkb::sg::SubMesh *submesh);
	bool bind_vertex_input(vkb::CommandBuffer &command_buffer, vkb::PipelineLayout &pipeline_layout, vkb::sg::SubMesh *submesh);

	VkExtent2D         render_extent;
	vkb::PipelineState pipeline_state;
    vkb::PipelineState sky_pipeline_state;
	vkb::sg::Camera *  render_camera{nullptr};
	vkb::core::Buffer *light_grid_buffer{nullptr};
	vkb::core::Buffer *light_data_buffer{nullptr};

	std::shared_ptr<vkb::core::Sampler> linear_clamp_sampler{nullptr};

	std::multimap<float, std::pair<vkb::sg::Node *, vkb::sg::SubMesh *>> *draw_meshes;
};