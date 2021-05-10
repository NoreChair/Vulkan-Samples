#pragma once
#include "common/vk_common.h"
#include "platform/platform.h"
#include "rendering/subpass.h"
#include "scene_graph/components/camera.h"
#include "scene_graph/components/sub_mesh.h"
class debug_draw_pass : vkb::Subpass
{
	struct instance_attribute
	{
		glm::vec3 center;
		glm::vec3 scale;
		glm::vec3 color;
	};

  public:
	debug_draw_pass(vkb::RenderContext &render_context, vkb::ShaderSource &&vertex_shader, vkb::ShaderSource &&fragment_shader);
	virtual ~debug_draw_pass();

	void prepare(vkb::RenderTarget *render_target);
	void set_up(vkb::sg::SubMesh *sphere, vkb::sg::SubMesh *cube, vkb::sg::Camera *camera);
	void draw(vkb::CommandBuffer &command_buffer);
	void bind_pipeline_state(vkb::CommandBuffer &comman_buffer, vkb::PipelineState &pipeline);

	void add_bounding_sphere(std::vector<glm::vec3> &&center, std::vector<float> &&radius);
	void add_bounding_box(std::vector<glm::vec3> &&center, std::vector<glm::vec3> &&extent);
	void reset_bounding_sphere() { bounding_sphere.clear(); }
	void reset_bounding_box() { bounding_cube.clear(); }

  private:
	void prepare(){};

	std::vector<instance_attribute> bounding_sphere;
	std::vector<instance_attribute> bounding_cube;

	vkb::PipelineState pipeline_state;
	vkb::sg::Camera *  render_camera{nullptr};
	vkb::RenderPass *  render_pass{nullptr};
	vkb::Framebuffer * frame_buffer{nullptr};
	vkb::RenderTarget *render_target{nullptr};

	vkb::sg::SubMesh *sphere_mesh{nullptr};
	vkb::sg::SubMesh *cube_mesh{nullptr};
};
