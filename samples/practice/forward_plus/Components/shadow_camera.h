#pragma once
#include "scene_graph/components/camera.h"
#include "scene_graph/components/light.h"
#include "scene_graph/node.h"

class shadow_camera : public vkb::sg::Camera
{
  public:
	shadow_camera(const std::string &name);

	virtual ~shadow_camera() = default;

	virtual std::type_index get_type() override;

	glm::mat4 get_projection() override;

    glm::mat4 get_projection_without_jitter() override;

	glm::mat4 get_view() override;

	glm::vec3 get_shadow_center();

	glm::vec3 get_shadow_bound();

	float get_far_plane() const { return shadow_bound.z; };

	void set_far_plane(float zfar) {};

	float get_near_plane() const { return -shadow_bound.z; };

	void set_near_plane(float znear) {};

	void set_up(glm::vec3 light_direction, glm::vec3 shadow_center, glm::vec3 shadow_bound, glm::vec2 shadow_texture_extent, uint32_t shadow_texture_precision);

  private:
	vkb::sg::LightType light_type{vkb::sg::LightType::Directional};
	glm::vec3          shadow_center;
	glm::vec3          shadow_bound;
	glm::mat4          world_to_camera_matrix;
	glm::mat4          camera_to_world_matrix;
	glm::mat4          projection_matrix;
	glm::mat4          view_projection_matrix;
	glm::mat4          shadow_matrix;
};