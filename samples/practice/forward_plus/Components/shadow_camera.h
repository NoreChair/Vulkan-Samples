#pragma once
#include "scene_graph/components/camera.h"
#include "scene_graph/components/light.h"

class shadow_camera : public vkb::sg::Component
{
  public:
	shadow_camera(vkb::sg::LightType type, const std::string &name);

	virtual ~shadow_camera() = default;

	virtual std::type_index get_type() override;

	glm::mat4 get_shadow_matrix();

	void set_up(glm::vec3 light_direction, glm::vec3 shadow_center, glm::vec3 shadow_bound, glm::vec2 shadow_texture_extent, uint32_t shadow_texture_precision);

  private:
	bool dirty{true};
	vkb::sg::LightType light_type{vkb::sg::LightType::Directional};

	glm::mat4 shadow_matrix;
};