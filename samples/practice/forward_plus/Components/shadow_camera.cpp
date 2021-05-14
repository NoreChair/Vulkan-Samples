#include "shadow_camera.h"
#include "scene_graph/node.h"

shadow_camera::shadow_camera(vkb::sg::LightType type, const std::string &name) :
    Component(name),
    light_type{type}
{
}

std::type_index shadow_camera::get_type()
{
	return typeid(shadow_camera);
}

glm::mat4 shadow_camera::get_shadow_matrix()
{
	if (!dirty)
	{
		return shadow_matrix;
	}
}

void shadow_camera::set_up(glm::vec3 light_direction, glm::vec3 shadow_center, glm::vec3 shadow_bound, glm::vec2 shadow_texture_extent, uint32_t shadow_texture_precision)
{
	glm::mat3 camera_to_world = glm::lookAtRH(glm::vec3(0.0), light_direction, glm::vec3(0.0, 0.0, 1.0));
	glm::quat world_to_camera = glm::transpose(camera_to_world);

	glm::vec3 rcp_bound      = 1.0f / shadow_bound;
	glm::vec3 quantize_scale = glm::vec3(shadow_texture_extent, (float) (1 << shadow_texture_precision - 1)) * rcp_bound;

	shadow_center = world_to_camera * shadow_center;
	shadow_center = glm::floor(shadow_center * quantize_scale) / quantize_scale;
	shadow_center = camera_to_world * shadow_center;



	dirty = false;
}
