#include "shadow_camera.h"

ShadowCamera::ShadowCamera(const std::string &name) :
    vkb::sg::Camera(name)
{
}

std::type_index ShadowCamera::get_type()
{
	return typeid(ShadowCamera);
}

glm::mat4 ShadowCamera::get_projection()
{
	return projection_matrix;
}

glm::mat4 ShadowCamera::get_view()
{
	return world_to_camera_matrix;
}

glm::vec3 ShadowCamera::get_shadow_center()
{
	return shadow_center;
}

glm::vec3 ShadowCamera::get_shadow_bound()
{
	return shadow_bound;
}

void ShadowCamera::set_up(glm::vec3 light_direction, glm::vec3 center, glm::vec3 bound, glm::vec2 shadow_texture_extent, uint32_t shadow_texture_precision)
{
	shadow_bound = bound;

    shadow_texture_precision = glm::min(31u, shadow_texture_precision);
	glm::mat3 world_to_camera_rotation = glm::lookAt(glm::vec3(0.0), light_direction, glm::vec3(0.0, 1.0, 0.0));
	glm::mat3 camera_to_world_rotation = glm::transpose(world_to_camera_rotation);

	glm::vec3 rcp_bound      = 1.0f / shadow_bound;
	glm::vec3 quantize_scale = glm::vec3(shadow_texture_extent, (float) ((1 << shadow_texture_precision) - 1)) * rcp_bound;

	shadow_center = world_to_camera_rotation * center;
	shadow_center = glm::floor(shadow_center * quantize_scale) / quantize_scale;
	shadow_center = camera_to_world_rotation * shadow_center;

	camera_to_world_matrix    = camera_to_world_rotation;
	camera_to_world_matrix[3] = glm::vec4(shadow_center, 1.0);
	world_to_camera_matrix    = glm::inverse(camera_to_world_matrix);

	auto node = get_node();
	if (node->get_parent() == nullptr)
	{
		node->get_transform().set_rotation(glm::toQuat(glm::mat3(camera_to_world_matrix)));
		node->get_transform().set_translation(camera_to_world_matrix[3]);
	}
	else
	{
		glm::mat4 local = camera_to_world_matrix * glm::inverse(node->get_parent()->get_transform().get_world_matrix());
		node->get_transform().set_rotation(glm::toQuat(glm::mat3(local)));
		node->get_transform().set_translation(local[3]);
	}
	projection_matrix = glm::ortho(-bound.x, bound.x, -bound.y, bound.y, -bound.z, bound.z);
}
