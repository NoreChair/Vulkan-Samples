#pragma once
#include "glm/glm.hpp"

struct alignas(16) GlobalUniform
{
	glm::mat4 model;
	glm::mat4 view_project;
	glm::vec3 camera_position;
};

struct LightBuffer
{
	glm::vec3 position;          // xyz=pos
	glm::vec3 color;             // xyz=rgb
	glm::vec3 coneDir;           // xyz=direction if light source is spot
	glm::vec3 coneAngles;        // x=1.0f/(cos(inner)-cos(outer)), y=cos(inner), z=cos(outer/2)
	float     radius;
	float     intensity;
	uint32_t  lightType;
	uint32_t  padding;
};