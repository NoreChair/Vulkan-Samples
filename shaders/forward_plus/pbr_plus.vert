#version 320 es
precision highp float;

layout(location = 0) in vec3 position;
layout(location = 1) in vec2 texcoord_0;
layout(location = 2) in vec3 normal;

layout(location = 0) out vec3 a_pos;
layout(location = 1) out vec2 a_uv;
layout(location = 2) out vec3 a_normal;

layout(set = 0, binding = 0) uniform GlobalUniform
{
	mat4  model;
	mat4  view;
	mat4  proj;
	vec3  camera_position;
};

void main(){
    vec4 world_pos = model * vec4(position, 1.0);

    a_pos    = world_pos.xyz;
    a_uv     = texcoord_0;
    a_normal = (model * vec4(normal, 0.0)).rgb;

	gl_Position = proj * view * world_pos;
}
