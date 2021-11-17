#version 320 es
precision highp float;

layout(location = 0) in vec3 position;
layout(location = 1) in vec2 texcoord_0;
layout(location = 2) in vec3 normal;

layout(location = 0) out vec3 v_pos;
layout(location = 1) out vec2 v_uv;
layout(location = 2) out vec3 v_normal;

layout(set = 0, binding = 0) uniform GlobalUniform
{
	mat4 _viewProj;
    vec4 _lightDirIntensity;
    vec4 _lightColor;
    vec4 _cameraPos;
};

layout(set = 0, binding = 1) uniform LocalUniform{
    mat4 _model;
    mat4 _invModel;
    vec4 _baseColor;
};

void main(){
    vec4 worldPos = _model * vec4(position, 1.0);

    v_pos    = worldPos.xyz;
    v_uv     = texcoord_0;
    v_normal = (_invModel * vec4(normal, 0.0)).rgb;

    gl_Position = _viewProj * worldPos;
}
