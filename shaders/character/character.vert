#version 320 es
precision highp float;

layout(location = 0) in vec3 position;
layout(location = 1) in vec2 texcoord_0;
layout(location = 2) in vec3 normal;

layout(location = 0) out vec3 v_posWS;
layout(location = 1) out vec3 v_normal;
layout(location = 2) out vec2 v_uv;

layout(set = 0, binding = 0) uniform GlobalUniform{
    mat4 model;
    mat4 viewProj;
    mat4 invModel;
    vec4 lightDirIntensity;
};

void main(){
    vec4 worldPos = model * vec4(position, 1.0);
    gl_Position = viewProj * worldPos;
    v_posWS = worldPos.xyz;
    v_uv = texcoord_0;
    v_normal = normalize(invModel * vec4(normal, 0.0)).xyz;
}