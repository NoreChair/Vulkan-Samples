#version 320 es
precision highp float;

layout(location = 0) in vec3 position;
layout(location = 1) in vec2 texcoord_0;
layout(location = 2) in vec3 normal;

layout(location = 0) out vec3 v_normal;
layout(location = 1) out vec2 v_uv;

layout(set = 0, binding = 0) uniform GlobalUniform{
    mat4 viewProj;
    vec4 lightDirIntensity;
    vec3 lightColor;
};

layout(set = 0, binding = 1) uniform LocalUniform{
    mat4 model;
    mat4 invModel;
};

void main(){
    vec4 worldPos = model * vec4(position, 1.0);
    gl_Position = viewProj * worldPos;
    v_uv = texcoord_0;
    v_normal = normalize(invModel * vec4(normal, 0.0)).xyz;
}
