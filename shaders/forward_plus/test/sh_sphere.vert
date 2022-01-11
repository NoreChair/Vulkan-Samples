#version 320 es
precision highp float;

layout(location = 0) in vec3 position;
layout(location = 1) in vec3 normal;

layout(location = 0) out vec3 a_normal;

layout(set = 0, binding = 0) uniform GlobalUniform{
    mat4 model;
    mat4 viewProj;
    vec4 rSH0;
    vec4 rSH1;
    vec4 gSH0;
    vec4 gSH1;
    vec4 bSH0;
    vec4 bSH1;
    vec4 rgbSH2;
};

void main(){
    a_normal = mat3(model) * normal;
    gl_Position = viewProj * model * vec4(position, 1.0);
}