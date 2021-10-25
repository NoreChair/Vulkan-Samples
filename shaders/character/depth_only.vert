#version 320 es
precision highp float;

layout(location = 0) in vec3 position;
layout(location = 1) in vec3 normal;

layout(set = 0, binding = 0) uniform GlobalUniform{
    mat4 model;
    mat4 viewProj;
    float normalBias;
};

void main(){
    vec3 posOS = position - normal * normalBias;
    gl_Position = viewProj * model * vec4(posOS, 1.0);
}
