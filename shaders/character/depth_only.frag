#version 320 es
precision highp float;

layout(location = 0) out vec4 o_color;

layout(set = 0, binding = 0) uniform GlobalUniform{
    mat4 model;
    mat4 viewProj;
    float normalBias;
};

void main(){
    o_color = vec4(0.0, 0.0, 0.0, 0.0);
}