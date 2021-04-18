#version 320 es
precision highp float;

layout(location = 0) in vec3 position;

layout(set = 0, binding = 0) uniform GlobalUniform{
    mat4 model;
    mat4 viewProj;
    vec3 cameraPosition;
};

void main(){
    gl_Position = viewProj * model * vec4(position, 1.0);
}