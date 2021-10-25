#version 320 es
precision highp float;

layout(location = 0) in vec3 position;

layout(location = 0) out vec3 v_pos;

layout(set = 0, binding = 0) uniform GlobalUniform{
    mat4 viewProj;
};

void main(){
    v_pos = position;
    gl_Position = viewProj * vec4(position, 1.0);
    gl_Position.z = 0.0001;
}