#version 320 es
precision highp float;

layout(location = 0) in vec3 position;
layout(location = 1) in vec3 instance_translate;
layout(location = 2) in vec3 instance_scale;
layout(location = 3) in vec3 instance_color;

layout(location = 0) out vec3 a_color;

layout(set = 0, binding = 0) uniform CameraUniform{
    mat4 viewPorj;
}; 

void main(){
    mat4 instance_matrix = mat4(
        instance_scale.x, 0.0, 0.0, 0.0,
        0.0, instance_scale.y, 0.0, 0.0,
        0.0, 0.0, instance_scale.z, 0.0,
        instance_translate.x, instance_translate.y,  instance_translate.z, 1.0
    );

    a_color = instance_color;
    gl_Position = viewPorj * instance_matrix * vec4(position, 1.0);
}