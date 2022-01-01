#version 320 es
precision highp float;

layout(location = 0) in vec3 position;

layout(location = 0) out vec3 v_normal;

layout(set = 0, binding = 0) uniform GlobalUniform{
    mat4 viewProj;
    vec4 sunDir;
    // vec4 cameraPos;
    // vec4 projOrigin;
    // vec4 projPlaneU;
    // vec4 projPlaneV;
    // vec4 viewport;
};

void main(){
    v_normal = position;
    gl_Position = viewProj * vec4(position, 1.0);
    gl_Position.z = 1e-10;
}