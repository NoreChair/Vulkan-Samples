#version 320 es
precision highp float;

layout(location = 0) in vec3 v_pos;

layout(location = 0) out vec4 o_color;

layout(set = 0, binding = 0) uniform GlobalUniform{
    mat4 viewProj;
};

layout(set = 0, binding = 1) uniform samplerCube skyTexture;

void main(){
    vec3 viewDir = normalize(v_pos);
    o_color = texture(skyTexture, viewDir);
}