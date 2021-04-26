#version 320 es
precision highp float;

layout(location = 0) in vec2 v_uv;

layout(location = 0) out vec4 o_color;

layout(set = 0, binding = 0) uniform sampler2D linearDepth;

void main(){
    float depth = texture(linearDepth, v_uv).r;
    o_color = vec4(depth, depth, depth, 1.0);
}