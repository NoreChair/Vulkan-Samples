#version 320 es
precision highp float;

layout(location = 0) in vec2 v_uv;

layout(location = 0) out vec4 o_color;

layout(set = 0, binding = 0) uniform sampler2D texture_input;

void main(){
#ifdef SINGLE_CHANNEL
    float r = texture(texture_input, v_uv).r;
    o_color = vec4(r, r, r, 1.0);
#else
    vec4 color = texture(texture_input, v_uv);
    o_color = vec4(color.rgb, 1.0);
#endif
}