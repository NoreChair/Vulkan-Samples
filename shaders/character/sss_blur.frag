#version 320 es
precision highp float;
#extension GL_ARB_shader_image_load_store : enable

#ifdef GL_ARB_shader_image_load_store // force to eanble early stencil test, to avoid unnecessary blur https://www.khronos.org/opengl/wiki/Per-Sample_Processing#Pixel_ownership_test
layout(early_fragment_tests) in;
#endif

layout(set = 0, binding = 0) uniform SSS{
    vec2 stepSize; // stepSize = sssLevel * width * pixelSize
    float correction;
    float maxdd;
    float depth;
};

layout(set = 0, binding = 1) uniform sampler2D tex1;
layout(set = 0, binding = 2) uniform sampler2D depthTex;

layout(location = 0) in vec2 v_uv;

layout(location = 0) out vec4 o_color;

void main(){
    const float w[7] = float[7](
        0.006,
        0.061,
        0.242,
        0.382,
        0.242,
        0.061,
        0.006
    );

    vec4 color = texture(tex1, v_uv);
    color.rgb *= w[3];

    float depth = texture(depthTex, v_uv).r;
    vec2 s_x = stepSize * vec2(1.0, 0.0) / (depth + correction * min(abs(dFdx(depth)), maxdd));
    vec2 finalWidth = s_x;
    float weightScale = 0.0;

    vec2 offset = v_uv - finalWidth;
    for (int i = 0; i < 3; i++) {
        vec3 diffuse = texture(tex1, offset).rgb;
        weightScale += step(1e-5, diffuse.r) * w[i];
        color.rgb += w[i] * diffuse;
        offset += finalWidth / 3.0;
    }
    offset += finalWidth / 3.0;

    for (int i = 4; i < 7; i++) {
        vec3 diffuse = texture(tex1, offset).rgb;
        weightScale += step(1e-5, diffuse.r) * w[i];
        color.rgb += w[i] * diffuse;
        offset += finalWidth / 3.0;
    }

    color *= 1.0 / (weightScale + 0.382);

    o_color = color;
}