#version 320 es
precision highp float;

layout(location = 0) in vec3 v_normal;
layout(location = 1) in vec2 v_uv;

layout(location = 0) out vec4 o_color;
layout(location = 1) out vec4 o_depth;

layout(set = 0, binding = 0) uniform GlobalUniform{
    mat4 viewProj;
    vec4 lightDirIntensity;
    vec3 lightColor;
};

layout(set = 0, binding = 1) uniform LocalUniform{
    mat4 model;
    mat4 invModel;
};

layout(set = 0, binding = 2) uniform sampler2D albedoTexture;

void main(){
    float ndotl = dot(normalize(v_normal), -lightDirIntensity.xyz);
    // pre-scatter texture or post-scatter texture?
    // vec3 diffuse = ndotl * texture(albedoTexture, v_uv).rgb / 3.14159265358979 * lightColor.rgb * lightDirIntensity.w;
    vec3 diffuse = ndotl / 3.14159265358979 * lightColor.rgb * lightDirIntensity.w * 0.5;
    o_color = vec4(diffuse, 1.0);
    o_depth = vec4(1.0 / gl_FragCoord.w, 0.0, 0.0, 0.0);
}