#version 320 es
precision highp float;

layout(location = 0) in vec3 v_posWS;
layout(location = 1) in vec3 v_normal;
layout(location = 2) in vec2 v_uv;

layout(location = 0) out vec4 o_color;

layout(set = 0, binding = 0) uniform GlobalUniform{
    mat4 model;
    mat4 viewProj;
    mat4 invModel;
    vec4 lightDirIntensity;
};

layout(set = 0, binding = 1) uniform sampler2D baseColorTexture;
layout(set = 0, binding = 2) uniform sampler2D normalTexture;
layout(set = 0, binding = 3) uniform sampler2D paramsTexture;

vec3 get_normal()
{
	vec3 pos_dx = dFdx(v_posWS);
	vec3 pos_dy = dFdy(v_posWS);
	vec3 st1    = dFdx(vec3(v_uv, 0.0));
	vec3 st2    = dFdy(vec3(v_uv, 0.0));
	vec3 T      = (st2.t * pos_dx - st1.t * pos_dy) / (st1.s * st2.t - st2.s * st1.t);
	vec3 N      = normalize(v_normal);
	T           = normalize(T - N * dot(N, T));
	vec3 B      = normalize(cross(N, T));
	mat3 TBN    = mat3(T, B, N);
    vec3 n = texture(normalTexture, v_uv).rgb;
	return normalize(TBN * (2.0 * n - 1.0));
}

float saturate(float x){
    return clamp(x, 0.0, 1.0);
}

void main(){
    const float pi = 3.14159265358979;
    vec3 normal = get_normal();
    float ndotl = saturate(dot(normal, lightDirIntensity.xyz));
    vec4 albedo = texture(baseColorTexture, v_uv);
    vec3 params = texture(paramsTexture, v_uv).xyz;
    float bakeAO = params.r;
    float intensity = lightDirIntensity.w;
    vec3 diffuse = albedo.rgb / pi * ndotl * bakeAO * intensity;
    o_color = vec4(diffuse, 1.0);
}