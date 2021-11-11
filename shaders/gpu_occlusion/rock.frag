#version 320 es
precision highp float;

layout(location = 0) in vec3 v_pos;
layout(location = 1) in vec2 v_uv;
layout(location = 2) in vec3 v_normal;

layout(location = 0) out vec4 o_color;

layout(set = 0, binding = 0) uniform GlobalUniform
{
	mat4 _viewProj;
    vec4 _lightDirIntensity;
    vec4 _lightColor;
    vec4 _cameraPos;
};

layout(set = 0, binding = 1) uniform LocalUniform{
    mat4 _model;
    mat4 _invModel;
    vec4 _baseColor;
};

layout(set = 0, binding = 3) uniform sampler2D base_color_texture;
layout(set = 0, binding = 4) uniform sampler2D normal_texture;
layout(set = 0, binding = 5) uniform sampler2D occlusion_texture;

float saturate(float x){
    return clamp(x, 0.0, 1.0);
}

vec2 saturate(vec2 x){
    return clamp(x, vec2(0.0), vec2(1.0));
}

vec3 saturate(vec3 x){
    return clamp(x, vec3(0.0), vec3(1.0));
}

vec4 saturate(vec4 x){
    return clamp(x, vec4(0.0), vec4(1.0));
}

/*----------------------------Fresnel----------------------------*/

vec3 schlick_fresnel(float cosT, vec3 f0, vec3 f90){
    float beta = 1.0 - cosT;
    float beta2 = beta * beta;
    float beta4 = beta2 * beta2;
    return f0 + (f90 - f0) * (beta4 * beta);
}

vec3 schlick_approximate_fresnel(float cosT, vec3 f0, vec3 f90){
    return f0 + (f90 - f0) * pow(2.0, -10.0 * cosT);
}

/*----------------------------GGX----------------------------*/
float smith_ggx_height_correlated_visibility(float ndotv, float ndotl, float roughness)
{
	float alphaRoughnessSq = roughness * roughness;

	float GGXV = ndotl * sqrt(ndotv * ndotv * (1.0 - alphaRoughnessSq) + alphaRoughnessSq);
	float GGXL = ndotv * sqrt(ndotl * ndotl * (1.0 - alphaRoughnessSq) + alphaRoughnessSq);

	float GGX = GGXV + GGXL;
	if (GGX > 0.0)
	{
		return 0.5 / GGX;
	}
	return 0.0;
}

float smith_ggx_height_correlated_hammon_approximate_visibility(float ndotv, float ndotl, float roughness){
    float a = 2.0 * ndotl * ndotv;
    float b = ndotl + ndotv;
    float GGX = mix(a, b, roughness);
    if(GGX > 0.0 ){
        return 0.5 / GGX;
    }
    return 0.0;
}

float ggx_ndf(float ndoth, float roughness)
{
	float alphaRoughnessSq = roughness * roughness;
	float f = (ndoth * alphaRoughnessSq - ndoth) * ndoth + 1.0;
	return alphaRoughnessSq / (3.14159265358979 * f * f);
}

vec3 ggx_specular_brdf(float ndotl, float ndotv, float ndoth, float hdotl, float vdoth, float roughness, vec3 specularColor){
    vec3 directSpecular = vec3(0.0, 0.0, 0.0);

    vec3 f = schlick_fresnel(hdotl, specularColor, vec3(1.0));
    float vis = smith_ggx_height_correlated_hammon_approximate_visibility(ndotv, ndotl, roughness * roughness);
    float d   = ggx_ndf(ndoth, roughness * roughness);
    return f * d * vis;
}

vec3 get_normal(vec3 n)
{
	vec3 pos_dx = dFdx(v_pos);
	vec3 pos_dy = dFdy(v_pos);
	vec3 st1    = dFdx(vec3(v_uv, 0.0));
	vec3 st2    = dFdy(vec3(v_uv, 0.0));
	vec3 T      = (st2.t * pos_dx - st1.t * pos_dy) / (st1.s * st2.t - st2.s * st1.t);
	vec3 N      = normalize(v_normal);
	T           = normalize(T - N * dot(N, T));
	vec3 B      = normalize(cross(N, T));
	mat3 TBN    = mat3(T, B, N);
	return normalize(TBN * (2.0 * n - 1.0));
}

void main(){
    const float k_pi = 3.14159265358979;
    vec3 view = normalize(_cameraPos.xyz - v_pos.xyz);
    vec3 light = -normalize(_lightDirIntensity.xyz);
    vec3 halfVector = normalize(view + light);
    vec3 normal = get_normal(texture(normal_texture, v_uv).rgb);

    float ndotl = saturate(dot(normal, light));
    float ndotv = saturate(dot(normal, view));
    float ndoth = saturate(dot(normal, halfVector));
    float hdotl = saturate(dot(halfVector, light));
    float vdoth = saturate(dot(view, halfVector));

    vec4 albedo = texture(base_color_texture, v_uv);
    vec3 baseColor = albedo.rgb * _baseColor.rgb;
    float roughness = (1.0 - albedo.a);
    roughness *= roughness;
    float metalness = 0.3;
    float ao = texture(occlusion_texture, v_uv).r;

    vec3 diffuse = baseColor.rgb / k_pi;
    vec3 surfaceColor = baseColor.rgb; // instread of metal f0
    vec3 f0 = mix(vec3(0.04), surfaceColor, metalness);
    diffuse = diffuse * (vec3(1.0) - schlick_fresnel(hdotl, f0, vec3(1.0))) * (1.0 - metalness);
    vec3 specular = ggx_specular_brdf(ndotl, ndotv, ndoth, hdotl, vdoth, roughness, f0);
    vec3 direct = (specular + diffuse) * ndotl * _lightDirIntensity.w;

    vec3 indirect = vec3(0.1) * baseColor * ao;
    o_color =  vec4(direct + indirect, 1.0);
    // o_color = albedo;
}