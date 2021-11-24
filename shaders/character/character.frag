#version 320 es
precision highp float;

layout(location = 0) in vec3 v_posWS;
layout(location = 1) in vec3 v_normal;
layout(location = 2) in vec2 v_uv;
layout(location = 3) in vec2 v_uvNDC;

layout(location = 0) out vec4 o_color;

layout(set = 0, binding = 0) uniform GlobalUniform{
    mat4 viewProj;
    mat4 shadowVP;
    vec4 lightDirIntensity;
    vec4 lightColor;
    vec4 cameraPosWS;
    vec4 shadowExtent;
    float roughness;
    float metalness;
    uint useColorBleedAO;
    uint useDoubleSpecular;
    uint useSSS;
    uint onlySSS;
    uint onlyShadow;
};

layout(set = 0, binding = 1) uniform LocalUniform{
    mat4 model;
    mat4 invModel;
};

layout(set = 0, binding = 2) uniform sampler2D baseColorTexture;
layout(set = 0, binding = 3) uniform sampler2D normalTexture;
layout(set = 0, binding = 4) uniform sampler2D paramsTexture;
layout(set = 0, binding = 5) uniform sampler2D detailTexture;
layout(set = 0, binding = 6) uniform sampler2D sssTexture;
layout(set = 0, binding = 7) uniform highp sampler2DShadow shadowTexture;
layout(set = 0, binding = 8) uniform samplerCube irradianceTexture;
layout(set = 0, binding = 9) uniform samplerCube radianceTexture;

float saturate(float x){
    return clamp(x, 0.0, 1.0);
}

vec2 saturate(vec2 x){
    return clamp(x, vec2(0.0), vec2(1.0));
}

vec3 saturate(vec3 x){
    return clamp(x, vec3(0.0), vec3(1.0));
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

/*----------------------------Binn-Phong----------------------------*/

vec2 binn_phong_ndf(vec2 alpha, float ndoth){
    return (alpha + 2.0) * 0.125 * pow(vec2(ndoth, ndoth), alpha);
}

vec2 schlick_smith_approximate_visibility(vec2 alpha, float ndotl, float ndotv){
    vec2 k = 2.0 * inversesqrt(3.14159265358979 * (alpha + 2.0));
    vec2 oneMinus = 1.0 - k;
    return 1.0 / ((ndotl * oneMinus + k) * (ndotv * oneMinus + k));
}

vec2 schlick_smith_opt_visibility(vec2 gloss, float vdoth){
    vec2 k = min(vec2(1.0), gloss + vec2(0.545));
    return 1.0 / (k * vdoth * vdoth + (vec2(1.0) - k));
}

// from [Getting More Physical in Call of Duty: Black Ops II]
// gloss range (0.0 ~ 1.0), map alpha to 8192
// Dpl(h) = pi/4 * D(h)
float binn_phong_ndf(float alpha, float ndoth){
    return (alpha + 2.0) * 0.125 * pow(ndoth, alpha);
}

float schlick_smith_approximate_visibility(float alpha, float ndotl, float ndotv){
    float k = 2.0 * inversesqrt(3.14159265358979 * (alpha + 2.0));
    float oneMinus = 1.0 - k;
    return 1.0 / ((ndotl * oneMinus + k) * (ndotv * oneMinus + k));
}

float schlick_smith_opt_visibility(float gloss, float vdoth){
    float k = min(1.0, gloss + 0.545);
    return 1.0 / (k * vdoth * vdoth + (1.0 - k));
}

vec3 blinn_phong_specular_brdf(float ndotl, float ndotv, float ndoth, float hdotl, float vdoth, float gloss, vec3 specularColor){
    vec3 directSpecular = vec3(0.0, 0.0, 0.0);

    if(useDoubleSpecular > 0u){
        float secondGloss = saturate(pow(2.0, 14.0 * gloss) / 255.0);
        secondGloss = secondGloss * secondGloss;
        vec2 alpha = pow(vec2(8192.0, 8192.0), vec2(gloss, secondGloss));

        // Dpl(h) * F(l,h) * V(l,v,h)
        // vec2 sigma = binn_phong_ndf(alpha, ndoth) * schlick_smith_approximate_visibility(alpha, ndotl, ndotv);
        vec2 sigma = binn_phong_ndf(alpha, ndoth)  * schlick_smith_opt_visibility(vec2(gloss, secondGloss), vdoth);
        vec3 fr = schlick_approximate_fresnel(hdotl, specularColor, vec3(1.0));
        directSpecular = mix(sigma.x * fr, sigma.y * fr, 0.15);
    }else{
        float alpha = pow(8192.0, gloss);
        directSpecular = binn_phong_ndf(alpha, ndoth) * schlick_smith_approximate_visibility(alpha, ndotl, ndotv) * schlick_fresnel(hdotl, specularColor, vec3(1.0));
    }

    return saturate(directSpecular);
}

/*----------------------------GGX----------------------------*/
// RTR4 - equation 9.43
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

vec2 smith_ggx_height_correlated_hammon_approximate_visibility(float ndotv, float ndotl, vec2 roughness){
    float a = 2.0 * ndotl * ndotv;
    float b = ndotl + ndotv;
    vec2 GGX = mix(vec2(a), vec2(b), roughness);
    vec2 s = step(vec2(1e-5), GGX) * vec2(0.5);
    return s / max(GGX, vec2(1e-5));
}

// RTR4 - equation 9.41
float ggx_ndf(float ndoth, float roughness)
{
	float alphaRoughnessSq = roughness * roughness;
	float f = (ndoth * alphaRoughnessSq - ndoth) * ndoth + 1.0;
	return alphaRoughnessSq / (3.14159265358979 * f * f);
}

vec2 ggx_ndf(float ndoth, vec2 roughness)
{
	vec2 alphaRoughnessSq = roughness * roughness;
	vec2 f = (ndoth * alphaRoughnessSq - ndoth) * ndoth + 1.0;
	return alphaRoughnessSq / (3.14159265358979 * f * f);
}

vec3 ggx_specular_brdf(float ndotl, float ndotv, float ndoth, float hdotl, float vdoth, float roughness, vec3 specularColor){
    vec3 directSpecular = vec3(0.0, 0.0, 0.0);

    vec3 f = schlick_fresnel(hdotl, specularColor, vec3(0.2));
    if(useDoubleSpecular > 0u){
        float secondRoughness = saturate(log2(roughness) / 8.0 + 1.0);
        vec2 vis = smith_ggx_height_correlated_hammon_approximate_visibility(ndotv, ndotl, vec2(roughness, secondRoughness));
        vec2 d   = ggx_ndf(ndoth, vec2(roughness, secondRoughness));
        return mix(f * d.x * vis.x, f * d.y * vis.y, 0.15);
    }else{
        float vis = smith_ggx_height_correlated_hammon_approximate_visibility(ndotv, ndotl, roughness);
        float d   = ggx_ndf(ndoth, roughness);
        return f * d * vis;
    }
}

/*----------------------------Shadow----------------------------*/
// #define PLANE_BIAS

vec2 plane_shadow_bias(vec3 shadowPos){
    // for large kernel, is better to use
    // (delta u, delta v) = (delta x) * (ddx u, ddx v) + (delta y) * (ddy u, ddy v);
    // (delta z) = (delta x) * (ddx z) + (delta y) * (ddy z);
    // dest ref z = origin ref z + delta z;
    // to determine shadow pcfs
    vec2 dx = dFdx(shadowPos.xy);
    vec2 dy = dFdy(shadowPos.xy);
    mat2 correctMatrix = mat2(dy.y, -dy.x, -dx.y, dx.x) * (1.0 / (dx.x * dy.y - dx.y * dy.x));
    vec2 dZ = vec2(dFdx(shadowPos.z), dFdy(shadowPos.z));
    vec2 correctOffset = correctMatrix * dZ;
    return correctOffset;
}

float regular_grid_sample(vec3 shadowPos, vec2 rcpShadowExtent){
    const float k_pcfWeight[9] = float[9](
        0.5,1.0,0.5,
        1.0,1.0,1.0,
        0.5,1.0,0.5
    );

    // regular grid 3 x 3 sampling
    vec2 kernel[9] = vec2[9](
        rcpShadowExtent * vec2(-1.0), rcpShadowExtent * vec2(0.0, -1.0), rcpShadowExtent * vec2(1.0, -1.0),
        rcpShadowExtent * vec2(-1.0, 0.0), rcpShadowExtent * vec2(0.0), rcpShadowExtent * vec2(1.0, 0.0),
        rcpShadowExtent * vec2(-1.0, 1.0), rcpShadowExtent * vec2(0.0, 1.0), rcpShadowExtent * vec2(1.0)
    );

    float shadowValue = 0.0;

#ifdef PLANE_BIAS
    vec2 shadowDepthBais = abs(plane_shadow_bias(shadowPos)) * rcpShadowExtent;
    shadowValue += texture(shadowTexture, vec3(shadowPos.xy + kernel[0], shadowPos.z + dot(shadowDepthBais, kernel[0]))) * k_pcfWeight[0];
    shadowValue += texture(shadowTexture, vec3(shadowPos.xy + kernel[1], shadowPos.z + dot(shadowDepthBais, kernel[1]))) * k_pcfWeight[1];
    shadowValue += texture(shadowTexture, vec3(shadowPos.xy + kernel[2], shadowPos.z + dot(shadowDepthBais, kernel[2]))) * k_pcfWeight[2];
    shadowValue += texture(shadowTexture, vec3(shadowPos.xy + kernel[3], shadowPos.z + dot(shadowDepthBais, kernel[3]))) * k_pcfWeight[3];
    shadowValue += texture(shadowTexture, vec3(shadowPos.xy + kernel[4], shadowPos.z + dot(shadowDepthBais, kernel[4]))) * k_pcfWeight[4];
    shadowValue += texture(shadowTexture, vec3(shadowPos.xy + kernel[5], shadowPos.z + dot(shadowDepthBais, kernel[5]))) * k_pcfWeight[5];
    shadowValue += texture(shadowTexture, vec3(shadowPos.xy + kernel[6], shadowPos.z + dot(shadowDepthBais, kernel[6]))) * k_pcfWeight[6];
    shadowValue += texture(shadowTexture, vec3(shadowPos.xy + kernel[7], shadowPos.z + dot(shadowDepthBais, kernel[7]))) * k_pcfWeight[7];
    shadowValue += texture(shadowTexture, vec3(shadowPos.xy + kernel[8], shadowPos.z + dot(shadowDepthBais, kernel[8]))) * k_pcfWeight[8];
#else
    shadowValue += texture(shadowTexture, vec3(shadowPos.xy + kernel[0], shadowPos.z)) * k_pcfWeight[0];
    shadowValue += texture(shadowTexture, vec3(shadowPos.xy + kernel[1], shadowPos.z)) * k_pcfWeight[1];
    shadowValue += texture(shadowTexture, vec3(shadowPos.xy + kernel[2], shadowPos.z)) * k_pcfWeight[2];
    shadowValue += texture(shadowTexture, vec3(shadowPos.xy + kernel[3], shadowPos.z)) * k_pcfWeight[3];
    shadowValue += texture(shadowTexture, vec3(shadowPos.xy + kernel[4], shadowPos.z)) * k_pcfWeight[4];
    shadowValue += texture(shadowTexture, vec3(shadowPos.xy + kernel[5], shadowPos.z)) * k_pcfWeight[5];
    shadowValue += texture(shadowTexture, vec3(shadowPos.xy + kernel[6], shadowPos.z)) * k_pcfWeight[6];
    shadowValue += texture(shadowTexture, vec3(shadowPos.xy + kernel[7], shadowPos.z)) * k_pcfWeight[7];
    shadowValue += texture(shadowTexture, vec3(shadowPos.xy + kernel[8], shadowPos.z)) * k_pcfWeight[8];
#endif
    return shadowValue * 0.14285714;
}

float poisson_sample(vec3 shadowPos, vec2 rcpShadowExtent){

    // For Poisson Disk PCF sampling
    const vec2 k_poissonSamples[64] = vec2[64]
    (
        vec2(-0.5119625f, -0.4827938f),
        vec2(-0.2171264f, -0.4768726f),
        vec2(-0.7552931f, -0.2426507f),
        vec2(-0.7136765f, -0.4496614f),
        vec2(-0.5938849f, -0.6895654f),
        vec2(-0.3148003f, -0.7047654f),
        vec2(-0.42215f, -0.2024607f),
        vec2(-0.9466816f, -0.2014508f),
        vec2(-0.8409063f, -0.03465778f),
        vec2(-0.6517572f, -0.07476326f),
        vec2(-0.1041822f, -0.02521214f),
        vec2(-0.3042712f, -0.02195431f),
        vec2(-0.5082307f, 0.1079806f),
        vec2(-0.08429877f, -0.2316298f),
        vec2(-0.9879128f, 0.1113683f),
        vec2(-0.3859636f, 0.3363545f),
        vec2(-0.1925334f, 0.1787288f),
        vec2(0.003256182f, 0.138135f),
        vec2(-0.8706837f, 0.3010679f),
        vec2(-0.6982038f, 0.1904326f),
        vec2(0.1975043f, 0.2221317f),
        vec2(0.1507788f, 0.4204168f),
        vec2(0.3514056f, 0.09865579f),
        vec2(0.1558783f, -0.08460935f),
        vec2(-0.0684978f, 0.4461993f),
        vec2(0.3780522f, 0.3478679f),
        vec2(0.3956799f, -0.1469177f),
        vec2(0.5838975f, 0.1054943f),
        vec2(0.6155105f, 0.3245716f),
        vec2(0.3928624f, -0.4417621f),
        vec2(0.1749884f, -0.4202175f),
        vec2(0.6813727f, -0.2424808f),
        vec2(-0.6707711f, 0.4912741f),
        vec2(0.0005130528f, -0.8058334f),
        vec2(0.02703013f, -0.6010728f),
        vec2(-0.1658188f, -0.9695674f),
        vec2(0.4060591f, -0.7100726f),
        vec2(0.7713396f, -0.4713659f),
        vec2(0.573212f, -0.51544f),
        vec2(-0.3448896f, -0.9046497f),
        vec2(0.1268544f, -0.9874692f),
        vec2(0.7418533f, -0.6667366f),
        vec2(0.3492522f, 0.5924662f),
        vec2(0.5679897f, 0.5343465f),
        vec2(0.5663417f, 0.7708698f),
        vec2(0.7375497f, 0.6691415f),
        vec2(0.2271994f, -0.6163502f),
        vec2(0.2312844f, 0.8725659f),
        vec2(0.4216993f, 0.9002838f),
        vec2(0.4262091f, -0.9013284f),
        vec2(0.2001408f, -0.808381f),
        vec2(0.149394f, 0.6650763f),
        vec2(-0.09640376f, 0.9843736f),
        vec2(0.7682328f, -0.07273844f),
        vec2(0.04146584f, 0.8313184f),
        vec2(0.9705266f, -0.1143304f),
        vec2(0.9670017f, 0.1293385f),
        vec2(0.9015037f, -0.3306949f),
        vec2(-0.5085648f, 0.7534177f),
        vec2(0.9055501f, 0.3758393f),
        vec2(0.7599946f, 0.1809109f),
        vec2(-0.2483695f, 0.7942952f),
        vec2(-0.4241052f, 0.5581087f),
        vec2(-0.1020106f, 0.6724468f)
    );

    float shadowValue = 0.0;

    // TODO : random rotate
#ifdef PLANE_BIAS
    vec2 shadowDepthBais = abs(plane_shadow_bias(shadowPos)) * rcpShadowExtent;
    shadowValue += texture(shadowTexture, vec3(k_poissonSamples[0] * rcpShadowExtent.xy + shadowPos.xy, shadowPos.z + dot(k_poissonSamples[0] * rcpShadowExtent.xy, shadowDepthBais)));
    shadowValue += texture(shadowTexture, vec3(k_poissonSamples[2] * rcpShadowExtent.xy + shadowPos.xy, shadowPos.z + dot(k_poissonSamples[2] * rcpShadowExtent.xy, shadowDepthBais)));
    shadowValue += texture(shadowTexture, vec3(k_poissonSamples[1] * rcpShadowExtent.xy + shadowPos.xy, shadowPos.z + dot(k_poissonSamples[1] * rcpShadowExtent.xy, shadowDepthBais)));
    shadowValue += texture(shadowTexture, vec3(k_poissonSamples[3] * rcpShadowExtent.xy + shadowPos.xy, shadowPos.z + dot(k_poissonSamples[3] * rcpShadowExtent.xy, shadowDepthBais)));
    shadowValue += texture(shadowTexture, vec3(k_poissonSamples[4] * rcpShadowExtent.xy + shadowPos.xy, shadowPos.z + dot(k_poissonSamples[4] * rcpShadowExtent.xy, shadowDepthBais)));
    shadowValue += texture(shadowTexture, vec3(k_poissonSamples[5] * rcpShadowExtent.xy + shadowPos.xy, shadowPos.z + dot(k_poissonSamples[5] * rcpShadowExtent.xy, shadowDepthBais)));
    shadowValue += texture(shadowTexture, vec3(k_poissonSamples[6] * rcpShadowExtent.xy + shadowPos.xy, shadowPos.z + dot(k_poissonSamples[6] * rcpShadowExtent.xy, shadowDepthBais)));
    shadowValue += texture(shadowTexture, vec3(k_poissonSamples[7] * rcpShadowExtent.xy + shadowPos.xy, shadowPos.z + dot(k_poissonSamples[7] * rcpShadowExtent.xy, shadowDepthBais)));
    shadowValue += texture(shadowTexture, vec3(k_poissonSamples[8] * rcpShadowExtent.xy + shadowPos.xy, shadowPos.z + dot(k_poissonSamples[8] * rcpShadowExtent.xy, shadowDepthBais)));
#else
    shadowValue += texture(shadowTexture, vec3(k_poissonSamples[0] * rcpShadowExtent.xy + shadowPos.xy, shadowPos.z));
    shadowValue += texture(shadowTexture, vec3(k_poissonSamples[2] * rcpShadowExtent.xy + shadowPos.xy, shadowPos.z));
    shadowValue += texture(shadowTexture, vec3(k_poissonSamples[1] * rcpShadowExtent.xy + shadowPos.xy, shadowPos.z));
    shadowValue += texture(shadowTexture, vec3(k_poissonSamples[3] * rcpShadowExtent.xy + shadowPos.xy, shadowPos.z));
    shadowValue += texture(shadowTexture, vec3(k_poissonSamples[4] * rcpShadowExtent.xy + shadowPos.xy, shadowPos.z));
    shadowValue += texture(shadowTexture, vec3(k_poissonSamples[5] * rcpShadowExtent.xy + shadowPos.xy, shadowPos.z));
    shadowValue += texture(shadowTexture, vec3(k_poissonSamples[6] * rcpShadowExtent.xy + shadowPos.xy, shadowPos.z));
    shadowValue += texture(shadowTexture, vec3(k_poissonSamples[7] * rcpShadowExtent.xy + shadowPos.xy, shadowPos.z));
    shadowValue += texture(shadowTexture, vec3(k_poissonSamples[8] * rcpShadowExtent.xy + shadowPos.xy, shadowPos.z));
#endif
    return shadowValue * 0.111111111111111;
}

float optimiazed_sample(vec3 shadowPos, vec2 rcpShadowExtent){
    vec2 uv = shadowPos.xy * shadowExtent.xy;
    vec2 baseUV = floor(uv + 0.5);

    vec2 st = (uv + 0.5 - baseUV);
    baseUV -= vec2(0.5);
    baseUV *= rcpShadowExtent;

    float shadowValue = 0.0;
    vec2 uvw0 = vec2(-2.0) * st + vec2(3.0);
    vec2 uvw1 = vec2(1.0) + vec2(2.0) * st;
    vec2 uv0 = (vec2(2.0) - st) / uvw0 - vec2(1.0);
    vec2 uv1 = st / uvw1 + vec2(1.0);

#ifdef PLANE_BIAS
    vec2 shadowDepthBais = abs(plane_shadow_bias(shadowPos)) * rcpShadowExtent;
    shadowValue += uvw0.x * uvw0.y * texture(shadowTexture, vec3(baseUV + uv0 * rcpShadowExtent, shadowPos.z + dot(uv0 * rcpShadowExtent, shadowDepthBais)));
    shadowValue += uvw1.x * uvw0.y * texture(shadowTexture, vec3(baseUV + vec2(uv1.x, uv0.y) * rcpShadowExtent, shadowPos.z + dot(vec2(uv1.x, uv0.y) * rcpShadowExtent, shadowDepthBais)));
    shadowValue += uvw0.x * uvw1.y * texture(shadowTexture, vec3(baseUV + vec2(uv0.x, uv1.y) * rcpShadowExtent, shadowPos.z + dot(vec2(uv0.x, uv1.y) * rcpShadowExtent, shadowDepthBais)));
    shadowValue += uvw1.x * uvw1.y * texture(shadowTexture, vec3(baseUV + uv1 * rcpShadowExtent, shadowPos.z + dot(uv1 * rcpShadowExtent, shadowDepthBais)));
#else
    shadowValue += uvw0.x * uvw0.y * texture(shadowTexture, vec3(baseUV + uv0 * rcpShadowExtent, shadowPos.z));
    shadowValue += uvw1.x * uvw0.y * texture(shadowTexture, vec3(baseUV + vec2(uv1.x, uv0.y) * rcpShadowExtent, shadowPos.z));
    shadowValue += uvw0.x * uvw1.y * texture(shadowTexture, vec3(baseUV + vec2(uv0.x, uv1.y) * rcpShadowExtent, shadowPos.z));
    shadowValue += uvw1.x * uvw1.y * texture(shadowTexture, vec3(baseUV + uv1 * rcpShadowExtent, shadowPos.z));
#endif
    return shadowValue / 16.0;
}

float percentage_closer_filtering(vec3 shadowPos){
    float pcf = 0.0;
    const float kernelWidth = 1.0f;
    // TODO : use poisson disk sampling to reduce shadow banding
    // pcf = regular_grid_sample(shadowPos, shadowExtent.zw * kernelWidth);
    pcf = optimiazed_sample(shadowPos, shadowExtent.zw * kernelWidth);
    return pcf;
}

vec3 reoriented_normal_mapping(vec3 base, vec3 detail){
    vec3 t = base * vec3(2.0, 2.0, 2.0) + vec3(-1.0, -1.0, 0.0);
    vec3 u = detail * vec3(-2.0, -2.0, 2.0) + vec3(1.0, 1.0, -1.0);
    vec3 r = normalize(t * dot(t,u) / t.z - u);
    return r;
}

float to_luminance(vec3 color){
    return dot(vec3(0.2126, 0.7152, 0.0722), color);
}

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
    vec3 d = texture(detailTexture, v_uv * 10.0).rgb;
    return TBN * reoriented_normal_mapping(n, d);
	// return normalize(TBN * (2.0 * n - 1.0));
}

void main(){
    const float k_pi = 3.14159265358979;
    vec3 normal = get_normal();
    vec3 view = normalize(cameraPosWS.xyz - v_posWS.xyz);
    vec3 light = -normalize(lightDirIntensity.xyz);
    vec3 halfVector = normalize(view + light);

    float ndotl = saturate(dot(normal, light));
    float ndotv = saturate(dot(normal, view));
    float ndoth = saturate(dot(normal, halfVector));
    float hdotl = saturate(dot(halfVector, light));
    float vdoth = saturate(dot(view, halfVector));

    if(onlySSS > 0u){
        o_color = texture(sssTexture, v_uvNDC);
        return;
    }

    vec4 albedo = texture(baseColorTexture, v_uv);
    vec3 params = texture(paramsTexture, v_uv).xyz;
    float intensity = lightDirIntensity.w;
    float cavity = params.b;

    // pore
    vec3 pore = schlick_fresnel(ndotv , vec3(cavity), vec3(1.0));

    // direct light
    vec3 diffuse = albedo.rgb / k_pi * intensity * ndotl;
    vec3 surfaceColor = albedo.rgb; // instread of metal f0
    vec3 f0 = mix(vec3(0.028), surfaceColor, metalness);
    if(useSSS > 0u){
        // // hack
        // float lum = to_luminance(albedo.rgb);
        // float sssFactor = smoothstep(0.0, 0.07, lum);
        // // hack end
        // diffuse = mix(diffuse, texture(sssTexture, v_uvNDC).rgb , vec3(sssFactor));
        diffuse = texture(sssTexture, v_uvNDC).rgb * albedo.rgb * 2.0;
    }
    // approximate : diffuse = diffuse * (vec3(1.0) - f0) * (1.0 - metalness);
    diffuse = diffuse * (vec3(1.0) - schlick_fresnel(hdotl, f0, vec3(0.2))) * (1.0 - metalness);

    // vec3 specular = blinn_phong_specular_brdf(ndotl, ndotv, ndoth, hdotl, vdoth, (1.0 - roughness) * glossScale, f0);
    vec3 specular = ggx_specular_brdf(ndotl, ndotv, ndoth, hdotl, vdoth, roughness, f0) * pore * ndotl * intensity;

    // direct shadow
    vec4 positionSS = shadowVP * vec4(v_posWS, 1.0);
    positionSS /= positionSS.w;
    positionSS.xy = positionSS.xy * vec2(0.5) + vec2(0.5);
    float shadow = percentage_closer_filtering(positionSS.xyz);
    // float shadow = texture(shadowTexture, positionSS.xyz);

    if(onlyShadow > 0u){
        o_color = vec4(shadow);
        return;
    }

    vec3 directShading = (diffuse + specular) * lightColor.rgb  * shadow ;

    // indirect light
    float ao = params.r; // TDOO : min(bakeAO, SSAO);
	vec3 irradiance = texture(irradianceTexture, normal).rgb;
	vec3 iblDiffuse = vec3(0.0);

    if(useColorBleedAO > 0u){
        const vec3 bleedAOStrength = vec3(0.4, 0.15, 0.13);
        iblDiffuse = albedo.rgb * pow(irradiance * ao, 1.0 - bleedAOStrength) * lightColor.rgb;
    }else{
        iblDiffuse = albedo.rgb * ao * irradiance * lightColor.rgb;
    }

    const float k_specularPow = 8.0;
    float s = saturate(-0.3 + ndotv * ndotv);
    float specularOcclusion = mix(pow(ao, k_specularPow), 1.0, s);
    vec3 r = -reflect(view, normal);
    vec3 radiance = texture(radianceTexture, r, roughness * 6.0).rgb; // 256 radiance
    vec3 iblSpecular = schlick_fresnel(hdotl, f0, vec3(0.2)) * radiance;
    vec3 indirectShading = iblDiffuse * (1.0 - metalness) + iblSpecular * pore * specularOcclusion;

    vec3 finalShading = directShading + indirectShading;
    o_color = vec4(finalShading, 1.0);
}