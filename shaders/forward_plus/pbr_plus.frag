#version 320 es
precision highp float;

#define MAX_LIGHTS 128
#define TILE_SIZE (4 + MAX_LIGHTS)              // align with 128 bit
#define LIGHT_ATT_CUTOFF 1e-2

layout(location = 0) in vec3 v_pos;     // world position
layout(location = 1) in vec2 v_uv;
layout(location = 2) in vec3 v_normal;

layout(location = 0) out vec4 o_color;

layout(set = 0, binding = 0) uniform GlobalUniform
{
	mat4  model;
	mat4  view_proj;
	vec3  camera_position;
};

layout(set = 0, binding = 1) uniform LightInfo
{
	vec4  direction_light;
    vec4  direction_light_color;
	uvec2 viewPort;
	float inv_tile_dim;
    uint  tile_count_x;
};

// for performance reason, data best align with 128-bit
// bit covert to LightBufferData
struct LightBufferDataByte{
    uint value[16];
};

layout(set = 0, std430, binding = 2) readonly buffer LightBuffer
{
    LightBufferDataByte lightBufferData[];
};

layout(set = 0, std430, binding = 3) readonly buffer LightGrid
{
    uint lightGridData[];
};

layout(set = 0, std430, binding = 4) readonly buffer LightGridBitMask
{
    uint lightGridBitMaskData[];
};

layout(push_constant, std430) uniform MaterialUniform
{
	vec4  base_color_factor;
	float metallic_factor;
	float roughness_factor;
};

#ifdef HAS_BASE_COLOR_TEXTURE
layout(set = 1, binding = 0) uniform sampler2D base_color_texture;
#endif

#ifdef HAS_NORMAL_TEXTURE
layout(set = 1, binding = 1) uniform sampler2D normal_texture;
#endif

#ifdef HAS_METALLIC_ROUGHNESS_TEXTURE
layout(set = 1, binding = 2) uniform sampler2D metallic_roughness_texture;
#endif

layout(set = 1, binding = 3) uniform sampler2D main_light_shadow_texture;

struct LightBufferData
{
    vec3 position;              // xyz=pos if light source is sphere
    vec3 color;                 // xyz=rgb
    vec3 coneDir;               // xyz=direction if light source is spot
    vec3 coneAngles;            // x=1.0f/(cos(inner)-cos(outer)), y=cos(inner), z=cos(outer/2)
    float radius;
    float intensity;
    uint lightType;
};

LightBufferData ConvertLightBufferByte(uint byteData[16])
{
    LightBufferData bufferData;
    bufferData.position = vec3(uintBitsToFloat(byteData[0]), uintBitsToFloat(byteData[1]), uintBitsToFloat(byteData[2]));
    bufferData.color = vec3(uintBitsToFloat(byteData[3]), uintBitsToFloat(byteData[4]), uintBitsToFloat(byteData[5]));
    bufferData.coneDir = vec3(uintBitsToFloat(byteData[6]), uintBitsToFloat(byteData[7]), uintBitsToFloat(byteData[8]));
    bufferData.coneAngles = vec3(uintBitsToFloat(byteData[9]), uintBitsToFloat(byteData[10]), uintBitsToFloat(byteData[11]));
    bufferData.radius = uintBitsToFloat(byteData[12]);
    bufferData.intensity = uintBitsToFloat(byteData[13]);
    bufferData.lightType = byteData[14];
    return bufferData;
}

uvec2 GetTilePos(vec2 pos, vec2 invTileDim)
{
    return uvec2(floor(pos * invTileDim));
}

uint GetTileIndex(uvec2 tilePos, uint tileCountX)
{
    return tilePos.y * tileCountX + tilePos.x;
}

uint GetTileOffset(uint tileIndex)
{
    return tileIndex * uint(TILE_SIZE);
}

const float PI = 3.14159265359;

// [0] Frensel Schlick
vec3 F_Schlick(vec3 f0, float f90, float u)
{
	return f0 + (f90 - f0) * pow(1.0 - u, 5.0);
}

// [1] IBL Defuse Irradiance
vec3 F_Schlick_Roughness(vec3 f0, float cos_theta, float roughness)
{
	return f0 + (max(vec3(1.0 - roughness), f0) - f0) * pow(1.0 - cos_theta, 5.0);
}

// [0] Diffuse Term
float Fr_DisneyDiffuse(float NdotV, float NdotL, float LdotH, float roughness)
{
	float E_bias        = 0.0 * (1.0 - roughness) + 0.5 * roughness;
	float E_factor      = 1.0 * (1.0 - roughness) + (1.0 / 1.51) * roughness;
	float fd90          = E_bias + 2.0 * LdotH * LdotH * roughness;
	vec3  f0            = vec3(1.0);
	float light_scatter = F_Schlick(f0, fd90, NdotL).r;
	float view_scatter  = F_Schlick(f0, fd90, NdotV).r;
	return light_scatter * view_scatter * E_factor;
}

// [0] Specular Microfacet Model
float V_SmithGGXCorrelated(float NdotV, float NdotL, float roughness)
{
	float alphaRoughnessSq = roughness * roughness;

	float GGXV = NdotL * sqrt(NdotV * NdotV * (1.0 - alphaRoughnessSq) + alphaRoughnessSq);
	float GGXL = NdotV * sqrt(NdotL * NdotL * (1.0 - alphaRoughnessSq) + alphaRoughnessSq);

	float GGX = GGXV + GGXL;
	if (GGX > 0.0)
	{
		return 0.5 / GGX;
	}
	return 0.0;
}

// [0] GGX Normal Distribution Function
float D_GGX(float NdotH, float roughness)
{
	float alphaRoughnessSq = roughness * roughness;
	float f                = (NdotH * alphaRoughnessSq - NdotH) * NdotH + 1.0;
	return alphaRoughnessSq / (PI * f * f);
}

vec3 normal()
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

#ifdef HAS_NORMAL_TEXTURE
	vec3 n = texture(normal_texture, v_uv).rgb;
	return normalize(TBN * (2.0 * n - 1.0));
#else
	return normalize(TBN[2].xyz);
#endif
}

vec3 diffuse(vec3 albedo, float metallic)
{
	return albedo * (1.0 - metallic) + ((1.0 - metallic) * albedo) * metallic;
}

float saturate(float t)
{
	return clamp(t, 0.0, 1.0);
}

vec3 saturate(vec3 t)
{
	return clamp(t, 0.0, 1.0);
}

float pow2(float t){
	return t * t;
}

float pow4(float t){
	return t * t * t * t;
}

float smoothstep01(float t){
	return t * t * (3.0 - 2.0 * t);
}

void main(void)
{
	vec3 F0 		 = vec3(0.04);
	float F90        = saturate(50.0 * F0.r);
	vec4  base_color = vec4(0.0, 0.0, 0.0, 1.0);

#ifdef HAS_BASE_COLOR_TEXTURE
	base_color = texture(base_color_texture, v_uv);
#else
	base_color = base_color_factor;
#endif

#ifdef HAS_METALLIC_ROUGHNESS_TEXTURE
    vec4 params     = texture(metallic_roughness_texture, v_uv);
	float roughness = saturate(params.g);
	float metallic  = saturate(params.b);
#else
	float roughness = roughness_factor;
	float metallic  = metallic_factor;
#endif

	// screen space shadow
	vec2 screenUV = gl_FragCoord.xy / vec2(viewPort);
	float shadowAttr = texture(main_light_shadow_texture, screenUV).r;

	vec3  N     = normal();
	vec3  V     = normalize(camera_position - v_pos);
	float NdotV = saturate(dot(N, V));

	vec3 diffuse_color      = base_color.rgb * (1.0 - metallic);
	vec3 light_contribution = vec3(0.0);

    // Direction Light
    {
        vec3 L = -direction_light.xyz;
		vec3 H = normalize(V + L);

		float LdotH = saturate(dot(L, H));
		float NdotH = saturate(dot(N, H));
		float NdotL = saturate(dot(N, L));

		vec3  F   = F_Schlick(F0, F90, LdotH);
		float Vis = V_SmithGGXCorrelated(NdotV, NdotL, roughness);
		float D   = D_GGX(NdotH, roughness);
		vec3  Fr  = F * D * Vis;
        float Fd  = Fr_DisneyDiffuse(NdotV, NdotL, LdotH, roughness);
        light_contribution += NdotL * shadowAttr * direction_light_color.xyz * direction_light_color.w * (diffuse_color * (vec3(1.0) - F) * Fd + Fr);
    }

    uvec2 tile_pos   = GetTilePos(gl_FragCoord.xy, vec2(inv_tile_dim));
    uint tile_index  = GetTileIndex(tile_pos, tile_count_x);
    uint tile_offset = GetTileOffset(tile_index);

    uint point_light_count = lightGridData[tile_offset];
    uint cone_light_count  = lightGridData[tile_offset + 1u];
    uint index_offset      = tile_offset + 4u;

    // Point Light
	for (uint i = 0u; i < point_light_count; ++i)
	{
        LightBufferData light_data = ConvertLightBufferByte(lightBufferData[lightGridData[index_offset + i]].value);

		vec3 world_to_light = light_data.position - v_pos;
		float dist			= length(world_to_light);
		// https://imdoingitwrong.wordpress.com/2011/02/10/improved-light-attenuation/
		float atten = 1.0 - pow4(smoothstep01(saturate(dist / light_data.radius)));

		atten = max(atten, 0.0) * light_data.intensity;
		vec3 L = normalize(world_to_light);
		vec3 H = normalize(V + L);

		float LdotH = saturate(dot(L, H));
		float NdotH = saturate(dot(N, H));
		float NdotL = saturate(dot(N, L));

		if(NdotL * atten < 1e-4){
			continue;
		}

		vec3  F   = F_Schlick(F0, F90, LdotH);
		float Vis = V_SmithGGXCorrelated(NdotV, NdotL, roughness);
		float D   = D_GGX(NdotH, roughness);
		vec3  Fr  = F * D * Vis;

		float Fd = Fr_DisneyDiffuse(NdotV, NdotL, LdotH, roughness);
		light_contribution += light_data.color * atten * NdotL * (diffuse_color * (vec3(1.0) - F) * Fd + Fr);
	}

    // Cone Light
    index_offset += point_light_count;
    for(uint i = 0u; i < cone_light_count; ++i){
        LightBufferData light_data = ConvertLightBufferByte(lightBufferData[lightGridData[index_offset + i]].value);

		vec3 world_to_light = light_data.position - v_pos;
		float dist 			= length(world_to_light);
		float atten = 1.0 - pow4(smoothstep01(saturate(dist / light_data.radius)));

		vec3 L 			   = normalize(world_to_light);
    	float cone_falloff = dot(-L, light_data.coneDir);
    	cone_falloff 	   = saturate((cone_falloff - light_data.coneAngles.y) * light_data.coneAngles.x);
		atten *= cone_falloff;

		atten = max(atten, 0.0) * light_data.intensity;
		vec3 H = normalize(V + L);

		float LdotH = saturate(dot(L, H));
		float NdotH = saturate(dot(N, H));
		float NdotL = saturate(dot(N, L));

		if(NdotL * atten < 1e-4){
			continue;
		}

		vec3  F   = F_Schlick(F0, F90, LdotH);
		float Vis = V_SmithGGXCorrelated(NdotV, NdotL, roughness);
		float D   = D_GGX(NdotH, roughness);
		vec3  Fr  = F * D * Vis;

		float Fd = Fr_DisneyDiffuse(NdotV, NdotL, LdotH, roughness);
		light_contribution += light_data.color * atten * NdotL * (diffuse_color * (vec3(1.0) - F) * Fd + Fr);
    }

    // Abient
	// [1] Tempory irradiance to fix dark metals
	// TODO: add specular irradiance for realistic metals
	vec3 irradiance    = vec3(0.2);
	vec3 F             = F_Schlick_Roughness(F0, max(dot(N, V), 0.0), roughness * roughness * roughness * roughness);
	vec3 ibl_diffuse   = irradiance * base_color.rgb;
	vec3 ambient_color = ibl_diffuse * shadowAttr;
	o_color = vec4(ambient_color * 0.3 + light_contribution, base_color.a);
}
