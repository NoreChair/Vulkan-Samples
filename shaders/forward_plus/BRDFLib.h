#ifndef SHADER_BRDF_LIB_H
#define SHADER_BRDF_LIB_H

#define M_PI 3.141592653;

float Pow2(float v)
{
	return v * v;
}

float Pow3(float v)
{
	return v * v * v;
}

float saturate(float v)
{
	return clamp(v, 0.0, 1.0);
}

// [ Duff et al. 2017, "Building an Orthonormal Basis, Revisited" ]
mat3 GetTangentBasis(vec3 tangentZ)
{
	const float sign = tangentZ.z >= 0 ? 1 : -1;
	const float a    = -rcp(sign + tangentZ.z);
	const float b    = tangentZ.x * tangentZ.y * a;

	vec3 tangentX = {1 + sign * a * Pow2(tangentZ.x), sign * b, -sign * tangentZ.x};
	vec3 tangentY = {b, sign + a * Pow2(tangentZ.y), -tangentZ.y};

	return vec3x3(tangentX, tangentY, tangentZ);
}

/**************************************************************************************************************/
/*                                                    BRDF                                                    */
/**************************************************************************************************************/

// Frensel Schlick
vec3 F_Schlick(vec3 f0, float f90, float u)
{
	return f0 + (f90 - f0) * pow(1.0 - u, 5.0);
}

// IBL Defuse Irradiance
vec3 F_Schlick_Roughness(vec3 f0, float cos_theta, float roughness)
{
	return f0 + (max(vec3(1.0 - roughness), f0) - f0) * pow(1.0 - cos_theta, 5.0);
}

// Diffuse Term
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

// Specular Microfacet Model
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

// GGX Normal Distribution Function
float D_GGX(float NdotH, float roughness)
{
	float alphaRoughnessSq = roughness * roughness;
	float f                = (NdotH * alphaRoughnessSq - NdotH) * NdotH + 1.0;
	return alphaRoughnessSq / (PI * f * f);
}

/**************************************************************************************************************/
/*                                        Monte Carlo Importance Sample                                       */
/**************************************************************************************************************/

vec4 UniformSampleHemisphere(vec2 randomNum)
{
	float phi      = 2.0 * M_PI * randomNum.x;
	float cosTheta = randomNum.y;
	float sinTheta = sqrt(1 - cosTheta * cosTheta);

	float pdf = 1.0 / (2 * M_PI);

	return vec4(sinTheta * cos(phi), sinTheta * sin(phi), cosTheta, pdf);
}

vec4 CosineSampleHemisphere(vec2 randomNum)
{
	float phi      = 2.0 * M_PI * randomNum.x;
	float cosTheta = sqrt(randomNum.y);
	float sinTheta = sqrt(1.0 - cosTheta * cosTheta);

	float pdf = cosTheta * (1.0 / M_PI);

	return vec4(sinTheta * cos(phi), sinTheta * sin(phi), cosTheta, pdf);
}

// pdf(theta, phi) = (n+1)/2pi * cos(theta)^n * sin(theta)
// pdf(theta) = integral(pdf(theta,phi)d(phi)) from 0 ~ 2pi = (n+1) * cos(theta)^n * sin(theta)
// pdf(phi) = pdf(theta, phi)/pdf(theta) = 1/2pi
// then intergral pdf to get cdf then inverted to map a pair of uniform random to sample direction
vec4 ImportanceSample_Phong(float n /* shininess */, vec2 randomNum)
{
	float cosTheta = saturate(pow(randomNum.x, 1.0 / (n + 1.0)));
	float phi      = 2.0 * M_PI * randomNum.y;

	vec2 cossinT = vec2(cosTheta, 1.0 - Pow2(cosTheta));
	vec2 cossinP = vec2(cos(phi), sin(phi));

	float pdf = (n + 1) / (2.0 * M_PI) * pow(cossinT.x, n) * cossinT.y;

	return vec4(cossinP.x * cossinT.y, cossinP.y * cossinT.y, cossinT.x, pdf);
	// then transform from tangent space finally to world space
}

vec4 ImportanceSample_Blinn(float n /* shininess */, vec2 randomNum)
{
	float cosTheta = saturate(pow(randomNum.x, 1.0 / (n + 2.0)));
	float phi      = 2.0 * M_PI * randomNum.y;

	vec2 cossinT = vec2(cosTheta, 1.0 - Pow2(cosTheta));
	vec2 cossinP = vec2(cos(phi), sin(phi));

	float pdf = (n + 2.0) / (2.0 * M_PI) * pow(cossinT.x, n + 1.0) * cossinT.y;

	return vec4(cossinP.x * cossinT.y, cossinP.y * cossinT.y, cossinT.x, pdf);
}

// [2015, "Sampling microfacet BRDF – A Graphics guy's note"]
// D(h) = 1.0/(pi * a^2 * cos(theta)^4) * e^(-(tan(theta) / a)^2)
// pdf(theta, phi) = sin(theta)/(pi* a^2 * cos(theta)^3)e^(-(tan(theta) / a)^2)
// pdf(theta) = 2sin(theta)/(a^2 * cos(theta)^3)e^(-(tan(theta) / a)^2)
vec4 ImportanceSample_Beckmann(float alpha2 vec2 randomNum)
{
	float cosTheta = sqrt(1.0 / (1.0 - alpha2 * log(1.0 - randomNum.x)));
	float phi      = 2.0 * M_PI * randomNum.y;

	float sinTheta = sqrt(1.0 - Pow2(cosTheta));
	float tanTheta = sinTheta / cosTheta;

	float pdf = (sinTheta) / (M_PI * alpha2 * Pow3(cosTheta)) * exp(-Pow2(tanTheta) / alpha2);

	return vec4(sinTheta * cos(phi), sinTheta * sin(phi), cosTheta, pdf);
}

// pdf(theta, phi) = (a^2 * cos(theta) * sin(theta))/(pi * ((a^2 - 1) * cos^2(theta) + 1)^2)
vec4 ImportanceSample_GGX(float alpha2, vec2 randomNum)
{
	float cosTheta = sqrt((1.0 - randomNum.x) / (randomNum.x * (alpha2 - 1.0)) + 1.0);
	float phi      = 2.0 * M_PI * randomNum.y;

	float sinTheta = sqrt(1.0 - Pow2(cosTheta));

	float pdf = (alpha2 * cosTheta * sinTheta) / (M_PI * Pow2((alpha2 - 1.0) * Pow2(cosTheta) + 1.0));

	return vec4(sinTheta * cos(phi), sinTheta * sin(phi), cosTheta, pdf);
}

// Input view: view direction
// Input alphaX, alphaY: roughness parameters
// Input randomNum: uniform random numbers
// Output Ne: normal sampled with PDF D_Ve(Ne) = G1(view) * max(0, dot(view, Ne)) * D(Ne) / view.z
// [ Heitz 2018, "Sampling the GGX Distribution of Visible Normals" ]
vec3 ImportanceSample_VisGGX_Aniso(vec3 view, float alphaX, float alphaY, vec2 randomNum)
{
	// Section 3.2: transforming the view direction to the hemisphere configuration
	vec3 Vh = normalize(vec3(alphaX * view.x, alphaY * view.y, view.z));

	// Section 4.1: orthonormal basis (with special case if cross product is zero)
	float lensq = Vh.x * Vh.x + Vh.y * Vh.y;
	vec3  T1    = lensq > 0 ? vec3(-Vh.y, Vh.x, 0) * inversesqrt(lensq) : vec3(1, 0, 0);
	vec3  T2    = cross(Vh, T1);

	// Section 4.2: parameterization of the projected area
	float r   = sqrt(randomNum.x);
	float phi = 2.0 * M_PI * randomNum.y;
	float t1  = r * cos(phi);
	float t2  = r * sin(phi);
	float s   = 0.5 * (1.0 + Vh.z);
	t2        = (1.0 - s) * sqrt(1.0 - t1 * t1) + s * t2;

	// Section 4.3: reprojection onto hemisphere
	vec3 Nh = t1 * T1 + t2 * T2 + sqrt(max(0.0, 1.0 - t1 * t1 - t2 * t2)) * Vh;

	// Section 3.4: transforming the normal back to the ellipsoid configuration
	vec3 Ne = normalize(vec3(alphaX * Nh.x, alphaY * Nh.y, max(0.0, Nh.z)));
	return Ne;
}

vec3 ImportanceSample_VisGGX(vec3 view, float alpha, vec2 randomNum)
{
	return ImportanceSample_VisGGX_Aniso(view, alpha, alpha, randomNum);
}

#endif