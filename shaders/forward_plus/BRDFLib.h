#ifndef SHADER_BRDF_LIB_H
#define SHADER_BRDF_LIB_H

/*
    Ref :
        * BRDF Explorer
        * Real-Time Rendering 4th
        * Advanced Global Illumination
        * GPU Gems/Pro
        * Eric Heitz/Joe Schutte blogs and papers
*/

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

float BeckmannToPhongRoughtness(float roughness)
{
	return 2.0 / Pow2(roughness) - 2.0;
}

// [ Duff et al. 2017, "Building an Orthonormal Basis, Revisited" ]
mat3 GetTangentBasis(vec3 tangentZ)
{
	const float sign = tangentZ.z >= 0 ? 1 : -1;
	const float a    = -rcp(sign + tangentZ.z);
	const float b    = tangentZ.x * tangentZ.y * a;

	vec3 tangentX = {1 + sign * a * Pow2(tangentZ.x), sign * b, -sign * tangentZ.x};
	vec3 tangentY = {b, sign + a * Pow2(tangentZ.y), -tangentZ.y};

	return mat3(tangentX, tangentY, tangentZ);
}

/**************************************************************************************************************/
/*                                                    BRDF                                                    */
/**************************************************************************************************************/

// Frensel Schlick
float F_Schlick(float u)
{
	float m  = clamp(1 - u, 0, 1);
	float m2 = m * m;
	return m2 * m2 * m;        // pow(m,5)
}

vec3 F_Schlick(vec3 f0, float f90, float u)
{
	return f0 + (f90 - f0) * pow(1.0 - u, 5.0);
}

// For rough surface that subsurface scattering distances less than microgeometry irregularities
vec3 F_Schlick_Roughness(vec3 f0, float u, float roughness)
{
	return f0 + (max(vec3(1.0 - roughness), f0) - f0) * pow(1.0 - u, 5.0);
}

/*													Diffuse													  */

// Rough-Surface diffuse from disney
float Diffuse_Disney(float ndotv, float ndotl, float ldoth, float roughness)
{
	float fl  = F_Schlick(ndotl);
	float fv  = F_Schlick(ndotv);
	float f90 = 0.5 + 2 * ldoth * ldoth * roughness;
	float fd  = mix(1.0, f90, fl) * mix(1.0, f90, fv);
	return fd;
}

float Diffuse_Lambert(float ndotl)
{
	return ndotl / M_PI;
}

/*													Specular										     	  */

// Height correlated : G2() = 1.0/(1.0 + Λ(v) + Λ(l))

float V_SmithGGXCorrelated(float ndotv, float ndotl, float roughness)
{
	float roughness2 = roughness * roughness;

	float ggxv = ndotl * sqrt(ndotv * ndotv * (1.0 - roughness2) + roughness2);
	float ggxl = ndotv * sqrt(ndotl * ndotl * (1.0 - roughness2) + roughness2);

	float ggx = ggxv + ggxl;
	if (ggx > 0.0)
	{
		return 0.5 / ggx;
	}
	return 0.0;
}

// [2007, "Microfacet Models for Refraction through Rough Surfaces"]
// Blinn- Phong NDF is not shape-invariant, and an analytic form does not exist for its Λ function.
// Walter et suggest using the Beckmann Λ function in conjunction with the 'αp = 2αb − 2' parameter equivalence.
float V_BeckmannApproximate(float ndotv, float ndotl, float roughness)
{
	float alphaV = ndotv / (roughness * sqrt(1.0 - ndotv * ndotv));
	float alphaL = ndotl / (roughness * sqrt(1.0 - ndotl * ndotl));

	vec2  alpha  = vec2(alphaV, alphaL);
	float factor = 1.0 / (4.0 * ndotv * ndotl);
	if (alpha >= 1.6)
	{
		return factor;
	}

	vec2 alpha2 = alpha * alpha;
	vec2 lambda = (1.0 - 1.259 * alpha + 0.396 * alpha2) / (3.535 * alpha + 2.181 * alpha2);
	return 1.0 / (1.0 + lambda.x + lambda.y) * factor;
}

// GGX Normal Distribution Function
float D_GGX(float NdotH, float roughness)
{
	float roughness2 = roughness * roughness;
	float f          = (NdotH * roughness2 - NdotH) * NdotH + 1.0;
	return roughness2 / (PI * f * f);
}

float D_Beckmann(float m, float t)
{
	float m2 = m * m;
	float t2 = t * t;
	return exp((t2 - 1) / (m2 * t2)) / (m2 * t2 * t2);
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

// [2006, "Advanced Global Illumination", Chapter 3.5.1]
vec4 CosineSampleHemisphere(vec2 randomNum)
{
	float phi      = 2.0 * M_PI * randomNum.x;
	float cosTheta = sqrt(randomNum.y);
	float sinTheta = sqrt(1.0 - cosTheta * cosTheta);

	float pdf = cosTheta * (1.0 / M_PI);

	return vec4(sinTheta * cos(phi), sinTheta * sin(phi), cosTheta, pdf);
}

// [gpu gems 3, "GPU Based Importance Sampling"]
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

// almost as the same as phong, but slightly adjust
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

// normal sampled with PDF D_Ve(Ne) = G1(view) * max(0, dot(view, Ne)) * D(Ne) / view.z
// [ Heitz 2018, "Sampling the GGX Distribution of Visible Normals" ]
float VisibleGGXPDF_Aniso(vec3 V, vec3 H, vec2 a)
{
	float NoV  = V.z;
	float NoH  = H.z;
	float VoH  = dot(V, H);
	float a2   = a.x * a.y;
	vec3  Hs   = vec3(a.y * H.x, a.x * H.y, a2 * NoH);
	float S    = dot(Hs, Hs);
	float D    = (1.0f / PI) * a2 * Pow2(a2 / S);
	float LenV = length(vec3(V.x * Alpha.x, V.y * Alpha.y, NoV));
	float pdf  = (2.0 * D * VoH) / (NoV + LenV);
	return pdf;
}

float VisibleGGXPDF(vec3 V, vec3 H, float a)
{
	float NoV = V.z;
	float NoH = H.z;
	float VoH = dot(V, H);
	float a2  = a * a;
	float D   = D_GGX(NoH, a);
	float pdf = 2.0 * VoH * D / (NoV + sqrt(NoV * (NoV - NoV * a2) + a2));
	return pdf;
}

vec4 ImportanceSample_VisGGX_Aniso(vec3 view, float alphaX, float alphaY, vec2 randomNum)
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
	return vec4(Ne, VisibleGGXPDF_Aniso(view, Ne, vec2(alphaX, alphaY)));
}

vec4 ImportanceSample_VisGGX(vec3 view, float alpha, vec2 randomNum)
{
	vec3 Vh = normalize(vec3(alpha * view.xy, view.z));

	float lensq = Vh.x * Vh.x + Vh.y * Vh.y;
	vec3  T1    = lensq > 0 ? vec3(-Vh.y, Vh.x, 0) * inversesqrt(lensq) : vec3(1, 0, 0);
	vec3  T2    = cross(Vh, T1);

	float r   = sqrt(randomNum.x);
	float phi = 2.0 * M_PI * randomNum.y;
	float t1  = r * cos(phi);
	float t2  = r * sin(phi);
	float s   = 0.5 * (1.0 + Vh.z);
	t2        = (1.0 - s) * sqrt(1.0 - t1 * t1) + s * t2;

	vec3 Nh = t1 * T1 + t2 * T2 + sqrt(max(0.0, 1.0 - t1 * t1 - t2 * t2)) * Vh;

	vec3 Ne = normalize(vec3(alpha * Nh.xy, max(0.0, Nh.z)));
	return vec4(Ne, VisibleGGXPDF(view, Ne, alpha));
}

/**************************************************************************************************************/
/*                                        Monte Carlo Integration                                             */
/**************************************************************************************************************/

vec3 IntergralBRDF_GGXD_Template(vec3 view, vec3 normal, float roughness, int sampleCount, vec2 sequence[])
{
	float ndotv          = saturate(dot(normal, view));
	float roughness2     = roughness * roughness;
	mat3  tangentToWorld = GetTangentBasis(normal);
	vec3  sepcularColor  = vec3(1.0, 1.0, 1.0);
	vec3  value          = vec3(0.0, 0.0, 0.0);

	for (int i = 0; i < sampleCount; ++i)
	{
		vec4 result = ImportanceSample_GGX(roughness2, sequence[i]);
		// if we only calc in tangent space, this transform can be skip
		vec3 wm = tangentToWorld * result.xyz;
		vec3 wi = 2.0 * dot(view, wm) * wm - view;

		float mi = dot(wi, wm);
		float ni = dot(wi, normal);
		if (ni > 0.0 && mi > 0.0)
		{
			float ndotl  = saturate(ni);
			vec3  f      = F_Schlick(sepcularColor, 1.0, mi);
			float v      = V_SmithGGXCorrelated(ndotv, ndotl, roughness);
			float weight = ndotl * 4.0 * saturate(dot(view, wm)) * result.w / dot(wm, normal);
			value += f * v * weight;
		}
	}

	return value / sampleCount;
}

#endif