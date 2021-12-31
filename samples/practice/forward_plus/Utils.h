#pragma once
#include "common/glm_common.h"

#if defined(DEBUG) || defined(_DEBUG)
#	define DEBUG_ASSERT(exp, ...) \
		{                          \
			if (!(exp))            \
				LOGD(__VA_ARGS__)  \
			assert(exp);           \
		}
#else
#	define DEBUG_ASSERT(exp, ...) 0
#endif

#define MAX_LIGHTS_COUNT 128

struct alignas(16) GlobalUniform
{
	glm::mat4 model;
	glm::mat4 view_project;
	glm::vec3 camera_position;
};

struct LightBuffer
{
	glm::vec3 position;          // xyz=pos
	glm::vec3 color;             // xyz=rgb
	glm::vec3 coneDir;           // xyz=direction if light source is spot
	glm::vec3 coneAngles;        // x=1.0f/(cos(inner)-cos(outer)), y=cos(inner), z=cos(outer/2)
	float     radius;
	float     intensity;
	uint32_t  lightType;
	uint32_t  padding;
};

#ifndef TWO_SQRT_PI
#define TWO_SQRT_PI 3.5449077018110320545963349666f //(2.0f * glm::sqrt(PI))
#endif

const float k_FactorOfSHBasis[6*6] = {
    // L<sub>0</sub>
    1.0f / TWO_SQRT_PI,
    // L<sub>1</sub>
    -glm::sqrt(3.0f)          / TWO_SQRT_PI,
    glm::sqrt(3.0f)           / TWO_SQRT_PI,
    -glm::sqrt(3.0f)          / TWO_SQRT_PI,
    // L<sub>2</sub>
    glm::sqrt(15.0f)          / TWO_SQRT_PI,
    -glm::sqrt(15.0f)         / TWO_SQRT_PI,
    glm::sqrt(5.0f)           / (2.0f * TWO_SQRT_PI),
    -glm::sqrt(15.0f)         / TWO_SQRT_PI,
    glm::sqrt(15.0f)          / (2.0f * TWO_SQRT_PI),
    // L<sub>3</sub>
    -glm::sqrt(70.0f)         / (4.0f * TWO_SQRT_PI),
    glm::sqrt(105.0f)         / TWO_SQRT_PI,
    -glm::sqrt(42.0f)         / (4.0f * TWO_SQRT_PI),
    glm::sqrt(7.0f)           / (2.0f * TWO_SQRT_PI),
    -glm::sqrt(42.0f)         / (4.0f * TWO_SQRT_PI),
    glm::sqrt(105.0f)         / (2.0f * TWO_SQRT_PI),
    -glm::sqrt(70.0f)         / (4.0f * TWO_SQRT_PI),
    // L<sub>4</sub>
    3.0f * glm::sqrt(35.0f)   / (2.0f * TWO_SQRT_PI),
    -3.0f * glm::sqrt(70.0f)  / (4.0f * TWO_SQRT_PI),
    3.0f * glm::sqrt(5.0f)    / (2.0f * TWO_SQRT_PI),
    -3.0f * glm::sqrt(10.0f)  / (4.0f * TWO_SQRT_PI),
    3.0f                      / (8.0f * TWO_SQRT_PI),
    -3.0f * glm::sqrt(10.0f)  / (4.0f * TWO_SQRT_PI),
    3.0f * glm::sqrt(5.0f)    / (4.0f * TWO_SQRT_PI),
    -3.0f * glm::sqrt(70.0f)  / (4.0f * TWO_SQRT_PI),
    3.0f * glm::sqrt(35.0f)   / (8.0f * TWO_SQRT_PI),
    // L<sub>5</sub>
    -3.0f * glm::sqrt(154.0f) / (16.0f * TWO_SQRT_PI),
    3.0f * glm::sqrt(385.0f)  / (2.0f * TWO_SQRT_PI),
    -glm::sqrt(770.0f)        / (16.0f * TWO_SQRT_PI),
    glm::sqrt(1155.0f)        / (2.0f * TWO_SQRT_PI),
    -glm::sqrt(165.0f)        / (8.0f * TWO_SQRT_PI),
    glm::sqrt(11.0f)          / (8.0f * TWO_SQRT_PI),
    -glm::sqrt(165.0f)        / (8.0f * TWO_SQRT_PI),
    glm::sqrt(1155.0f)        / (4.0f * TWO_SQRT_PI),
    -glm::sqrt(770.0f)        / (16.0f * TWO_SQRT_PI),
    3.0f * glm::sqrt(385.0f)  / (8.0f * TWO_SQRT_PI),
    -3.0f * glm::sqrt(154.0f) / (16.0f * TWO_SQRT_PI)
};


// https://www.ppsloan.org/publications/StupidSH36.pdf
// this only for K factor
// P factor need `Associated Legendre polynomials`,can and be evaluated using these recurrences P^0<sub>0</sub> = 1
// cos(m$/psi$), sin(m$/psi$) can be evaluated using C(m),S(m) with C(0) = 1, S(0) = 0
inline double SHK(const unsigned int l, const int m) {
    const unsigned int cAM = abs(m);
    double uVal = 1;// must be double
    for (unsigned int k = l + cAM; k > (l - cAM); k--) uVal *= k;
    return sqrt((2.0 * l + 1.0) / (4 * glm::pi<double>() * uVal));
}

//void SHProject3(const glm::vec3 f, float* __restrict pSH) {
//    float fC0, fC1, fS0, fS1, fTmpA, fTmpB, fTmpC;
//    float fZ2 = f.z * f.z;
//    pSH[0] = 0.2820947917738781f;
//    pSH[2] = 0.4886025119029199f * f.z;
//    pSH[6] = 0.9461746957575601f * fZ2 + -0.3153915652525201f;
//    fC0 = f.x;
//    fS0 = f.y;
//    fTmpA = -0.48860251190292f;
//    pSH[3] = fTmpA * fC0;
//    pSH[1] = fTmpA * fS0;
//    fTmpB = -1.092548430592079f * f.z;
//    pSH[7] = fTmpB * fC0;
//    pSH[5] = fTmpB * fS0;
//    fC1 = f.x * fC0 - f.y * fS0;
//    fS1 = f.x * fS0 + f.y * fC0;
//    fTmpC = 0.5462742152960395f;
//    pSH[8] = fTmpC * fC1;
//    pSH[4] = fTmpC * fS1;
//}

inline void SHProject3(const glm::vec3& n, float* __restrict pWeight) {
    float x = n.x; float y = n.y; float z = n.z;
    float x2 = x * x; float y2 = y * y; float z2 = z * z;

    const float* w = k_FactorOfSHBasis;
    pWeight[0] = w[0];
    pWeight[1] = w[1] * y;
    pWeight[2] = w[2] * z;
    pWeight[3] = w[3] * x;
    pWeight[4] = w[4] * x * y;
    pWeight[5] = w[5] * y * z;
    pWeight[6] = w[6] * (3.0f * z2 - 1.0f);
    pWeight[7] = w[7] * z * x;
    pWeight[8] = w[8] * (x2 - y2);
}

inline void SHProject4(const glm::vec3& n, float* __restrict pWeight) {
    float x = n.x; float y = n.y; float z = n.z;
    float x2 = x * x; float y2 = y * y; float z2 = z * z;

    const float* w = k_FactorOfSHBasis;
    pWeight[0] = w[0];
    pWeight[1] = w[1] * y;
    pWeight[2] = w[2] * z;
    pWeight[3] = w[3] * x;
    pWeight[4] = w[4] * x * y;
    pWeight[5] = w[5] * y * z;
    pWeight[6] = w[6] * (3.0f * z2 - 1.0f);
    pWeight[7] = w[7] * z * x;
    pWeight[8] = w[8] * (x2 - y2);
    pWeight[9] = w[9] * y * (3.0f * x2 - y2);
    pWeight[10] = w[10] * y * x * z;
    pWeight[11] = w[11] * y * (-1.0f + 5.0f * z2);
    pWeight[12] = w[12] * z * (5.0f * z2 - 3.0f);
    pWeight[13] = w[13] * x * (-1.0f + 5.0f * z2);
    pWeight[14] = w[14] * z * (x2 - y2);
    pWeight[15] = w[15] * x * (x2 - 3.0f * y2);
    // TODO : L4 , L5
}

inline float SHConstruct3(const glm::vec3& n, const float* pWeight) {
    float x = n.x; float y = n.y; float z = n.z;
    float x2 = x * x; float y2 = y * y; float z2 = z * z;

    const float* w = k_FactorOfSHBasis;
    float result = w[0];
    result += pWeight[1] * w[1] * y;
    result += pWeight[2] * w[2] * z;
    result += pWeight[3] * w[3] * x;
    result += pWeight[4] * w[4] * x * y;
    result += pWeight[5] * w[5] * y * z;
    result += pWeight[6] * w[6] * (3.0f * z2 - 1.0f);
    result += pWeight[7] * w[7] * z * x;
    result += pWeight[8] * w[8] * (x2 - y2);
    return result;
}