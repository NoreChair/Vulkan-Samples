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

#ifndef PI
#define PI glm::pi<double>()
#define TWO_SQRT_PI (2.0 * glm::sqrt(PI))
#endif

const double k_PolynomialFormsOfSHBasis[3*3] = {
    // L<sub>0</sub>
    1.0/TWO_SQRT_PI,
    // L<sub>1</sub>
    -glm::sqrt(3.0) / TWO_SQRT_PI, glm::sqrt(3.0) / TWO_SQRT_PI, -glm::sqrt(3.0) / TWO_SQRT_PI,
    // L<sub>2</sub>
    glm::sqrt(15.0) / TWO_SQRT_PI, -glm::sqrt(15.0) / TWO_SQRT_PI, glm::sqrt(5.0) / (2.0 * TWO_SQRT_PI), -glm::sqrt(15.0) / TWO_SQRT_PI, glm::sqrt(15.0) / (2.0 * TWO_SQRT_PI)
};


// https://www.ppsloan.org/publications/StupidSH36.pdf
// this only for K factor
// P factor need `Associated Legendre polynomials`,can and be evaluated using these recurrences P^0<sub>0</sub> = 1
// cos(m$/psi$), sin(m$/psi$) can be evaluated using C(m),S(m) with C(0) = 1, S(0) = 0
double SHK(const unsigned int l, const int m) {
    const unsigned int cAM = abs(m);
    double uVal = 1;// must be double
    for (unsigned int k = l + cAM; k > (l - cAM); k--) uVal *= k;
    return sqrt((2.0 * l + 1.0) / (4 * glm::pi<double>() * uVal));
}

void SHProject3(const glm::vec3 n, float* __restrict pWeight) {
    glm::vec3 sq = n * n;
    const double* w = k_PolynomialFormsOfSHBasis;
    pWeight[0] = w[0];
    pWeight[1] = w[1] * n.x;
    pWeight[2] = w[2] * n.y;
    pWeight[3] = w[3] * n.z;
    pWeight[4] = w[4] * (-sq.x - sq.y + 2 * sq.z);
    pWeight[5] = w[5] * n.y * n.z;
    pWeight[6] = w[6] * n.z * n.x;
    pWeight[7] = w[7] * n.x * n.y;
    pWeight[8] = w[8] * (sq.x - sq.y);
}

void SHConstruct3(const glm::vec3 f, float* __restrict pSH) {
    float fC0, fC1, fS0, fS1, fTmpA, fTmpB, fTmpC;
    float fZ2 = f.z * f.z;
    pSH[0] = 0.2820947917738781f;
    pSH[2] = 0.4886025119029199f * f.z;
    pSH[6] = 0.9461746957575601f * fZ2 + -0.3153915652525201f;
    fC0 = f.x;
    fS0 = f.y;
    fTmpA = -0.48860251190292f;
    pSH[3] = fTmpA * fC0;
    pSH[1] = fTmpA * fS0;
    fTmpB = -1.092548430592079f * f.z;
    pSH[7] = fTmpB * fC0;
    pSH[5] = fTmpB * fS0;
    fC1 = f.x * fC0 - f.y * fS0;
    fS1 = f.x * fS0 + f.y * fC0;
    fTmpC = 0.5462742152960395f;
    pSH[8] = fTmpC * fC1;
    pSH[4] = fTmpC * fS1;
}