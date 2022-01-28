#pragma once
#include "common/glm_common.h"

#ifndef TWO_SQRT_PI
#define TWO_SQRT_PI 3.5449077018110320545963349666f //(2.0f * glm::sqrt(PI))
#endif

extern const float k_FactorOfSHBasis[6 * 6];

void SHProject3(const glm::vec3& n, float* __restrict pWeight);

void SHProject4(const glm::vec3& n, float* __restrict pWeight);

float SHConstruct3(const glm::vec3& n, const float* pWeight);

// pImageData   : rgb or rgba color from cube map
// width        : image width
// height       : image height
// stride       : stride of pixel
// pSH          : array length must be 9*3(r,g,b), the result order : Wr0, Wr1,... Wr8, Wg0,... Wg8, Wb0,... Wb8
// order        : <= 6
void ProjectFromCubeMap(const float* pImageData, const int width, const int height, const int stride, float* pSH);