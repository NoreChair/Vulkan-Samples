#pragma once
#include "common/glm_common.h"

#ifndef TWO_SQRT_PI
#define TWO_SQRT_PI 3.5449077018110320545963349666f //(2.0f * glm::sqrt(PI))
#endif

const float k_FactorOfSHBasis[6 * 6] = {
    // L<sub>0</sub>
    1.0f / TWO_SQRT_PI,
    // L<sub>1</sub>
    -glm::sqrt(3.0f) / TWO_SQRT_PI,
    glm::sqrt(3.0f) / TWO_SQRT_PI,
    -glm::sqrt(3.0f) / TWO_SQRT_PI,
    // L<sub>2</sub>
    glm::sqrt(15.0f) / TWO_SQRT_PI,
    -glm::sqrt(15.0f) / TWO_SQRT_PI,
    glm::sqrt(5.0f) / (2.0f * TWO_SQRT_PI),
    -glm::sqrt(15.0f) / TWO_SQRT_PI,
    glm::sqrt(15.0f) / (2.0f * TWO_SQRT_PI),
    // L<sub>3</sub>
    -glm::sqrt(70.0f) / (4.0f * TWO_SQRT_PI),
    glm::sqrt(105.0f) / TWO_SQRT_PI,
    -glm::sqrt(42.0f) / (4.0f * TWO_SQRT_PI),
    glm::sqrt(7.0f) / (2.0f * TWO_SQRT_PI),
    -glm::sqrt(42.0f) / (4.0f * TWO_SQRT_PI),
    glm::sqrt(105.0f) / (2.0f * TWO_SQRT_PI),
    -glm::sqrt(70.0f) / (4.0f * TWO_SQRT_PI),
    // L<sub>4</sub>
    3.0f * glm::sqrt(35.0f) / (2.0f * TWO_SQRT_PI),
    -3.0f * glm::sqrt(70.0f) / (4.0f * TWO_SQRT_PI),
    3.0f * glm::sqrt(5.0f) / (2.0f * TWO_SQRT_PI),
    -3.0f * glm::sqrt(10.0f) / (4.0f * TWO_SQRT_PI),
    3.0f / (8.0f * TWO_SQRT_PI),
    -3.0f * glm::sqrt(10.0f) / (4.0f * TWO_SQRT_PI),
    3.0f * glm::sqrt(5.0f) / (4.0f * TWO_SQRT_PI),
    -3.0f * glm::sqrt(70.0f) / (4.0f * TWO_SQRT_PI),
    3.0f * glm::sqrt(35.0f) / (8.0f * TWO_SQRT_PI),
    // L<sub>5</sub>
    -3.0f * glm::sqrt(154.0f) / (16.0f * TWO_SQRT_PI),
    3.0f * glm::sqrt(385.0f) / (2.0f * TWO_SQRT_PI),
    -glm::sqrt(770.0f) / (16.0f * TWO_SQRT_PI),
    glm::sqrt(1155.0f) / (2.0f * TWO_SQRT_PI),
    -glm::sqrt(165.0f) / (8.0f * TWO_SQRT_PI),
    glm::sqrt(11.0f) / (8.0f * TWO_SQRT_PI),
    -glm::sqrt(165.0f) / (8.0f * TWO_SQRT_PI),
    glm::sqrt(1155.0f) / (4.0f * TWO_SQRT_PI),
    -glm::sqrt(770.0f) / (16.0f * TWO_SQRT_PI),
    3.0f * glm::sqrt(385.0f) / (8.0f * TWO_SQRT_PI),
    -3.0f * glm::sqrt(154.0f) / (16.0f * TWO_SQRT_PI)
};

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