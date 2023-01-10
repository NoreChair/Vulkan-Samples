#pragma once
#include "common/vk_common.h"
#include "core/buffer.h"
#include "core/device.h"
#include "core/image.h"
#include "core/sampler.h"

namespace GraphicContext
{
#define DECLARE_IMAGE(x) \
    extern std::shared_ptr<vkb::core::Image> x; \
    extern std::shared_ptr<vkb::core::ImageView> x##View;

#define DECLARE_IMAGE_ARRAY(x,c) \
    extern std::shared_ptr<vkb::core::Image> x[c]; \
    extern std::shared_ptr<vkb::core::ImageView> x##View[c];

    DECLARE_IMAGE(hdrColorImage);
    DECLARE_IMAGE(sceneDepthImage);
    DECLARE_IMAGE(sceneNormalImage);
    DECLARE_IMAGE(sceneParamsImage);
    DECLARE_IMAGE(shadowImage);
    DECLARE_IMAGE_ARRAY(linearDepthImage,2);       // TAA used : current/previous
    DECLARE_IMAGE(screenShadowImage);
    DECLARE_IMAGE(lumaResultImage);
    DECLARE_IMAGE_ARRAY(temporalBlendImage, 2);     // TAA used
    DECLARE_IMAGE(velocityImage);
    DECLARE_IMAGE_ARRAY(bloomChainImage, 4);        // 2x/4x/8x/16x down sample
    DECLARE_IMAGE(rayIndexImage);
    DECLARE_IMAGE(reflectImage);

    extern std::shared_ptr<vkb::core::Buffer> lightBuffer;
    extern std::shared_ptr<vkb::core::Buffer> lightGridBuffer;
    extern std::shared_ptr<vkb::core::Buffer> postProcessVB;
    extern std::shared_ptr<vkb::core::Buffer> exposureBuffer;
    extern std::shared_ptr<vkb::core::Buffer> lumaHistogram;
    extern std::shared_ptr<vkb::core::Buffer> raysBuffer;
    extern std::shared_ptr<vkb::core::Buffer> indirectBuffer;

    extern std::vector<std::shared_ptr<vkb::core::ImageView>> depthChainImageViews[2];

    extern std::shared_ptr<vkb::core::Sampler> linearClampSampler;
    extern std::shared_ptr<vkb::core::Sampler> pointClampSampler;

    void Init(vkb::Device &device, int width, int height);
    void Release();
}        // namespace GraphicContext