#pragma once
#include "common/vk_common.h"
#include "core/image.h"
#include "core/buffer.h"
#include "rendering/render_target.h"

namespace GraphicContext{
    extern std::shared_ptr<vkb::core::Image>     linearDepthImage;
    extern std::shared_ptr<vkb::core::Image>     screenShadowImage;
    extern std::shared_ptr<vkb::core::Image>     lumaResultImage;

    extern std::shared_ptr<vkb::core::Buffer>    lightBuffer;
    extern std::shared_ptr<vkb::core::Buffer>    lightGridBuffer;
    extern std::shared_ptr<vkb::core::Buffer>    lightMaskBuffer;
    extern std::shared_ptr<vkb::core::Buffer>    postProcessVB;
    extern std::shared_ptr<vkb::core::Buffer>    exposureBuffer;
    extern std::shared_ptr<vkb::core::Buffer>    lumaHistogram;

    extern std::shared_ptr<vkb::RenderTarget>    offScreenRT;
    extern std::shared_ptr<vkb::RenderTarget>    shadowMapRT;

    extern std::shared_ptr<vkb::core::ImageView> linearDepthImageView;
    extern std::shared_ptr<vkb::core::ImageView> screenShadowImageView;
    extern std::shared_ptr<vkb::core::ImageView> lumaResultImageView;

    void Init(vkb::Device & device, int width, int height);
}