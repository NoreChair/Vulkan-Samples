#pragma once
#include "common/vk_common.h"
#include "core/device.h"
#include "core/image.h"
#include "core/buffer.h"

namespace GraphicContext{
    extern std::shared_ptr<vkb::core::Image>     hdrColorImage;
    extern std::shared_ptr<vkb::core::Image>     sceneDepthImage;
    extern std::shared_ptr<vkb::core::Image>     shadowImage;
    extern std::shared_ptr<vkb::core::Image>     linearDepthImage;
    extern std::shared_ptr<vkb::core::Image>     screenShadowImage;
    extern std::shared_ptr<vkb::core::Image>     lumaResultImage;

    extern std::shared_ptr<vkb::core::Buffer>    lightBuffer;
    extern std::shared_ptr<vkb::core::Buffer>    lightGridBuffer;
    extern std::shared_ptr<vkb::core::Buffer>    lightMaskBuffer;
    extern std::shared_ptr<vkb::core::Buffer>    postProcessVB;
    extern std::shared_ptr<vkb::core::Buffer>    exposureBuffer;
    extern std::shared_ptr<vkb::core::Buffer>    lumaHistogram;

    extern std::shared_ptr<vkb::core::ImageView> hdrColorImageView;
    extern std::shared_ptr<vkb::core::ImageView> sceneDepthImageView;
    extern std::shared_ptr<vkb::core::ImageView> shadowImageView;
    extern std::shared_ptr<vkb::core::ImageView> linearDepthImageView;
    extern std::shared_ptr<vkb::core::ImageView> screenShadowImageView;
    extern std::shared_ptr<vkb::core::ImageView> lumaResultImageView;

    void Init(vkb::Device & device, int width, int height);
}