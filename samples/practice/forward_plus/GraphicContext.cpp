#include "GraphicContext.h"

using namespace vkb;
using namespace vkb::core;

namespace GraphicContext {

    std::shared_ptr<vkb::core::Image>     linearDepthImage{nullptr};
    std::shared_ptr<vkb::core::Image>     screenShadowImage{nullptr};
    std::shared_ptr<vkb::core::Image>     lumaResultImage{nullptr};

    std::shared_ptr<vkb::core::Buffer>    lightBuffer{nullptr};
    std::shared_ptr<vkb::core::Buffer>    lightGridBuffer{nullptr};
    std::shared_ptr<vkb::core::Buffer>    lightMaskBuffer{nullptr};
    std::shared_ptr<vkb::core::Buffer>    postProcessVB{nullptr};
    std::shared_ptr<vkb::core::Buffer>    exposureBuffer{nullptr};
    std::shared_ptr<vkb::core::Buffer>    lumaHistogram{nullptr};

    std::shared_ptr<vkb::RenderTarget>    offScreenRT{nullptr};
    std::shared_ptr<vkb::RenderTarget>    shadowMapRT{nullptr};

    std::shared_ptr<vkb::core::ImageView> linearDepthImageView{nullptr};
    std::shared_ptr<vkb::core::ImageView> screenShadowImageView{nullptr};
    std::shared_ptr<vkb::core::ImageView> lumaResultImageView{nullptr};

    void Init(vkb::Device & device, int width, int height) {
        VkExtent3D extent{width, height, 1};
        VkExtent3D shadowExtent{2048, 2048, 1};
        // Create Render Image
        Image depthImage(device, extent, VK_FORMAT_D32_SFLOAT, VK_IMAGE_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT | VK_IMAGE_USAGE_SAMPLED_BIT | VK_IMAGE_USAGE_STORAGE_BIT, VmaMemoryUsage::VMA_MEMORY_USAGE_GPU_ONLY);

        Image colorImage(device, extent, VK_FORMAT_R8G8B8A8_SRGB, VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_SAMPLED_BIT | VK_IMAGE_USAGE_TRANSFER_SRC_BIT, VmaMemoryUsage::VMA_MEMORY_USAGE_GPU_ONLY);
        Image shadowImage(device, shadowExtent, VK_FORMAT_D32_SFLOAT, VK_IMAGE_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT | VK_IMAGE_USAGE_SAMPLED_BIT | VK_IMAGE_USAGE_STORAGE_BIT, VmaMemoryUsage::VMA_MEMORY_USAGE_GPU_ONLY);

        linearDepthImage = std::make_shared<Image>(device, extent, VK_FORMAT_R32_SFLOAT, VK_IMAGE_USAGE_STORAGE_BIT | VK_IMAGE_USAGE_SAMPLED_BIT, VmaMemoryUsage::VMA_MEMORY_USAGE_GPU_ONLY);
        linearDepthImageView = std::make_shared<ImageView>(*linearDepthImage, VK_IMAGE_VIEW_TYPE_2D);

        screenShadowImage = std::make_shared<Image>(device, extent, VK_FORMAT_R8_UNORM, VK_IMAGE_USAGE_STORAGE_BIT | VK_IMAGE_USAGE_SAMPLED_BIT, VmaMemoryUsage::VMA_MEMORY_USAGE_GPU_ONLY);
        screenShadowImageView = std::make_shared<ImageView>(*screenShadowImage, VK_IMAGE_VIEW_TYPE_2D);

        lumaResultImage = std::make_shared<Image>(device, extent, VK_FORMAT_R8_UINT, VK_IMAGE_USAGE_STORAGE_BIT, VmaMemoryUsage::VMA_MEMORY_USAGE_GPU_ONLY);
        lumaResultImageView = std::make_shared<ImageView>(*lumaResultImage, VK_IMAGE_VIEW_TYPE_2D);

        std::vector<Image> offScreenImgs;
        offScreenImgs.emplace_back(std::move(colorImage));
        offScreenImgs.emplace_back(std::move(depthImage));
        offScreenRT = std::make_shared<RenderTarget>(std::move(offScreenImgs));

        std::vector<Image> shadowImgs;
        shadowImgs.emplace_back(std::move(shadowImage));
        shadowMapRT = std::make_shared<RenderTarget>(std::move(shadowImgs));
    }
}