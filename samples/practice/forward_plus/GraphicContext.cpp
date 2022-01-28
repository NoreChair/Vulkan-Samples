#include "GraphicContext.h"
#include "Utils.h"

using namespace vkb;
using namespace vkb::core;

namespace GraphicContext {

    std::shared_ptr<vkb::core::Image>     hdrColorImage{nullptr};
    std::shared_ptr<vkb::core::Image>     sceneDepthImage{nullptr};
    std::shared_ptr<vkb::core::Image>     shadowImage{nullptr};
    std::shared_ptr<vkb::core::Image>     linearDepthImage{nullptr};
    std::shared_ptr<vkb::core::Image>     screenShadowImage{nullptr};
    std::shared_ptr<vkb::core::Image>     lumaResultImage{nullptr};

    std::shared_ptr<vkb::core::Buffer>    lightBuffer{nullptr};
    std::shared_ptr<vkb::core::Buffer>    lightGridBuffer{nullptr};
    std::shared_ptr<vkb::core::Buffer>    lightMaskBuffer{nullptr};
    std::shared_ptr<vkb::core::Buffer>    postProcessVB{nullptr};
    std::shared_ptr<vkb::core::Buffer>    exposureBuffer{nullptr};
    std::shared_ptr<vkb::core::Buffer>    lumaHistogram{nullptr};

    std::shared_ptr<vkb::core::ImageView> hdrColorImageView{nullptr};
    std::shared_ptr<vkb::core::ImageView> sceneDepthImageView{nullptr};
    std::shared_ptr<vkb::core::ImageView> shadowImageView{nullptr};
    std::shared_ptr<vkb::core::ImageView> linearDepthImageView{nullptr};
    std::shared_ptr<vkb::core::ImageView> screenShadowImageView{nullptr};
    std::shared_ptr<vkb::core::ImageView> lumaResultImageView{nullptr};

    void Init(vkb::Device & device, int width, int height) {
        VkExtent3D extent{width, height, 1};
        VkExtent3D shadowExtent{2048, 2048, 1};

        VkFormat hdrColorFormat = VK_FORMAT_B10G11R11_UFLOAT_PACK32;
        assert(device.is_image_format_supported(hdrColorFormat));
        // Create Render Image

        hdrColorImage = std::make_shared<Image>(device, extent, hdrColorFormat, VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_SAMPLED_BIT | VK_IMAGE_USAGE_STORAGE_BIT, VmaMemoryUsage::VMA_MEMORY_USAGE_GPU_ONLY);
        sceneDepthImage = std::make_shared<Image>(device, extent, VK_FORMAT_D32_SFLOAT, VK_IMAGE_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT | VK_IMAGE_USAGE_STORAGE_BIT, VmaMemoryUsage::VMA_MEMORY_USAGE_GPU_ONLY);
        shadowImage = std::make_shared<Image>(device, shadowExtent, VK_FORMAT_D32_SFLOAT, VK_IMAGE_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT | VK_IMAGE_USAGE_SAMPLED_BIT | VK_IMAGE_USAGE_STORAGE_BIT, VmaMemoryUsage::VMA_MEMORY_USAGE_GPU_ONLY);
        hdrColorImageView = std::make_shared<ImageView>(*hdrColorImage, VK_IMAGE_VIEW_TYPE_2D);
        sceneDepthImageView = std::make_shared<ImageView>(*sceneDepthImage, VK_IMAGE_VIEW_TYPE_2D);
        shadowImageView = std::make_shared<ImageView>(*shadowImage, VK_IMAGE_VIEW_TYPE_2D);

        linearDepthImage = std::make_shared<Image>(device, extent, VK_FORMAT_R32_SFLOAT, VK_IMAGE_USAGE_STORAGE_BIT | VK_IMAGE_USAGE_SAMPLED_BIT, VmaMemoryUsage::VMA_MEMORY_USAGE_GPU_ONLY);
        linearDepthImageView = std::make_shared<ImageView>(*linearDepthImage, VK_IMAGE_VIEW_TYPE_2D);

        screenShadowImage = std::make_shared<Image>(device, extent, VK_FORMAT_R8_UNORM, VK_IMAGE_USAGE_STORAGE_BIT | VK_IMAGE_USAGE_SAMPLED_BIT, VmaMemoryUsage::VMA_MEMORY_USAGE_GPU_ONLY);
        screenShadowImageView = std::make_shared<ImageView>(*screenShadowImage, VK_IMAGE_VIEW_TYPE_2D);

        lumaResultImage = std::make_shared<Image>(device, extent, VK_FORMAT_R8_UINT, VK_IMAGE_USAGE_STORAGE_BIT, VmaMemoryUsage::VMA_MEMORY_USAGE_GPU_ONLY);
        lumaResultImageView = std::make_shared<ImageView>(*lumaResultImage, VK_IMAGE_VIEW_TYPE_2D);


        int tileCount = (int)(glm::ceil(height / 16.0f) * glm::ceil(width / 16.0f));

        lightBuffer = std::make_shared<Buffer>(device, sizeof(LightBuffer) * MAX_LIGHTS_COUNT, VK_BUFFER_USAGE_STORAGE_BUFFER_BIT, VmaMemoryUsage::VMA_MEMORY_USAGE_CPU_TO_GPU);
        lightGridBuffer = std::make_shared<Buffer>(device, sizeof(uint32_t) * (MAX_LIGHTS_COUNT + 4) * tileCount, VK_BUFFER_USAGE_STORAGE_BUFFER_BIT, VmaMemoryUsage::VMA_MEMORY_USAGE_GPU_ONLY);
        //lightMaskBuffer = std::make_shared<Buffer>(refDevice, sizeof(uint32_t) * tileCount * 4, VK_BUFFER_USAGE_STORAGE_BUFFER_BIT, VmaMemoryUsage::VMA_MEMORY_USAGE_GPU_ONLY);
        postProcessVB = std::make_shared<Buffer>(device, sizeof(float) * 9, VK_BUFFER_USAGE_VERTEX_BUFFER_BIT, VmaMemoryUsage::VMA_MEMORY_USAGE_CPU_TO_GPU);

        float fullScreenTriangle[] = {-1.0, -1.0, 0.0, -1.0, 3.0, 0.0, 3.0, -1.0, 0.0};
        postProcessVB->update(&fullScreenTriangle, sizeof(float) * 9);
    }
}