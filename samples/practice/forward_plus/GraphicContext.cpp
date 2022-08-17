#include "GraphicContext.h"
#include "Utils.h"

using namespace vkb;
using namespace vkb::core;

namespace GraphicContext
{
    //std::shared_ptr<vkb::core::Image>     sdrColorImage{nullptr};
    std::shared_ptr<vkb::core::Image> hdrColorImage{nullptr};
    std::shared_ptr<vkb::core::Image> sceneDepthImage{nullptr};
    std::shared_ptr<vkb::core::Image> shadowImage{nullptr};
    std::shared_ptr<vkb::core::Image> linearDepthImage[2]{nullptr, nullptr};
    std::shared_ptr<vkb::core::Image> screenShadowImage{nullptr};
    std::shared_ptr<vkb::core::Image> lumaResultImage{nullptr};
    std::shared_ptr<vkb::core::Image> temporalBlendImage[2]{nullptr, nullptr};
    std::shared_ptr<vkb::core::Image> velocityImage{nullptr};
    std::shared_ptr<vkb::core::Image> bloomChainImage[4]{nullptr, nullptr, nullptr, nullptr}; // 2x/4x/8x/16x down sample


    std::shared_ptr<vkb::core::Buffer> lightBuffer{nullptr};
    std::shared_ptr<vkb::core::Buffer> lightGridBuffer{nullptr};
    //std::shared_ptr<vkb::core::Buffer>    lightMaskBuffer{nullptr};
    std::shared_ptr<vkb::core::Buffer> postProcessVB{nullptr};
    std::shared_ptr<vkb::core::Buffer> exposureBuffer{nullptr};
    std::shared_ptr<vkb::core::Buffer> lumaHistogram{nullptr};

    //std::shared_ptr<vkb::core::ImageView> sdrColorImageView{nullptr};
    std::shared_ptr<vkb::core::ImageView> hdrColorImageView{nullptr};
    std::shared_ptr<vkb::core::ImageView> sceneDepthImageView{nullptr};
    std::shared_ptr<vkb::core::ImageView> shadowImageView{nullptr};
    std::shared_ptr<vkb::core::ImageView> linearDepthImageView[2]{nullptr, nullptr};
    std::shared_ptr<vkb::core::ImageView> screenShadowImageView{nullptr};
    std::shared_ptr<vkb::core::ImageView> lumaResultImageView{nullptr};
    std::shared_ptr<vkb::core::ImageView> temporalBlendImageView[2]{nullptr, nullptr};
    std::shared_ptr<vkb::core::ImageView> velocityImageView{nullptr};
    std::shared_ptr<vkb::core::ImageView> bloomChainImageView[4]{nullptr, nullptr, nullptr, nullptr};

    std::shared_ptr<vkb::core::Sampler> linearClampSampler{nullptr};
    std::shared_ptr<vkb::core::Sampler> pointClampSampler{nullptr};

    void Init(vkb::Device &device, int width, int height)
    {
	    VkExtent3D extent{width, height, 1};
	    VkExtent3D shadowExtent{2048, 2048, 1};

	    VkFormat hdrColorFormat = VK_FORMAT_R16G16B16A16_SFLOAT;

	    assert(device.is_image_format_supported(hdrColorFormat));
	    assert(device.is_image_format_supported(VK_FORMAT_R32_UINT));
        assert(device.is_image_format_supported(VK_FORMAT_R8_UINT));
	    // Create Render Image

	    //sdrColorImageView = std::make_shared<ImageView>(*sdrColorImage, VK_IMAGE_VIEW_TYPE_2D);
	    // sdrColorImage = std::make_shared<Image>(device, extent, VK_FORMAT_R8G8B8A8_SRGB, VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_STORAGE_BIT | VK_IMAGE_USAGE_TRANSFER_SRC_BIT,VmaMemoryUsage::VMA_MEMORY_USAGE_GPU_ONLY);

	    hdrColorImage   = std::make_shared<Image>(device, extent, hdrColorFormat, VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_SAMPLED_BIT | VK_IMAGE_USAGE_STORAGE_BIT, VmaMemoryUsage::VMA_MEMORY_USAGE_GPU_ONLY);
	    sceneDepthImage = std::make_shared<Image>(device, extent, VK_FORMAT_D32_SFLOAT, VK_IMAGE_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT | VK_IMAGE_USAGE_SAMPLED_BIT | VK_IMAGE_USAGE_STORAGE_BIT, VmaMemoryUsage::VMA_MEMORY_USAGE_GPU_ONLY);
	    shadowImage     = std::make_shared<Image>(device, shadowExtent, VK_FORMAT_D32_SFLOAT, VK_IMAGE_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT | VK_IMAGE_USAGE_SAMPLED_BIT | VK_IMAGE_USAGE_STORAGE_BIT, VmaMemoryUsage::VMA_MEMORY_USAGE_GPU_ONLY);

	    hdrColorImageView   = std::make_shared<ImageView>(*hdrColorImage, VK_IMAGE_VIEW_TYPE_2D);
	    sceneDepthImageView = std::make_shared<ImageView>(*sceneDepthImage, VK_IMAGE_VIEW_TYPE_2D);
	    shadowImageView     = std::make_shared<ImageView>(*shadowImage, VK_IMAGE_VIEW_TYPE_2D);

        for (int i = 0; i < 2; i++)
        {
            linearDepthImage[i] = std::make_shared<Image>(device, extent, VK_FORMAT_R32_SFLOAT, VK_IMAGE_USAGE_STORAGE_BIT | VK_IMAGE_USAGE_SAMPLED_BIT, VmaMemoryUsage::VMA_MEMORY_USAGE_GPU_ONLY);
            linearDepthImageView[i] = std::make_shared<ImageView>(*linearDepthImage[i], VK_IMAGE_VIEW_TYPE_2D);
        }

	    screenShadowImage     = std::make_shared<Image>(device, extent, VK_FORMAT_R8_UNORM, VK_IMAGE_USAGE_STORAGE_BIT | VK_IMAGE_USAGE_SAMPLED_BIT, VmaMemoryUsage::VMA_MEMORY_USAGE_GPU_ONLY);
	    screenShadowImageView = std::make_shared<ImageView>(*screenShadowImage, VK_IMAGE_VIEW_TYPE_2D);

	    lumaResultImage     = std::make_shared<Image>(device, extent, VK_FORMAT_R8_UINT, VK_IMAGE_USAGE_STORAGE_BIT | VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_SAMPLED_BIT, VmaMemoryUsage::VMA_MEMORY_USAGE_GPU_ONLY);
	    lumaResultImageView = std::make_shared<ImageView>(*lumaResultImage, VK_IMAGE_VIEW_TYPE_2D);

	    velocityImage          = std::make_shared<Image>(device, extent, VK_FORMAT_R32_UINT, VK_IMAGE_USAGE_SAMPLED_BIT | VK_IMAGE_USAGE_STORAGE_BIT, VmaMemoryUsage::VMA_MEMORY_USAGE_GPU_ONLY);
	    velocityImageView      = std::make_shared<ImageView>(*velocityImage, VK_IMAGE_VIEW_TYPE_2D);
	    for (int i = 0; i < 2; i++)
	    {
		    temporalBlendImage[i]     = std::make_shared<Image>(device, extent, hdrColorFormat, VK_IMAGE_USAGE_SAMPLED_BIT | VK_IMAGE_USAGE_STORAGE_BIT, VmaMemoryUsage::VMA_MEMORY_USAGE_GPU_ONLY);
		    temporalBlendImageView[i] = std::make_shared<ImageView>(*temporalBlendImage[i], VK_IMAGE_VIEW_TYPE_2D);
	    }

        for (int i = 0; i < 4; i++)
        {
            VkExtent3D bloomSize{extent.width >> (i + 1), extent.height >> (i + 1), 1};
            bloomChainImage[i] = std::make_shared<Image>(device, bloomSize, hdrColorFormat, VK_IMAGE_USAGE_SAMPLED_BIT | VK_IMAGE_USAGE_STORAGE_BIT, VmaMemoryUsage::VMA_MEMORY_USAGE_GPU_ONLY);
            bloomChainImageView[i] = std::make_shared<ImageView>(*bloomChainImage[i], VK_IMAGE_VIEW_TYPE_2D);
        }

	    int tileCount = (int) (glm::ceil(height / 16.0f) * glm::ceil(width / 16.0f));

	    lightBuffer     = std::make_shared<Buffer>(device, sizeof(LightBuffer) * MAX_LIGHTS_COUNT, VK_BUFFER_USAGE_STORAGE_BUFFER_BIT, VmaMemoryUsage::VMA_MEMORY_USAGE_CPU_TO_GPU);
	    lightGridBuffer = std::make_shared<Buffer>(device, sizeof(uint32_t) * (MAX_LIGHTS_COUNT + 4) * tileCount, VK_BUFFER_USAGE_STORAGE_BUFFER_BIT, VmaMemoryUsage::VMA_MEMORY_USAGE_GPU_ONLY);
	    //lightMaskBuffer = std::make_shared<Buffer>(refDevice, sizeof(uint32_t) * tileCount * 4, VK_BUFFER_USAGE_STORAGE_BUFFER_BIT, VmaMemoryUsage::VMA_MEMORY_USAGE_GPU_ONLY);
	    postProcessVB = std::make_shared<Buffer>(device, sizeof(float) * 9, VK_BUFFER_USAGE_VERTEX_BUFFER_BIT, VmaMemoryUsage::VMA_MEMORY_USAGE_CPU_TO_GPU);

	    float exposureParams[8] = {1.0f, 1.0f, 1.0f, 0.0f, -8.0f, 8.0f, 16.0f, 1.0f / 16.0f};

	    exposureBuffer = std::make_shared<Buffer>(device, sizeof(float) * 8, VK_BUFFER_USAGE_STORAGE_BUFFER_BIT, VmaMemoryUsage::VMA_MEMORY_USAGE_CPU_TO_GPU);
	    exposureBuffer->update((void *) exposureParams, sizeof(exposureParams));
	    lumaHistogram = std::make_shared<Buffer>(device, sizeof(uint32_t) * 256, VK_BUFFER_USAGE_STORAGE_BUFFER_BIT, VmaMemoryUsage::VMA_MEMORY_USAGE_GPU_ONLY);

	    float fullScreenTriangle[] = {-1.0, -1.0, 0.0, -1.0, 3.0, 0.0, 3.0, -1.0, 0.0};
	    postProcessVB->update(&fullScreenTriangle, sizeof(float) * 9);

	    VkSamplerCreateInfo samplerInfo{};
	    samplerInfo.magFilter    = VK_FILTER_LINEAR;
	    samplerInfo.minFilter    = VK_FILTER_LINEAR;
	    samplerInfo.mipmapMode   = VK_SAMPLER_MIPMAP_MODE_NEAREST;
	    samplerInfo.addressModeU = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
	    samplerInfo.addressModeV = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
	    samplerInfo.addressModeW = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
	    samplerInfo.minLod       = 0;
	    samplerInfo.maxLod       = VK_LOD_CLAMP_NONE;
	    linearClampSampler       = std::make_unique<vkb::core::Sampler>(device, samplerInfo);

        samplerInfo.magFilter    = VK_FILTER_NEAREST;
        samplerInfo.minFilter    = VK_FILTER_NEAREST;
        pointClampSampler        = std::make_unique<vkb::core::Sampler>(device, samplerInfo);
    }

    void Release()
    {
	    //sdrColorImage.reset();
	    hdrColorImage.reset();
	    sceneDepthImage.reset();
	    shadowImage.reset();
	    linearDepthImage[0].reset();
        linearDepthImage[1].reset();
	    screenShadowImage.reset();
	    lumaResultImage.reset();
	    temporalBlendImage[0].reset();
	    temporalBlendImage[1].reset();
	    velocityImage.reset();

        bloomChainImage[0].reset();
        bloomChainImage[1].reset();
        bloomChainImage[2].reset();
        bloomChainImage[3].reset();

	    lightBuffer.reset();
	    lightGridBuffer.reset();
	    //lightMaskBuffer.reset();
	    postProcessVB.reset();
	    exposureBuffer.reset();
	    lumaHistogram.reset();

	    //sdrColorImageView.reset();
	    hdrColorImageView.reset();
	    sceneDepthImageView.reset();
	    shadowImageView.reset();
	    linearDepthImageView[0].reset();
        linearDepthImageView[1].reset();
	    screenShadowImageView.reset();
	    lumaResultImageView.reset();
	    temporalBlendImageView[0].reset();
	    temporalBlendImageView[1].reset();
	    velocityImageView.reset();

        bloomChainImageView[0].reset();
        bloomChainImageView[1].reset();
        bloomChainImageView[2].reset();
        bloomChainImageView[3].reset();

	    linearClampSampler.reset();
        pointClampSampler.reset();
    }
}        // namespace GraphicContext