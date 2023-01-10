#include "GraphicContext.h"
#include "Utils.h"

using namespace vkb;
using namespace vkb::core;

namespace GraphicContext
{
#define DEFINE_IMAGE(x) \
    std::shared_ptr<vkb::core::Image> x{nullptr}; \
    std::shared_ptr<vkb::core::ImageView> x##View{nullptr};

#define DEFINE_IMAGE_ARRAY(x,c) \
    std::shared_ptr<vkb::core::Image> x[c]; \
    std::shared_ptr<vkb::core::ImageView> x##View[c];

#define RELEASE_IMAGE(x) \
    x.reset(); x##View.reset();

#define RELEASE_IMAGE_ARRAY(x,c)   \
    for (int i = 0; i < c; i++)    \
    {                              \
        x[i].reset(); x##View[i].reset(); \
    }

    DEFINE_IMAGE(hdrColorImage);
    DEFINE_IMAGE(sceneDepthImage);
    DEFINE_IMAGE(sceneNormalImage);
    DEFINE_IMAGE(sceneParamsImage);
    DEFINE_IMAGE(shadowImage);
    DEFINE_IMAGE_ARRAY(linearDepthImage, 2);
    DEFINE_IMAGE(screenShadowImage);
    DEFINE_IMAGE(lumaResultImage);
    DEFINE_IMAGE_ARRAY(temporalBlendImage, 2);
    DEFINE_IMAGE(velocityImage);
    DEFINE_IMAGE_ARRAY(bloomChainImage, 4);        // 2x/4x/8x/16x down sample
    DEFINE_IMAGE(rayIndexImage);
    DEFINE_IMAGE(reflectImage);

    std::shared_ptr<vkb::core::Buffer> lightBuffer{nullptr};
    std::shared_ptr<vkb::core::Buffer> lightGridBuffer{nullptr};
    std::shared_ptr<vkb::core::Buffer> postProcessVB{nullptr};
    std::shared_ptr<vkb::core::Buffer> exposureBuffer{nullptr};
    std::shared_ptr<vkb::core::Buffer> lumaHistogram{nullptr};
    std::shared_ptr<vkb::core::Buffer> raysBuffer{nullptr};
    std::shared_ptr<vkb::core::Buffer> indirectBuffer{nullptr};

    std::vector<std::shared_ptr<vkb::core::ImageView>> depthChainImageViews[2];

    std::shared_ptr<vkb::core::Sampler> linearClampSampler{nullptr};
    std::shared_ptr<vkb::core::Sampler> pointClampSampler{nullptr};

    void Init(vkb::Device &device, int width, int height)
    {
	    VkExtent3D extent{width, height, 1};
	    VkExtent3D shadowExtent{2048, 2048, 1};
        VkExtent3D quarter{width / 4, height / 4, 1};

        uint32_t mipmapCount = static_cast<uint32_t>(floor(log2(std::max(width, height))) + 1);

	    VkFormat hdrColorFormat = VK_FORMAT_R16G16B16A16_SFLOAT;
	    if (device.is_image_format_supported(VK_FORMAT_B10G11R11_UFLOAT_PACK32))
	    {
		    hdrColorFormat = VK_FORMAT_B10G11R11_UFLOAT_PACK32;
	    }

	    // Pack Velocity
	    assert(device.is_image_format_supported(VK_FORMAT_R32_UINT));
	    // Luma Histogram
	    assert(device.is_image_format_supported(VK_FORMAT_R8_UINT));
	    // Normal Buffer
	    assert(device.is_image_format_supported(VK_FORMAT_R16G16_SFLOAT));
        // PBR Params
        assert(device.is_image_format_supported(VK_FORMAT_R8G8_UNORM));

	    // Create Render Image
	    hdrColorImage    = std::make_shared<Image>(device, extent, hdrColorFormat, VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_SAMPLED_BIT | VK_IMAGE_USAGE_STORAGE_BIT, VmaMemoryUsage::VMA_MEMORY_USAGE_GPU_ONLY);
	    sceneDepthImage  = std::make_shared<Image>(device, extent, VK_FORMAT_D32_SFLOAT, VK_IMAGE_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT | VK_IMAGE_USAGE_SAMPLED_BIT | VK_IMAGE_USAGE_STORAGE_BIT, VmaMemoryUsage::VMA_MEMORY_USAGE_GPU_ONLY);
	    sceneNormalImage = std::make_shared<Image>(device, extent, VK_FORMAT_R16G16_SFLOAT, VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_STORAGE_BIT | VK_IMAGE_USAGE_SAMPLED_BIT, VmaMemoryUsage::VMA_MEMORY_USAGE_GPU_ONLY);
        sceneParamsImage = std::make_shared<Image>(device, extent, VK_FORMAT_R8G8_UNORM, VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_STORAGE_BIT | VK_IMAGE_USAGE_SAMPLED_BIT, VmaMemoryUsage::VMA_MEMORY_USAGE_GPU_ONLY);
	    shadowImage      = std::make_shared<Image>(device, shadowExtent, VK_FORMAT_D32_SFLOAT, VK_IMAGE_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT | VK_IMAGE_USAGE_SAMPLED_BIT | VK_IMAGE_USAGE_STORAGE_BIT, VmaMemoryUsage::VMA_MEMORY_USAGE_GPU_ONLY);

        rayIndexImage = std::make_shared<Image>(device, quarter, VK_FORMAT_R16_UINT, VK_IMAGE_USAGE_STORAGE_BIT | VK_IMAGE_USAGE_SAMPLED_BIT, VmaMemoryUsage::VMA_MEMORY_USAGE_GPU_ONLY);
        reflectImage = std::make_shared<Image>(device, quarter, VK_FORMAT_R16G16B16A16_SFLOAT, VK_IMAGE_USAGE_STORAGE_BIT | VK_IMAGE_USAGE_SAMPLED_BIT, VmaMemoryUsage::VMA_MEMORY_USAGE_GPU_ONLY);

	    hdrColorImageView    = std::make_shared<ImageView>(*hdrColorImage, VK_IMAGE_VIEW_TYPE_2D);
	    sceneDepthImageView  = std::make_shared<ImageView>(*sceneDepthImage, VK_IMAGE_VIEW_TYPE_2D);
	    sceneNormalImageView = std::make_shared<ImageView>(*sceneNormalImage, VK_IMAGE_VIEW_TYPE_2D);
        sceneParamsImageView = std::make_shared<ImageView>(*sceneParamsImage, VK_IMAGE_VIEW_TYPE_2D);
	    shadowImageView      = std::make_shared<ImageView>(*shadowImage, VK_IMAGE_VIEW_TYPE_2D);
        rayIndexImageView    = std::make_shared<ImageView>(*rayIndexImage, VK_IMAGE_VIEW_TYPE_2D);
        reflectImageView     = std::make_shared<ImageView>(*reflectImage, VK_IMAGE_VIEW_TYPE_2D);

	    for (int i = 0; i < 2; i++)
	    {
		    linearDepthImage[i]     = std::make_shared<Image>(device, extent, VK_FORMAT_R32_SFLOAT, VK_IMAGE_USAGE_STORAGE_BIT | VK_IMAGE_USAGE_SAMPLED_BIT, VmaMemoryUsage::VMA_MEMORY_USAGE_GPU_ONLY, VK_SAMPLE_COUNT_1_BIT, mipmapCount);
		    linearDepthImageView[i] = std::make_shared<ImageView>(*linearDepthImage[i], VK_IMAGE_VIEW_TYPE_2D);

            for (int j = 1; j < mipmapCount; j++)
            {
                auto view = std::make_shared<ImageView>(*linearDepthImage[i], VK_IMAGE_VIEW_TYPE_2D, VK_FORMAT_UNDEFINED, j, 0, 1, 0);
                depthChainImageViews[i].emplace_back(std::move(view));
            }
	    }

	    screenShadowImage     = std::make_shared<Image>(device, extent, VK_FORMAT_R8_UNORM, VK_IMAGE_USAGE_STORAGE_BIT | VK_IMAGE_USAGE_SAMPLED_BIT, VmaMemoryUsage::VMA_MEMORY_USAGE_GPU_ONLY);
	    screenShadowImageView = std::make_shared<ImageView>(*screenShadowImage, VK_IMAGE_VIEW_TYPE_2D);

	    lumaResultImage     = std::make_shared<Image>(device, extent, VK_FORMAT_R8_UINT, VK_IMAGE_USAGE_STORAGE_BIT | VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_SAMPLED_BIT, VmaMemoryUsage::VMA_MEMORY_USAGE_GPU_ONLY);
	    lumaResultImageView = std::make_shared<ImageView>(*lumaResultImage, VK_IMAGE_VIEW_TYPE_2D);

	    velocityImage     = std::make_shared<Image>(device, extent, VK_FORMAT_R32_UINT, VK_IMAGE_USAGE_SAMPLED_BIT | VK_IMAGE_USAGE_STORAGE_BIT, VmaMemoryUsage::VMA_MEMORY_USAGE_GPU_ONLY);
	    velocityImageView = std::make_shared<ImageView>(*velocityImage, VK_IMAGE_VIEW_TYPE_2D);
	    for (int i = 0; i < 2; i++)
	    {
		    temporalBlendImage[i]     = std::make_shared<Image>(device, extent, hdrColorFormat, VK_IMAGE_USAGE_SAMPLED_BIT | VK_IMAGE_USAGE_STORAGE_BIT, VmaMemoryUsage::VMA_MEMORY_USAGE_GPU_ONLY);
		    temporalBlendImageView[i] = std::make_shared<ImageView>(*temporalBlendImage[i], VK_IMAGE_VIEW_TYPE_2D);
	    }

	    for (int i = 0; i < 4; i++)
	    {
		    VkExtent3D bloomSize{extent.width >> (i + 1), extent.height >> (i + 1), 1};
		    bloomChainImage[i]     = std::make_shared<Image>(device, bloomSize, hdrColorFormat, VK_IMAGE_USAGE_SAMPLED_BIT | VK_IMAGE_USAGE_STORAGE_BIT, VmaMemoryUsage::VMA_MEMORY_USAGE_GPU_ONLY);
		    bloomChainImageView[i] = std::make_shared<ImageView>(*bloomChainImage[i], VK_IMAGE_VIEW_TYPE_2D);
	    }

	    int tileCount = (int) (glm::ceil(height / 16.0f) * glm::ceil(width / 16.0f));

	    lightBuffer     = std::make_shared<Buffer>(device, sizeof(LightBuffer) * MAX_LIGHTS_COUNT, VK_BUFFER_USAGE_STORAGE_BUFFER_BIT, VmaMemoryUsage::VMA_MEMORY_USAGE_CPU_TO_GPU);
	    lightGridBuffer = std::make_shared<Buffer>(device, sizeof(uint32_t) * (MAX_LIGHTS_COUNT + 4) * tileCount, VK_BUFFER_USAGE_STORAGE_BUFFER_BIT, VmaMemoryUsage::VMA_MEMORY_USAGE_GPU_ONLY);

        postProcessVB = std::make_shared<Buffer>(device, sizeof(float) * 9, VK_BUFFER_USAGE_VERTEX_BUFFER_BIT, VmaMemoryUsage::VMA_MEMORY_USAGE_CPU_TO_GPU);

	    float exposureParams[8] = {1.0f, 1.0f, 1.0f, 0.0f, -8.0f, 8.0f, 16.0f, 1.0f / 16.0f};

	    exposureBuffer = std::make_shared<Buffer>(device, sizeof(float) * 8, VK_BUFFER_USAGE_STORAGE_BUFFER_BIT, VmaMemoryUsage::VMA_MEMORY_USAGE_CPU_TO_GPU);
	    exposureBuffer->update((void *) exposureParams, sizeof(exposureParams));
	    lumaHistogram = std::make_shared<Buffer>(device, sizeof(uint32_t) * 256, VK_BUFFER_USAGE_STORAGE_BUFFER_BIT, VmaMemoryUsage::VMA_MEMORY_USAGE_GPU_ONLY);

        raysBuffer = std::make_shared<Buffer>(device, 40 * quarter.width * quarter.height, VK_BUFFER_USAGE_STORAGE_BUFFER_BIT, VmaMemoryUsage::VMA_MEMORY_USAGE_GPU_ONLY);
        indirectBuffer = std::make_shared<Buffer>(device, 12, VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_INDIRECT_BUFFER_BIT, VmaMemoryUsage::VMA_MEMORY_USAGE_GPU_ONLY);

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

	    samplerInfo.magFilter = VK_FILTER_NEAREST;
	    samplerInfo.minFilter = VK_FILTER_NEAREST;
	    pointClampSampler     = std::make_unique<vkb::core::Sampler>(device, samplerInfo);
    }

    void Release()
    {
	    RELEASE_IMAGE(hdrColorImage);
	    RELEASE_IMAGE(sceneDepthImage);
        RELEASE_IMAGE(sceneNormalImage);
        RELEASE_IMAGE(sceneParamsImage);
	    RELEASE_IMAGE(shadowImage);
	    RELEASE_IMAGE_ARRAY(linearDepthImage, 2);
	    RELEASE_IMAGE(screenShadowImage);
	    RELEASE_IMAGE(lumaResultImage);
	    RELEASE_IMAGE_ARRAY(temporalBlendImage, 2);
        RELEASE_IMAGE_ARRAY(bloomChainImage, 4);
	    RELEASE_IMAGE(velocityImage);
        RELEASE_IMAGE(rayIndexImage);
        RELEASE_IMAGE(reflectImage);

	    lightBuffer.reset();
	    lightGridBuffer.reset();
	    postProcessVB.reset();
	    exposureBuffer.reset();
	    lumaHistogram.reset();
        raysBuffer.reset();
        indirectBuffer.reset();

        for (int i = 0; i < depthChainImageViews[0].size(); i++)
        {
            depthChainImageViews[0][i].reset();
        }
        depthChainImageViews[0].clear();

        for (int i = 0; i < depthChainImageViews[1].size(); i++)
        {
            depthChainImageViews[1][i].reset();
        }
        depthChainImageViews[1].clear();

	    linearClampSampler.reset();
	    pointClampSampler.reset();
    }
}        // namespace GraphicContext