#include "graphic_context.h"

namespace GraphicContext {
    std::unique_ptr<vkb::RenderTarget> g_offScreenRT{nullptr};

    std::unique_ptr<vkb::core::Image> g_characterSSS{nullptr};
    std::unique_ptr<vkb::core::Image> g_characterDepthStencil{nullptr};
    std::unique_ptr<vkb::core::Image> g_shadowImage{nullptr};

    std::unique_ptr<vkb::core::Image> g_linearDepth{nullptr};
    std::unique_ptr<vkb::core::Image> g_transientBlurH{nullptr};
    std::unique_ptr<vkb::core::Image> g_transientBlurV{nullptr};

    std::unique_ptr<vkb::core::ImageView> g_characterSSSView{nullptr};
    std::unique_ptr<vkb::core::ImageView> g_characterDepthStencilView{nullptr};
    std::unique_ptr<vkb::core::ImageView> g_shadowImageView{nullptr};

    std::unique_ptr<vkb::core::ImageView> g_linearDepthView{nullptr};
    std::unique_ptr<vkb::core::ImageView> g_transientBlurHView{nullptr};
    std::unique_ptr<vkb::core::ImageView> g_transientBlurVView{nullptr};

    std::unique_ptr<vkb::core::Sampler> g_linearClampSampler{nullptr};
    std::unique_ptr<vkb::core::Sampler> g_pointClampSampler{nullptr};

    void InitGraphicBuffer(vkb::Device& device, uint32_t width, uint32_t height) {
        VkExtent3D fullSize{width, height, 1};
        VkExtent3D halfSize{width >> 1 , height >> 1, 1};
        VkExtent3D shadowSize{1024, 1024, 1};

        VkFormat depthStencilFormat = VK_FORMAT_D24_UNORM_S8_UINT;
        VkFormat colorFormat = VK_FORMAT_R8G8B8A8_SRGB;

        VkImageUsageFlags outputAndShaderFlag = VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_SAMPLED_BIT;
        VkImageUsageFlags storageFlag = VK_IMAGE_USAGE_STORAGE_BIT;
        VkImageUsageFlags transientDepthFlag = VK_IMAGE_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT | VK_IMAGE_USAGE_TRANSIENT_ATTACHMENT_BIT;
        VkImageUsageFlags transientDepthInOutFlag = transientDepthFlag | VK_IMAGE_USAGE_INPUT_ATTACHMENT_BIT;
        VkImageUsageFlags shadowMapFlag = VK_IMAGE_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT | VK_IMAGE_USAGE_SAMPLED_BIT;

        VmaMemoryUsage transientMemory = VMA_MEMORY_USAGE_GPU_LAZILY_ALLOCATED;
#ifndef VK_USE_PLATFORM_ANDROID_KHR
        transientMemory = VMA_MEMORY_USAGE_GPU_ONLY;
#endif
        vkb::core::Image colorImage(device, fullSize, colorFormat, outputAndShaderFlag, VmaMemoryUsage::VMA_MEMORY_USAGE_GPU_ONLY);
        vkb::core::Image depthImage(device, fullSize, depthStencilFormat, transientDepthFlag, transientMemory);

        std::vector<vkb::core::Image> imgs; 
        imgs.emplace_back(std::move(colorImage));
        imgs.emplace_back(std::move(depthImage));
        g_offScreenRT = std::make_unique<vkb::RenderTarget>(std::move(imgs));

        g_characterSSS = std::make_unique<vkb::core::Image>(device, halfSize, colorFormat, outputAndShaderFlag | storageFlag, VmaMemoryUsage::VMA_MEMORY_USAGE_GPU_ONLY);
        g_characterDepthStencil = std::make_unique<vkb::core::Image>(device, halfSize, depthStencilFormat, transientDepthInOutFlag, transientMemory);
        g_shadowImage = std::make_unique<vkb::core::Image>(device, shadowSize, VK_FORMAT_D32_SFLOAT, shadowMapFlag, VmaMemoryUsage::VMA_MEMORY_USAGE_GPU_ONLY);

        g_linearDepth = std::make_unique<vkb::core::Image>(device, fullSize, VK_FORMAT_R16_SFLOAT, outputAndShaderFlag, VmaMemoryUsage::VMA_MEMORY_USAGE_GPU_ONLY);
        g_transientBlurH = std::make_unique<vkb::core::Image>(device, halfSize, colorFormat, storageFlag, VmaMemoryUsage::VMA_MEMORY_USAGE_GPU_ONLY);
        g_transientBlurV = std::make_unique<vkb::core::Image>(device, halfSize, colorFormat, storageFlag, VmaMemoryUsage::VMA_MEMORY_USAGE_GPU_ONLY);

        g_characterSSSView = std::make_unique<vkb::core::ImageView>(*g_linearDepth, VkImageViewType::VK_IMAGE_VIEW_TYPE_2D);
        g_characterDepthStencilView = std::make_unique<vkb::core::ImageView>(*g_characterDepthStencil, VkImageViewType::VK_IMAGE_VIEW_TYPE_2D);
        g_shadowImageView = std::make_unique<vkb::core::ImageView>(*g_shadowImage, VkImageViewType::VK_IMAGE_VIEW_TYPE_2D);

        g_linearDepthView = std::make_unique<vkb::core::ImageView>(*g_linearDepth, VkImageViewType::VK_IMAGE_VIEW_TYPE_2D);
        g_transientBlurHView = std::make_unique<vkb::core::ImageView>(*g_transientBlurH, VkImageViewType::VK_IMAGE_VIEW_TYPE_2D);
        g_transientBlurVView = std::make_unique<vkb::core::ImageView>(*g_transientBlurV, VkImageViewType::VK_IMAGE_VIEW_TYPE_2D);

        auto samplerCreateInfo = VkSamplerCreateInfo{VK_STRUCTURE_TYPE_SAMPLER_CREATE_INFO};
        samplerCreateInfo.maxAnisotropy = 1.0f;
        samplerCreateInfo.magFilter = VK_FILTER_LINEAR;
        samplerCreateInfo.minFilter = VK_FILTER_LINEAR;
        samplerCreateInfo.mipmapMode = VK_SAMPLER_MIPMAP_MODE_NEAREST;
        samplerCreateInfo.addressModeU = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
        samplerCreateInfo.addressModeV = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
        samplerCreateInfo.addressModeW = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
        samplerCreateInfo.minLod = 0;
        samplerCreateInfo.maxLod = VK_LOD_CLAMP_NONE;
        g_linearClampSampler = std::make_unique<vkb::core::Sampler>(device, samplerCreateInfo);

        samplerCreateInfo.magFilter = VK_FILTER_NEAREST;
        samplerCreateInfo.minFilter = VK_FILTER_NEAREST;
        g_pointClampSampler = std::make_unique<vkb::core::Sampler>(device, samplerCreateInfo);
    }

    void ResizeGraphicBuffer(vkb::Device& device, uint32_t width, uint32_t height) {
        DestroyGraphicBuffer();
        InitGraphicBuffer(device, width, height);
    }

    void DestroyGraphicBuffer() {
        g_offScreenRT.reset();
        g_linearDepthView.reset();
        g_transientBlurHView.reset();
        g_transientBlurVView.reset();

        g_characterSSSView.reset();
        g_characterDepthStencilView.reset();
        g_shadowImageView.reset();

        g_characterSSS.reset();
        g_characterDepthStencil.reset();
        g_shadowImage.reset();

        g_linearDepth.reset();
        g_transientBlurH.reset();
        g_transientBlurV.reset();

        g_linearClampSampler.reset();
        g_pointClampSampler.reset();
    }
}

namespace ShaderProgram {
    std::unordered_map<std::string, vkb::ShaderSource> g_shaderSources;
    std::unordered_map<std::string, vkb::ShaderModule*> g_shaderModules;
}