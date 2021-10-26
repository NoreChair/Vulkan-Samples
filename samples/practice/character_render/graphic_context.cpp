#include "graphic_context.h"

namespace GraphicContext {

    const float k_fullScreenTriangle[9] = {-1.0, -1.0, 0.0, -1.0, 3.0, 0.0, 3.0, -1.0, 0.0};

    std::unique_ptr<vkb::RenderTarget> g_shadowImage{nullptr};

    std::unique_ptr<vkb::core::Image> g_sceneDepth{nullptr};
    std::unique_ptr<vkb::core::Image> g_sceneColorMS{nullptr};
     std::unique_ptr<vkb::core::Image> g_sceneDepthMS{nullptr};
    std::unique_ptr<vkb::core::Image> g_characterSSS{nullptr};
    std::unique_ptr<vkb::core::Image> g_linearDepth{nullptr};
    std::unique_ptr<vkb::core::Image> g_characterDepthStencil{nullptr};

    std::unique_ptr<vkb::core::Image> g_transientBlurH{nullptr};
    std::unique_ptr<vkb::core::Image> g_transientBlurV{nullptr};

    std::unique_ptr<vkb::core::ImageView> g_sceneDepthView{nullptr};
    std::unique_ptr<vkb::core::ImageView> g_sceneColorMSView{nullptr};
    std::unique_ptr<vkb::core::ImageView> g_sceneDepthMSView{nullptr};
    std::unique_ptr<vkb::core::ImageView> g_characterSSSView{nullptr};
    std::unique_ptr<vkb::core::ImageView> g_characterDepthStencilView{nullptr};

    std::unique_ptr<vkb::core::ImageView> g_linearDepthView{nullptr};
    std::unique_ptr<vkb::core::ImageView> g_transientBlurHView{nullptr};
    std::unique_ptr<vkb::core::ImageView> g_transientBlurVView{nullptr};

    std::unique_ptr<vkb::core::Sampler> g_linearClampSampler{nullptr};
    std::unique_ptr<vkb::core::Sampler> g_pointClampSampler{nullptr};
    std::unique_ptr<vkb::core::Sampler> g_shadowSampler{nullptr};
    
    std::unique_ptr<vkb::core::Buffer> g_fullScreenTriangle;

    void InitGraphicBuffer(vkb::Device& device, uint32_t width, uint32_t height) {
        VkExtent3D fullSize{width, height, 1};
        VkExtent3D halfSize{width >> 1 , height >> 1, 1};
        VkExtent3D shadowSize{512, 512, 1};

        VkFormat depthStencilFormat = VK_FORMAT_D24_UNORM_S8_UINT;
        VkFormat colorFormat = VK_FORMAT_R8G8B8A8_SRGB;

        VkImageUsageFlags outputAndShaderFlag = VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_SAMPLED_BIT;
        VkImageUsageFlags storageFlag = VK_IMAGE_USAGE_STORAGE_BIT;
        VkImageUsageFlags transientDepthFlag = VK_IMAGE_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT | VK_IMAGE_USAGE_TRANSIENT_ATTACHMENT_BIT;
        VkImageUsageFlags transientDepthInOutFlag = transientDepthFlag | VK_IMAGE_USAGE_INPUT_ATTACHMENT_BIT;
        VkImageUsageFlags transientColorFlag = VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_TRANSIENT_ATTACHMENT_BIT;
        VkImageUsageFlags shadowMapFlag = VK_IMAGE_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT | VK_IMAGE_USAGE_SAMPLED_BIT;

        VmaMemoryUsage transientMemory = VMA_MEMORY_USAGE_GPU_LAZILY_ALLOCATED;
#ifndef VK_USE_PLATFORM_ANDROID_KHR
        transientMemory = VMA_MEMORY_USAGE_GPU_ONLY;
#endif

        if (RenderSetting::g_multiSampleCount != VK_SAMPLE_COUNT_1_BIT) {
            g_sceneColorMS = std::make_unique<vkb::core::Image>(device, fullSize, colorFormat, transientColorFlag, transientMemory, RenderSetting::g_multiSampleCount);
            g_sceneColorMSView = std::make_unique<vkb::core::ImageView>(*g_sceneColorMS, VK_IMAGE_VIEW_TYPE_2D);

            g_sceneDepthMS = std::make_unique<vkb::core::Image>(device, fullSize, depthStencilFormat, transientDepthFlag, transientMemory, RenderSetting::g_multiSampleCount);
            g_sceneDepthMSView = std::make_unique<vkb::core::ImageView>(*g_sceneDepthMS, VK_IMAGE_VIEW_TYPE_2D);
        } else {
            g_sceneDepth = std::make_unique<vkb::core::Image>(device, fullSize, depthStencilFormat, transientDepthFlag, transientMemory);
            g_sceneDepthView = std::make_unique<vkb::core::ImageView>(*g_sceneDepth, VK_IMAGE_VIEW_TYPE_2D);
        }

        std::vector<vkb::core::Image> imgs; 
        imgs.clear();
        vkb::core::Image shadowImage(device, shadowSize, VK_FORMAT_D32_SFLOAT, shadowMapFlag, VmaMemoryUsage::VMA_MEMORY_USAGE_GPU_ONLY);
        imgs.push_back(std::move(shadowImage));
        g_shadowImage = std::make_unique<vkb::RenderTarget>(std::move(imgs));

        g_characterSSS = std::make_unique<vkb::core::Image>(device, halfSize, colorFormat, outputAndShaderFlag, VmaMemoryUsage::VMA_MEMORY_USAGE_GPU_ONLY);
        g_characterDepthStencil = std::make_unique<vkb::core::Image>(device, halfSize, depthStencilFormat, VK_IMAGE_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT, VmaMemoryUsage::VMA_MEMORY_USAGE_GPU_ONLY);
        g_linearDepth = std::make_unique<vkb::core::Image>(device, halfSize, VK_FORMAT_R32_SFLOAT, outputAndShaderFlag, VmaMemoryUsage::VMA_MEMORY_USAGE_GPU_ONLY);

        g_transientBlurH = std::make_unique<vkb::core::Image>(device, halfSize, colorFormat, outputAndShaderFlag, VmaMemoryUsage::VMA_MEMORY_USAGE_GPU_ONLY);
        g_transientBlurV = std::make_unique<vkb::core::Image>(device, halfSize, colorFormat, outputAndShaderFlag, VmaMemoryUsage::VMA_MEMORY_USAGE_GPU_ONLY);

        g_characterSSSView = std::make_unique<vkb::core::ImageView>(*g_characterSSS, VkImageViewType::VK_IMAGE_VIEW_TYPE_2D);
        g_characterDepthStencilView = std::make_unique<vkb::core::ImageView>(*g_characterDepthStencil, VkImageViewType::VK_IMAGE_VIEW_TYPE_2D);
        g_linearDepthView = std::make_unique<vkb::core::ImageView>(*g_linearDepth, VkImageViewType::VK_IMAGE_VIEW_TYPE_2D);

        g_transientBlurHView = std::make_unique<vkb::core::ImageView>(*g_transientBlurH, VkImageViewType::VK_IMAGE_VIEW_TYPE_2D);
        g_transientBlurVView = std::make_unique<vkb::core::ImageView>(*g_transientBlurV, VkImageViewType::VK_IMAGE_VIEW_TYPE_2D);

        auto samplerCreateInfo = VkSamplerCreateInfo{VK_STRUCTURE_TYPE_SAMPLER_CREATE_INFO};
        samplerCreateInfo.maxAnisotropy = 1.0f;
        samplerCreateInfo.magFilter = VK_FILTER_LINEAR;
        samplerCreateInfo.minFilter = VK_FILTER_LINEAR;
        samplerCreateInfo.mipmapMode = VK_SAMPLER_MIPMAP_MODE_LINEAR;
        samplerCreateInfo.addressModeU = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
        samplerCreateInfo.addressModeV = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
        samplerCreateInfo.addressModeW = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
        samplerCreateInfo.minLod = 0;
        samplerCreateInfo.maxLod = VK_LOD_CLAMP_NONE;
        g_linearClampSampler = std::make_unique<vkb::core::Sampler>(device, samplerCreateInfo);

        samplerCreateInfo.compareEnable = VK_TRUE;
        samplerCreateInfo.compareOp = VK_COMPARE_OP_LESS;
        g_shadowSampler = std::make_unique<vkb::core::Sampler>(device, samplerCreateInfo);

        samplerCreateInfo.compareEnable = VK_FALSE;
        samplerCreateInfo.compareOp = VK_COMPARE_OP_NEVER;
        samplerCreateInfo.magFilter = VK_FILTER_NEAREST;
        samplerCreateInfo.minFilter = VK_FILTER_NEAREST;
        g_pointClampSampler = std::make_unique<vkb::core::Sampler>(device, samplerCreateInfo);

        g_fullScreenTriangle = std::make_unique<vkb::core::Buffer>(device, sizeof(k_fullScreenTriangle), VK_BUFFER_USAGE_VERTEX_BUFFER_BIT, VmaMemoryUsage::VMA_MEMORY_USAGE_CPU_TO_GPU);
        g_fullScreenTriangle->update((void*)k_fullScreenTriangle, sizeof(k_fullScreenTriangle));  
    }

    void ResizeGraphicBuffer(vkb::Device& device, uint32_t width, uint32_t height) {
        DestroyGraphicBuffer();
        InitGraphicBuffer(device, width, height);
    }

    void DestroyGraphicBuffer() {
        g_shadowImage.reset();

        g_sceneDepthView.reset();
        g_sceneColorMSView.reset();
        g_sceneDepthMSView.reset();
        g_linearDepthView.reset();
        g_characterSSSView.reset();
        g_characterDepthStencilView.reset();
        g_transientBlurHView.reset();
        g_transientBlurVView.reset();

        g_sceneDepth.reset();
        g_sceneColorMS.reset();
        g_sceneDepthMS.reset();
        g_characterSSS.reset();
        g_characterDepthStencil.reset();
        g_linearDepth.reset();
        g_transientBlurH.reset();
        g_transientBlurV.reset();

        g_linearClampSampler.reset();
        g_pointClampSampler.reset();
        g_shadowSampler.reset();

        g_fullScreenTriangle.reset();
    }
}

namespace GraphicResources {
    const vkb::core::ImageView * g_sceneTextures[ETextureType::TextureCount];
    std::unordered_map<std::string, vkb::ShaderSource> g_shaderSources;
    std::unordered_map<std::string, vkb::ShaderModule*> g_shaderModules;
}

namespace RenderSetting {
    float g_shadowBias[3];
    float g_shadowNormalBias;
    float g_roughness;
    float g_metalness;
    float g_sssLevel;
    float g_sssCorrection;
    float g_sssMaxDD;
    bool g_onlySSS{false};
    bool g_onlyShadow{false};
    bool g_useScreenSpaceSSS{true};
    bool g_useColorBleedAO{true};
    bool g_useDoubleSpecular{false};
    bool g_useSSAO{false};
    bool g_depthResolveSupported{false};
    VkSampleCountFlagBits g_multiSampleCount{VK_SAMPLE_COUNT_1_BIT};
    VkResolveModeFlagBits g_depthResolveMode{VK_RESOLVE_MODE_NONE};

    void InitRenderSetting() {
        g_shadowBias[0] = 0.0f; g_shadowBias[1] = 0.0f; g_shadowBias[2] = 3.0f;
        g_shadowNormalBias = 0.2f;
        g_roughness = 0.45f;
        g_metalness = 0.0f;
        g_sssLevel = 400.0f;
        g_sssCorrection = 800.0f;
        g_sssMaxDD = 0.001f;
    }
}