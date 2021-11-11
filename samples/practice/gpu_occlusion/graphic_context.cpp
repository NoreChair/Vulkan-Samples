#include "graphic_context.h"

namespace GraphicContext {
    std::unique_ptr<vkb::core::Image> g_sceneDepth{nullptr};
    std::unique_ptr<vkb::core::Image> g_visibleBuffer{nullptr};
    std::unique_ptr<vkb::core::ImageView> g_sceneDepthView{nullptr};
    std::unique_ptr<vkb::core::ImageView> g_visibleBufferView{nullptr};

    std::unique_ptr<vkb::core::Buffer> g_visibleResultBuffer{nullptr};
    std::unique_ptr<vkb::core::Buffer> g_cubeInstanceBuffer{nullptr};

    std::unique_ptr<vkb::core::Sampler> g_linearClampSampler{nullptr};

    void InitAll(vkb::Device& device, int width, int height) {
        InitWithoutResolution(device);
        InitWithResolution(device, width, height);
    }

    void InitWithResolution(vkb::Device& device, int width, int height) {
        VkExtent3D fullSize{width, height, 1};
        VkExtent3D halfSize{width >> 1 , height >> 1, 1};

        VkFormat depthFormat = VK_FORMAT_D32_SFLOAT;
        VkFormat colorFormat = VK_FORMAT_R8G8B8A8_SRGB;

        VkImageUsageFlags outputAndShaderFlag = VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_SAMPLED_BIT;
        VkImageUsageFlags storageFlag = VK_IMAGE_USAGE_STORAGE_BIT;
        VkImageUsageFlags transientDepthFlag = VK_IMAGE_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT | VK_IMAGE_USAGE_TRANSIENT_ATTACHMENT_BIT;
        VkImageUsageFlags transientDepthInOutFlag = transientDepthFlag | VK_IMAGE_USAGE_INPUT_ATTACHMENT_BIT;
        VkImageUsageFlags transientColorFlag = VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_TRANSIENT_ATTACHMENT_BIT;
        VkImageUsageFlags shadowMapFlag = VK_IMAGE_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT | VK_IMAGE_USAGE_SAMPLED_BIT;

        VmaMemoryUsage transientMemory = VMA_MEMORY_USAGE_GPU_ONLY;
#if defined(ANDROID) || defined(_ANDROID_)
        transientMemory = VMA_MEMORY_USAGE_GPU_LAZILY_ALLOCATED;
#endif

        g_sceneDepth = std::make_unique<vkb::core::Image>(device, fullSize, depthFormat, transientDepthFlag, transientMemory);
        g_sceneDepthView = std::make_unique<vkb::core::ImageView>(*g_sceneDepth, VkImageViewType::VK_IMAGE_VIEW_TYPE_2D);

        if (RenderSetting::g_queryMode == RenderSetting::Deferral) {
            g_visibleBuffer = std::make_unique<vkb::core::Image>(device, fullSize, depthFormat, VK_IMAGE_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT, VmaMemoryUsage::VMA_MEMORY_USAGE_GPU_ONLY);
            g_visibleBufferView = std::make_unique<vkb::core::ImageView>(*g_visibleBuffer, VkImageViewType::VK_IMAGE_VIEW_TYPE_2D);
        } else if(RenderSetting::g_queryMode == RenderSetting::Immediateness) {
            g_visibleBuffer = std::make_unique<vkb::core::Image>(device, halfSize, depthFormat, VK_IMAGE_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT,
                transientMemory);
        }
    }

    void InitWithoutResolution(vkb::Device& device) {
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

        if (RenderSetting::g_queryMode != RenderSetting::QueryMode::None) {
            VmaMemoryUsage usage = VmaMemoryUsage::VMA_MEMORY_USAGE_GPU_TO_CPU;
            if (RenderSetting::g_conditionRender) {
                usage = VmaMemoryUsage::VMA_MEMORY_USAGE_GPU_ONLY;
            }

            g_visibleResultBuffer = std::make_unique<vkb::core::Buffer>(device, RenderSetting::g_maxVisibleQueryCount * sizeof(uint32_t), VK_BUFFER_USAGE_TRANSFER_DST_BIT, usage);
        }
    }

    void Resize(vkb::Device& device, int width, int height) {
        ReleaseWithResolution();
        InitWithResolution(device, width, height);
    }

    void ReleaseAll() {
        ReleaseWithoutResolution();
        ReleaseWithResolution();
    }

    void ReleaseWithResolution() {
        g_sceneDepthView.reset();
        g_sceneDepth.reset();

        g_visibleBufferView.reset();
        g_visibleBuffer.reset();
    }

    void ReleaseWithoutResolution() {
        g_linearClampSampler.reset();

        g_visibleResultBuffer.reset();
        g_cubeInstanceBuffer.reset();
    }
}

namespace GraphicResources {
    std::unordered_map<std::string, vkb::ShaderModule*> g_shaderModules;
}

namespace RenderSetting {
    bool g_conditionRender = false;
    bool g_useLODQuery = false;
    QueryMode g_queryMode = QueryMode::None;
    bool g_useAABBQuery = false;
    uint32_t g_maxVisibleQueryCount = 512;
}