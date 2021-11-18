#include "graphic_context.h"
#include "core/device.h"

namespace GraphicContext {
    std::unique_ptr<vkb::core::Image> g_sceneDepth{nullptr};
    std::unique_ptr<vkb::core::Image> g_visibleBuffer{nullptr};
    std::unique_ptr<vkb::core::ImageView> g_sceneDepthView{nullptr};
    std::unique_ptr<vkb::core::ImageView> g_visibleBufferView{nullptr};

    std::unique_ptr<vkb::core::Sampler> g_linearClampSampler{nullptr};

    std::unique_ptr<vkb::core::Buffer> g_visibleReadbackBuffer{nullptr};
    std::unique_ptr<uint32_t[]> g_visibleResultBuffer{nullptr};
    std::unique_ptr<vkb::QueryPool> g_queryPool[3]{nullptr, nullptr, nullptr};
    std::unordered_map<vkb::sg::Node*, uint32_t> g_visibleNodeMap[3];

    void InitAll(vkb::Device& device, uint32_t width, uint32_t height) {
        InitWithoutResolution(device);
        InitWithResolution(device, width, height);
        PrepareOCContext(device, width, height);
    }

    void InitWithResolution(vkb::Device& device, uint32_t width, uint32_t height) {
        VkExtent3D fullSize{width, height, 1u};
        VkExtent3D halfSize{width >> 1 , height >> 1, 1u};

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
    }

    void PrepareOCContext(vkb::Device & device, uint32_t width, uint32_t height) {
        VkExtent3D bufferSize{256, 128, 1};
        VkFormat depthFormat = VK_FORMAT_D32_SFLOAT;

        VmaMemoryUsage transientMemory = VMA_MEMORY_USAGE_GPU_ONLY;
#if defined(ANDROID) || defined(_ANDROID_)
        transientMemory = VMA_MEMORY_USAGE_GPU_LAZILY_ALLOCATED;
#endif
        g_visibleBuffer = std::make_unique<vkb::core::Image>(device, bufferSize, depthFormat, VK_IMAGE_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT | VK_IMAGE_USAGE_TRANSIENT_ATTACHMENT_BIT, transientMemory);
        g_visibleBufferView = std::make_unique<vkb::core::ImageView>(*g_visibleBuffer, VkImageViewType::VK_IMAGE_VIEW_TYPE_2D);

        VkQueryPoolCreateInfo createInfo{VK_STRUCTURE_TYPE_QUERY_POOL_CREATE_INFO};
        createInfo.queryType = VK_QUERY_TYPE_OCCLUSION;
        createInfo.queryCount = RenderSetting::g_maxVisibleQueryCount;

        g_queryPool[0] = std::make_unique<vkb::QueryPool>(device, createInfo);
        g_queryPool[1] = std::make_unique<vkb::QueryPool>(device, createInfo);
        g_queryPool[2] = std::make_unique<vkb::QueryPool>(device, createInfo);

        g_visibleResultBuffer = std::make_unique<uint32_t[]>(RenderSetting::g_maxVisibleQueryCount * 3);
        memset(g_visibleResultBuffer.get(), 255, sizeof(uint32_t) * RenderSetting::g_maxVisibleQueryCount * 3);

        g_visibleReadbackBuffer = std::make_unique<vkb::core::Buffer>(device, sizeof(uint32_t) * RenderSetting::g_maxVisibleQueryCount * 3, VK_BUFFER_USAGE_TRANSFER_DST_BIT, VmaMemoryUsage::VMA_MEMORY_USAGE_GPU_TO_CPU);
    }

    void Resize(vkb::Device& device, uint32_t width, uint32_t height) {
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
    }

    void ReleaseWithoutResolution() {
        g_visibleBufferView.reset();
        g_visibleBuffer.reset();

        g_linearClampSampler.reset();

        g_visibleResultBuffer.reset();
        g_queryPool[0].reset();
        g_queryPool[1].reset();
        g_queryPool[2].reset();
        g_visibleReadbackBuffer.reset();
    }
}

namespace GraphicResources {
    std::unordered_map<std::string, vkb::ShaderModule*> g_shaderModules;
}

namespace RenderSetting {
    QueryMode g_queryMode = QueryMode::Delay;
    ProxyMode g_proxyMode = ProxyMode::Full;
    uint32_t g_maxVisibleQueryCount = 1024;
    uint32_t g_delayFrame = 1;
}