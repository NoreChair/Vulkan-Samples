#pragma once
#include "core/image.h"
#include "core/image_view.h"
#include "core/sampler.h"
#include "core/buffer.h"
#include "core/shader_module.h"
#include "core/query_pool.h"

namespace GraphicContext {
    extern std::unique_ptr<vkb::core::Image> g_sceneDepth;
    extern std::unique_ptr<vkb::core::Image> g_visibleBuffer;
    extern std::unique_ptr<vkb::core::ImageView> g_sceneDepthView;
    extern std::unique_ptr<vkb::core::ImageView> g_visibleBufferView;

    extern std::unique_ptr<vkb::core::Buffer> g_conditionRenderBuffer;

    extern std::unique_ptr<vkb::core::Sampler> g_linearClampSampler;

    extern std::unique_ptr<vkb::QueryPool> g_queryPool[3];

    extern std::unique_ptr<uint32_t[]> g_visibleResultBuffer;

    void InitAll(vkb::Device& device, uint32_t width, uint32_t height);
    void InitWithResolution(vkb::Device& device, uint32_t width, uint32_t height);
    void InitWithoutResolution(vkb::Device& device);
    void PrepareOCContext(vkb::Device& device, uint32_t width, uint32_t height);
    void Resize(vkb::Device& device, uint32_t width, uint32_t height);
    void ReleaseAll();
    void ReleaseWithResolution();
    void ReleaseWithoutResolution();
}

namespace GraphicResources {
    struct ShaderProgram {
        std::string vert;
        std::string frag;
    };

    extern std::unordered_map<std::string, vkb::ShaderModule*> g_shaderModules;
}

namespace RenderSetting {
    enum QueryMode :int {
        None = 0,
        Immediateness = 1,
    };

    enum ProxyMode :int {
        Full = 0,
        AABB = 1
    };

    extern bool g_conditionRender;
    extern bool g_waitAllResult;
    extern QueryMode g_queryMode;
    extern ProxyMode g_proxyMode;
    extern uint32_t g_maxVisibleQueryCount;
}