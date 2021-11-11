#pragma once
#include "core/image.h"
#include "core/image_view.h"
#include "core/sampler.h"
#include "core/buffer.h"
#include "core/shader_module.h"

namespace GraphicContext {
    extern std::unique_ptr<vkb::core::Image> g_sceneDepth;
    extern std::unique_ptr<vkb::core::Image> g_visibleBuffer;
    extern std::unique_ptr<vkb::core::ImageView> g_sceneDepthView;

    extern std::unique_ptr<vkb::core::Buffer> g_visibleResultBuffer;
    extern std::unique_ptr<vkb::core::Buffer> g_cubeInstanceBuffer;

    extern std::unique_ptr<vkb::core::Sampler> g_linearClampSampler;

    void InitAll(vkb::Device& device, int width, int height);
    void InitWithResolution(vkb::Device& device, int width, int height);
    void InitWithoutResolution(vkb::Device& device);
    void Resize(vkb::Device& device, int width, int height);
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
    enum QueryMode :uint32_t {
        None,
        Deferral,
        Immediateness
    };

    extern bool g_conditionRender;
    extern bool g_useLODQuery;
    extern bool g_useAABBQuery;
    extern QueryMode g_queryMode;
    extern uint32_t g_maxVisibleQueryCount;
}