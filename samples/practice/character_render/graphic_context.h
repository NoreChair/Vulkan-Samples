#pragma once
#include <string>
#include "core/image.h"
#include "core/image_view.h"
#include "core/sampler.h"
#include "core/buffer.h"
#include "core/shader_module.h"
#include "rendering/render_target.h"

namespace GraphicContext {
    extern std::unique_ptr<vkb::RenderTarget> g_offScreenRT;
    extern std::unique_ptr<vkb::RenderTarget> g_shadowImage;

    extern std::unique_ptr<vkb::core::Image> g_characterSSS;
    extern std::unique_ptr<vkb::core::Image> g_linearDepth;
    extern std::unique_ptr<vkb::core::Image> g_characterDepthStencil;

    extern std::unique_ptr<vkb::core::Image> g_transientBlurH;
    extern std::unique_ptr<vkb::core::Image> g_transientBlurV;

    extern std::unique_ptr<vkb::core::ImageView> g_characterSSSView;
    extern std::unique_ptr<vkb::core::ImageView> g_characterDepthStencilView;

    extern std::unique_ptr<vkb::core::ImageView> g_linearDepthView;
    extern std::unique_ptr<vkb::core::ImageView> g_transientBlurHView;
    extern std::unique_ptr<vkb::core::ImageView> g_transientBlurVView;

    // bilinear sample
    extern std::unique_ptr<vkb::core::Sampler> g_linearClampSampler;
    // nearest sample
    extern std::unique_ptr<vkb::core::Sampler> g_pointClampSampler;
    extern std::unique_ptr<vkb::core::Sampler> g_shadowSampler;

    extern std::unique_ptr<vkb::core::Buffer> g_fullScreenTriangle;

    void InitGraphicBuffer(vkb::Device& device, uint32_t width, uint32_t height);
    void ResizeGraphicBuffer(vkb::Device& device, uint32_t width, uint32_t height);
    void DestroyGraphicBuffer();
}

namespace GraphicResources {
    enum ETextureType {
        Albedo,
        Normal,
        Occlusion,
        PBR,
        Detail,
        SkyBox,
        Irradiance,
        Radiance,
        TextureCount
    };

    extern const vkb::core::ImageView * g_sceneTextures[ETextureType::TextureCount];
    extern std::unordered_map<std::string, vkb::ShaderSource> g_shaderSources;
    extern std::unordered_map<std::string, vkb::ShaderModule*> g_shaderModules;
}

namespace RenderSetting {
    extern float g_shadowBias[3];
    extern float g_shadowNormalBias;
    extern float g_roughness;
    extern float g_metalness;
    extern float g_sssLevel;
    extern float g_sssCorrection;
    extern float g_sssMaxDD;
    extern bool g_onlySSS;
    extern bool g_onlyShadow;
    extern bool g_useScreenSpaceSSS;
    extern bool g_useColorBleedAO;
    extern bool g_useDoubleSpecular;
    extern bool g_useSSAO;

    void Init();
}