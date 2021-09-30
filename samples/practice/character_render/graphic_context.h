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

    extern std::unique_ptr<vkb::core::Image> g_characterSSS;
    extern std::unique_ptr<vkb::core::Image> g_characterDepthStencil;
    extern std::unique_ptr<vkb::core::Image> g_shadowImage;

    extern std::unique_ptr<vkb::core::Image> g_linearDepth;
    extern std::unique_ptr<vkb::core::Image> g_transientBlurH;
    extern std::unique_ptr<vkb::core::Image> g_transientBlurV;

    extern std::unique_ptr<vkb::core::ImageView> g_characterSSSView;
    extern std::unique_ptr<vkb::core::ImageView> g_characterDepthStencilView;
    extern std::unique_ptr<vkb::core::ImageView> g_shadowImageView;

    extern std::unique_ptr<vkb::core::ImageView> g_linearDepthView;
    extern std::unique_ptr<vkb::core::ImageView> g_transientBlurHView;
    extern std::unique_ptr<vkb::core::ImageView> g_transientBlurVView;

    // bilinear sample
    extern std::unique_ptr<vkb::core::Sampler> g_linearClampSampler;
    // nearest sample
    extern std::unique_ptr<vkb::core::Sampler> g_pointClampSampler;

    void InitGraphicBuffer(vkb::Device& device, uint32_t width, uint32_t height);
    void ResizeGraphicBuffer(vkb::Device& device, uint32_t width, uint32_t height);
    void DestroyGraphicBuffer();
}

namespace ShaderProgram {
    extern std::unordered_map<std::string, vkb::ShaderSource> g_shaderSources;
    extern std::unordered_map<std::string, vkb::ShaderModule*> g_shaderModules;
}