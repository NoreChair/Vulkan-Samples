#pragma once

#include "common/vk_common.h"
#include "platform/platform.h"
#include "vulkan_sample.h"

namespace character_context{
    vkb::RenderTarget g_sceneDepthStencil;
    vkb::RenderTarget g_characterRT;

    vkb::core::Image g_linearDepth;
    vkb::core::Image g_transientBlurH;
    vkb::core::Image g_transientBlurV;

    vkb::core::ImageView g_linearDepthView;
    vkb::core::ImageView g_transientBlurHView;
    vkb::core::ImageView g_transientBlurVView;
}

class character_render : public vkb::VulkanSample {
    static const vkb::RenderTarget::CreateFunc swap_chain_create_func;
public:
    character_render();
    virtual ~character_render();

private:
    virtual bool prepare(vkb::Platform &platform) override;
    virtual void prepare_render_context() override;
    virtual void request_gpu_features(vkb::PhysicalDevice &gpu) override;
    virtual void resize(const uint32_t width, const uint32_t height) override;
    virtual void update(float delta_time) override;
    virtual void input_event(const vkb::InputEvent &input_event) override;
    virtual void draw_gui();

    void prepare_shader();
    void prepare_scene();

private:
    const std::string k_title = "Vulkan Example";
    const std::string k_name = "Character Render";
};

std::unique_ptr<vkb::Application> create_character_render();