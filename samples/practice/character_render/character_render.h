#pragma once

#include "common/vk_common.h"
#include "platform/platform.h"
#include "vulkan_sample.h"
#include "shadow_camera.h"
#include "scene_graph/scripts/free_camera.h"

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
    virtual void draw_gui() override;
    virtual void finish() override;
    virtual const std::vector<const char *> get_validation_layers() override;

    void prepare_msaa_mode();
    void prepare_resources();
    void prepare_scene();
    void render(float delta_time);

private:
    const std::string k_title = "Vulkan Example";
    const std::string k_name = "Character Render";

    vkb::sg::SubMesh* m_unitCube{nullptr};
    vkb::sg::FreeCamera* m_freeCamera{nullptr};
    vkb::sg::Camera* m_mainCamera{nullptr};
    ShadowCamera* m_lightCamera{nullptr};
};

std::unique_ptr<vkb::Application> create_character_render();