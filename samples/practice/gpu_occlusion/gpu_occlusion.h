#pragma once
#include "common/vk_common.h"
#include "platform/platform.h"
#include "vulkan_sample.h"
#include "scene_graph/components/camera.h"

class gpu_occlusion : public vkb::VulkanSample {
    static const vkb::RenderTarget::CreateFunc swap_chain_create_func;
public:
    gpu_occlusion();
    virtual ~gpu_occlusion();

private:
    virtual bool prepare(vkb::Platform& platform) override;
    virtual void prepare_render_context() override;
    virtual void request_gpu_features(vkb::PhysicalDevice& gpu) override;
    virtual void resize(const uint32_t width, const uint32_t height) override;
    virtual void update(float delta_time) override;
    virtual void input_event(const vkb::InputEvent& input_event) override;
    virtual void draw_gui() override;
    virtual void finish() override;

    void prepare_resources();
    void prepare_scene();
    void render(float delta_time);

private:
    const std::string k_name = "GPU Occlusion";

    bool m_firstFrame{true};
    vkb::Timer m_timer;
    vkb::sg::SubMesh* m_unitCube{nullptr};
    vkb::sg::Camera* m_mainCamera{nullptr};
    VkFence m_ocFence[3];

    struct Profiler {
        bool drawAABB = false;
        int sceneDrawCount = 0;
        int frustumVisibleCount = 0;
        int occlusionTestCount = 0;
        int occlusionVisibleCount = 0;
        int actualDrawCount = 0;
        int frameCount = 0;
        float avgFrustumCullTime = 0.0f;
        float avgOcclusionCullTime = 0.0f;
        float avgRenderDrawTime = 0.0f;
        float avgOcclusionWaitTime = 0.0f;
        float frustumCullTime = 0.0f;
        float occlusionCullTime = 0.0f;
        float renderDrawTime = 0.0f;
        float occlusionWaitTime = 0.0f;

        void Profile() {
            if (frameCount >= 30) {
                avgFrustumCullTime = frustumCullTime / frameCount;
                avgOcclusionCullTime = occlusionCullTime / frameCount;
                avgRenderDrawTime = renderDrawTime / frameCount;
                avgOcclusionWaitTime = occlusionWaitTime / frameCount;
                frustumCullTime = 0.0f;
                occlusionCullTime = 0.0f;
                renderDrawTime = 0.0f;
                occlusionWaitTime = 0.0f;
                frameCount = 0;
            }
        }
    } m_profilerData;
};

std::unique_ptr<vkb::Application> create_gpu_occlusion();