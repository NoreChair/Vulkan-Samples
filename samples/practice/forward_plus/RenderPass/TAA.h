#pragma once

#include "common/vk_common.h"
#include "platform/platform.h"
#include "ShaderProgram.h"

class TAA {
public:
    TAA(vkb::Device* d, vkb::RenderContext* c);
    ~TAA();

    void prepare();
    void set_up(vkb::core::ImageView* history, vkb::core::ImageView* velocity);
    glm::vec2 get_jitter(uint32_t frameIndex);
    void clear_history();
    void draw(bool enableTAA);

private:
    const static uint32_t k_jittlerSampleCount = 8;
    static bool           s_samplePointInited;
    static glm::vec2      s_hammersleyPoint[k_jittlerSampleCount];

    bool                  clear_history_flag{false};
    vkb::Device*          device{nullptr};
    vkb::RenderContext*   context{nullptr};
    vkb::core::ImageView* history_buffer{nullptr};
    vkb::core::ImageView* velocity_buffer{nullptr};
    vkb::core::ImageView* cur_depth_buffer{nullptr};
    vkb::core::ImageView* pre_depth_buffer{nullptr};
};
