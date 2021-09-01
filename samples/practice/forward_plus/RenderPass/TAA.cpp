#include "TAA.h"
#include "HammersleyHaltonPoints.h"
#include "ShaderProgram.h"

bool TAA::s_samplePointInited = false;
glm::vec2 TAA::s_hammersleyPoint[k_jittlerSampleCount] = {};

TAA::TAA(vkb::Device* d, vkb::RenderContext* c) {
    if (!s_samplePointInited) {
        s_samplePointInited = true;
        Hammersley((float*)s_hammersleyPoint, k_jittlerSampleCount);
    }

    device = d;
    context = c;
}

TAA::~TAA() {}

void TAA::prepare() {
    // Prepare shader, pipeline etc ...
}

void TAA::set_up(vkb::core::ImageView* history, vkb::core::ImageView* velocity) {
    history_buffer = history;
    velocity_buffer = velocity;
}


glm::vec2 TAA::get_jitter(uint32_t frameIndex) {
    uint32_t jitterSampleIndex = frameIndex % k_jittlerSampleCount;
    return s_hammersleyPoint[jitterSampleIndex];
}

void TAA::clear_history() {
    clear_history_flag = true;
}

void TAA::draw(bool enableTAA) {
    // TODO : 
    clear_history_flag = false;

}
