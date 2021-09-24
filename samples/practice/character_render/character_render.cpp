#include "character_render.h"
#include "api_vulkan_sample.h"
#include "gltf_loader.h"

using namespace vkb;
using namespace vkb::core;

const RenderTarget::CreateFunc character_render::swap_chain_create_func = [](core::Image &&swapchain_image) -> std::unique_ptr<RenderTarget> {
    VkFormat                 depth_format = get_suitable_depth_format(swapchain_image.get_device().get_gpu().get_handle());
    std::vector<core::Image> images;
    images.push_back(std::move(swapchain_image));
    return std::make_unique<RenderTarget>(std::move(images));
};

 std::unique_ptr<vkb::Application> create_character_render() {
    return std::make_unique<character_render>();
}

 character_render::character_render() {}

 character_render::~character_render() {}

 bool character_render::prepare(vkb::Platform & platform) {
     return false;
 }

 void character_render::prepare_render_context() {}

 void character_render::request_gpu_features(vkb::PhysicalDevice & gpu) {}

 void character_render::resize(const uint32_t width, const uint32_t height) {}

 void character_render::update(float delta_time) {}

 void character_render::input_event(const vkb::InputEvent & input_event) {}

 void character_render::draw_gui() {}
