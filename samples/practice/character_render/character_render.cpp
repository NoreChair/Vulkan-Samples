#include "character_render.h"
#include "api_vulkan_sample.h"
#include "render_utils.h"

#include "gltf_loader.h"
#include "ctpl_stl.h"

#include "common/utils.h"
#include "scene_graph/node.h"
#include "scene_graph/components/mesh.h"
#include "scene_graph/components/sub_mesh.h"
#include "scene_graph/components/material.h"
#include "scene_graph/components/image/astc.h"

#include "graphic_context.h"
#include "shadow_pass.h"
#include "ssao.h"
#include "main_pass.h"
#include "subsurface_pass.h"

#include <thread>
#include <algorithm>

using namespace vkb;
using namespace vkb::core;

const RenderTarget::CreateFunc character_render::swap_chain_create_func = [](Image &&swapchain_image) -> std::unique_ptr<RenderTarget> {
    const auto& physicalDevice = swapchain_image.get_device().get_gpu().get_handle();
    std::vector<Image> images;
    images.push_back(std::move(swapchain_image));
    return std::make_unique<RenderTarget>(std::move(images));
};

std::unique_ptr<vkb::Application> create_character_render() {
    return std::make_unique<character_render>();
}

character_render::character_render() {
    set_name(k_name);
    auto &config = get_configuration();

    config.insert<vkb::BoolSetting>(0, m_useScreenSpaceSSS, false);
    config.insert<vkb::BoolSetting>(0, m_useColorBleedAO, false);
    config.insert<vkb::BoolSetting>(0, m_useDoubleSpecular, false);
}

character_render::~character_render() {
}

bool character_render::prepare(vkb::Platform & platform) {
    if (!VulkanSample::prepare(platform)) {
        return false;
    }

    auto& extent = get_render_context().get_surface_extent();
    GraphicContext::InitGraphicBuffer(get_device(), extent.width, extent.height);

    prepare_resources();
    prepare_scene();

    // init render resource
    //ShadowPass::Init();
    //SubsurfacePass::Init();
    MainPass::Init(get_device());
    //SSAO::Init();

    gui = std::make_unique<vkb::Gui>(*this, platform.get_window(), stats.get());

    return true;
}

const std::vector<const char *> character_render::get_validation_layers() {
    //return {"VK_LAYER_LUNARG_standard_validation"};
    return {};
}

void character_render::prepare_resources() {
    auto& device = get_render_context().get_device();

    Timer timer;
    timer.start();

    auto thread_count = std::thread::hardware_concurrency();
    thread_count = thread_count == 0 ? 1 : thread_count - 1;
#ifndef LOAD_IN_MAIN_THREAD
    ctpl::thread_pool thread_pool(thread_count);
#else
    thread_count = 1;
#endif
    // load texture
    struct TextureInfo {
        std::string name;
        bool isColor;
    };
    std::vector<TextureInfo> textures{
        {"T_Color.png", true},
        {"T_N.png", false},
        {"T_AO_SSS_CA.png", false}
    };

    auto image_count = textures.size();
    std::vector<std::unique_ptr<sg::Image>> image_components;

#ifndef LOAD_IN_MAIN_THREAD
    std::vector<std::future<std::unique_ptr<sg::Image>>> image_component_futures;
    for (size_t image_index = 0; image_index < image_count; image_index++) {
        auto fut = thread_pool.push(
            [this, textures, image_index](size_t) {
            const std::string image_path = "character/";
            std::unique_ptr<sg::Image> image = sg::Image::load(textures[image_index].name, image_path + textures[image_index].name, textures[image_index].isColor);
            image->generate_mipmaps();
            image->create_vk_image(get_device());
            LOGI("Loadedimage #{} ({})", image_index, textures[image_index].name.c_str());
            return image;
        });

        image_component_futures.push_back(std::move(fut));
    }
#else
    const std::string image_path = "character/";
    for (size_t image_index = 0; image_index < image_count; image_index++) {
        std::unique_ptr<sg::Image> image = sg::Image::load(texture_names[image_index], image_path + texture_names[image_index]);
        LOGI("Loadedimage #{} ({})", image_index, texture_names[image_index].c_str());

        // Check whether the format is supported by the GPU
        if (sg::is_astc(image->get_format())) {
            if (!device.is_image_format_supported(image->get_format())) {
                LOGW("ASTC not supported: decoding {}", image->get_name());
                image = std::make_unique<sg::Astc>(*image);
            }
        }

        image->generate_mipmaps();
        image->create_vk_image(get_device());
        image_components.push_back(std::move(image));
    }
#endif
    // load shader
    struct GShader {
        std::string name;
        std::string define;
        VkShaderStageFlagBits flags;
    };

    std::vector<GShader> shaders{
        {"depth_only.vert", "", VK_SHADER_STAGE_VERTEX_BIT},
        {"depth_only.frag", "", VK_SHADER_STAGE_FRAGMENT_BIT},
        {"character.vert", "", VK_SHADER_STAGE_VERTEX_BIT},
        {"character.frag", "", VK_SHADER_STAGE_FRAGMENT_BIT},
    };

    static std::mutex shaderMutex;
    static std::condition_variable cv;
    static int shaderLoadedCount = 0;

    for (int i = 0; i < shaders.size(); ++i) {
#ifndef LOAD_IN_MAIN_THREAD
        thread_pool.push([this, i, shaders](int) {
#endif
            const std::string shader_path = "character/";
            ShaderSource source(shader_path + shaders[i].name);
            ShaderModule& moudle = get_device().get_resource_cache().request_shader_module(shaders[i].flags, source);
            {
                std::lock_guard<std::mutex> lock(shaderMutex);
                ShaderProgram::g_shaderSources.emplace(shaders[i].name, std::move(source));
                ShaderProgram::g_shaderModules.emplace(shaders[i].name, &moudle);
                shaderLoadedCount++;
            }
            cv.notify_one();
#ifndef LOAD_IN_MAIN_THREAD
        });
#endif
    }

    // load from gltf
    load_scene("character/Head.gltf");

    // Sync, update images
#ifndef LOAD_IN_MAIN_THREAD
    for (auto &fut : image_component_futures) {
        image_components.push_back(fut.get());
    }
#endif

    std::vector<uint8_t> white {255,255,255,255};
    std::vector<uint8_t> blue {0,0,255,255};
    std::unique_ptr<sg::Image> whiteImage = std::make_unique<sg::Image>("white", std::move(white), std::move(std::vector<sg::Mipmap>{sg::Mipmap{0,0,{1,1,1}}}));
    whiteImage->create_vk_image(device);
    std::unique_ptr<sg::Image> blueImage = std::make_unique<sg::Image>("blue", std::move(blue), std::move(std::vector<sg::Mipmap>{sg::Mipmap{0,0,{1,1,1}}}));
    blueImage->create_vk_image(device);
    image_components.push_back(std::move(whiteImage));
    image_components.push_back(std::move(blueImage));

    RenderUtils::UploadImage(get_device(), image_components);
    scene->set_components(std::move(image_components));

    // wait for all shader are loaded
    std::unique_lock<std::mutex> lock(shaderMutex);
    cv.wait(lock, [shaders] {return shaders.size() == shaderLoadedCount; });

    auto elapsed_time = timer.stop();
    LOGI("Time spent loading all resources: {} seconds across {} threads.", vkb::to_string(elapsed_time), thread_count);
}

void character_render::prepare_scene() {
    // create scene, camera, light
    auto  sceneAABB = std::make_unique<sg::AABB>();
    auto& meshs = scene->get_components<sg::Mesh>();

    for (size_t i = 0; i < meshs.size(); i++) {
        const auto &bounds = meshs[i]->get_bounds();
        for (auto &node : meshs[i]->get_nodes()) {
            auto     node_transform = node->get_transform().get_world_matrix();
            sg::AABB world_bounds{bounds.get_min(), bounds.get_max()};
            world_bounds.transform(node_transform);
            sceneAABB->update(world_bounds.get_min());
            sceneAABB->update(world_bounds.get_max());
        }
    }

    auto &camera_node = vkb::add_free_camera(*scene, "main_camera", get_render_context().get_surface_extent(), 0.5);
    m_freeCamera = (vkb::sg::FreeCamera*)&camera_node.get_component<vkb::sg::Script>();
    m_mainCamera = &camera_node.get_component<vkb::sg::Camera>();
    m_mainCamera->set_far_plane(10000.0f);
    m_mainCamera->set_near_plane(0.01f);
    m_mainCamera->get_node()->get_transform().set_translation({1.5, 5.78, -170.0});

    auto& lights = scene->get_components<sg::Light>();
    auto  iter = std::find_if(lights.begin(), lights.end(), [](sg::Light *iter) -> bool { return iter->get_light_type() == sg::LightType::Directional; });
    assert(iter != lights.end());

    // sun direction
    sg::Light* directLight = *iter;
    sg::LightProperties sunLight;
    sunLight.direction = glm::normalize(glm::vec3(-0.615, -0.064, 0.786));
    sunLight.intensity = 10.0f;
    directLight->set_properties(sunLight);

    // add shadow camera
    auto lightNode = directLight->get_node();
    auto shadowCamera = std::make_unique<ShadowCamera>("main_light_camera");
    shadowCamera->set_node(*lightNode);
    m_lightCamera = shadowCamera.get();
    glm::vec2 shadowMapExtent = glm::vec2(GraphicContext::g_shadowImage->get_extent().width, GraphicContext::g_shadowImage->get_extent().height);
    m_lightCamera->set_up(sunLight.direction, sceneAABB->get_center(), glm::vec3(500, 500, 500), shadowMapExtent, 32);
    lightNode->get_transform().set_translation(sceneAABB->get_center() - sunLight.direction * 500.0f);
    scene->add_component(std::move(shadowCamera), *lightNode);

    if (!scene->get_root_node().has_component<sg::AABB>()) {
        scene->add_component(std::move(sceneAABB), scene->get_root_node());
    }
}

void character_render::prepare_render_context() {
    auto &properties = render_context->get_swapchain().get_properties();
    properties.image_usage |= VK_IMAGE_USAGE_TRANSFER_DST_BIT;
    render_context->prepare(1, swap_chain_create_func);
}


void character_render::request_gpu_features(vkb::PhysicalDevice & gpu) {
    VkFormatFeatureFlags flags = gpu.get_format_properties(VK_FORMAT_R8G8B8A8_SRGB).optimalTilingFeatures;
    bool supportBlit = (flags & (VK_FORMAT_FEATURE_BLIT_SRC_BIT | VK_FORMAT_FEATURE_BLIT_DST_BIT)) == (VK_FORMAT_FEATURE_BLIT_SRC_BIT | VK_FORMAT_FEATURE_BLIT_DST_BIT);
    flags = gpu.get_format_properties(VK_FORMAT_B8G8R8A8_SRGB).optimalTilingFeatures;
    supportBlit = supportBlit && (flags & VK_FORMAT_FEATURE_BLIT_DST_BIT) != 0;
    assert(supportBlit);
}

void character_render::resize(const uint32_t width, const uint32_t height) {
    VulkanSample::resize(width, height);
    device->wait_idle();
    GraphicContext::ResizeGraphicBuffer(get_device(), width, height);
}

void character_render::update(float delta_time) {
    update_scene(delta_time);
    update_gui(delta_time);
    render(delta_time);
}

void character_render::render(float delta_time) {
    RenderUtils::SortedMeshes opaqueNodes;
    RenderUtils::SortedMeshes transparentNodes;
    RenderUtils::SortedMeshes shadowNodes;

    sg::Transform &mainCameraRTS = m_mainCamera->get_node()->get_transform();
    glm::vec3      cameraForward = glm::mat3(mainCameraRTS.get_world_matrix()) * glm::vec3(0.0, 0.0, 1.0);
    sg::Transform &lightCameraRTS = m_lightCamera->get_node()->get_transform();
    glm::vec3      lightForward = glm::mat3(lightCameraRTS.get_world_matrix()) * glm::vec3(0.0, 0.0, 1.0);

    RenderUtils::GetSortedNodes(scene.get(), cameraForward, mainCameraRTS.get_world_translation(), &opaqueNodes, &transparentNodes);
    RenderUtils::GetSortedNodes(scene.get(), lightForward, m_lightCamera->get_shadow_center(), &shadowNodes);

    // reverse z
    std::vector<VkClearValue> zeroClearValue{initializers::clear_color_value(0.0, 0.0, 0.0, 0.0), initializers::clear_depth_stencil_value(0.0, 0)};
    // forward z
    std::vector<VkClearValue> oneClearValue{initializers::clear_color_value(0.0, 0.0, 0.0, 0.0),initializers::clear_depth_stencil_value(1.0, 0)};
        
    VkExtent2D     extent = get_render_context().get_surface_extent();
    RenderContext &context = get_render_context();
    CommandBuffer &commandBuffer = context.begin();
    commandBuffer.begin(VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT);

    // image barrier to color output
    ImageMemoryBarrier barrier{};
    barrier.src_stage_mask = VK_PIPELINE_STAGE_BOTTOM_OF_PIPE_BIT;
    barrier.dst_stage_mask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
    barrier.src_access_mask = 0;
    barrier.dst_access_mask = VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT;
    barrier.old_layout = VK_IMAGE_LAYOUT_UNDEFINED;
    barrier.new_layout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
    commandBuffer.image_memory_barrier(GraphicContext::g_offScreenRT->get_views()[0], barrier);

    barrier.src_stage_mask = VK_PIPELINE_STAGE_BOTTOM_OF_PIPE_BIT;
    barrier.dst_stage_mask = VK_PIPELINE_STAGE_EARLY_FRAGMENT_TESTS_BIT | VK_PIPELINE_STAGE_LATE_FRAGMENT_TESTS_BIT;
    barrier.src_access_mask = 0;
    barrier.dst_access_mask = VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_READ_BIT | VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT;
    barrier.old_layout = VK_IMAGE_LAYOUT_UNDEFINED;
    barrier.new_layout = VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL;

    commandBuffer.image_memory_barrier(GraphicContext::g_offScreenRT->get_views()[1], barrier);

    {
        std::vector<LoadStoreInfo> loadStoreInfos;
        loadStoreInfos.emplace_back(LoadStoreInfo{VK_ATTACHMENT_LOAD_OP_CLEAR, VK_ATTACHMENT_STORE_OP_STORE});
        loadStoreInfos.emplace_back(LoadStoreInfo{VK_ATTACHMENT_LOAD_OP_CLEAR, VK_ATTACHMENT_STORE_OP_DONT_CARE});

        std::vector<SubpassInfo> subPassInfos;
        subPassInfos.emplace_back(SubpassInfo{{}, {0}, {}, false, 0, VK_RESOLVE_MODE_NONE});

        auto &renderPass = device->get_resource_cache().request_render_pass(GraphicContext::g_offScreenRT->get_attachments(), loadStoreInfos, subPassInfos);
        auto &frameBuffer = device->get_resource_cache().request_framebuffer(*GraphicContext::g_offScreenRT, renderPass);

        set_viewport_and_scissor(commandBuffer, get_render_context().get_surface_extent());
        commandBuffer.begin_render_pass(*GraphicContext::g_offScreenRT, renderPass,frameBuffer, zeroClearValue);

        MainPass::Draw(context, commandBuffer, m_mainCamera, &opaqueNodes, scene.get());
        
        // draw gui
        if (gui) {
            gui->draw(commandBuffer);
        }

        commandBuffer.end_render_pass();
    }

    blit_and_present(commandBuffer);
    commandBuffer.end();
    context.submit(commandBuffer);
}

void character_render::blit_and_present(vkb::CommandBuffer &commandBuffer) {
    auto &colorImageView = GraphicContext::g_offScreenRT->get_views()[0];
    auto &swapChainView = render_context->get_active_frame().get_render_target().get_views()[0];

    ImageMemoryBarrier barrier{};
    barrier.src_stage_mask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
    barrier.dst_stage_mask = VK_PIPELINE_STAGE_TRANSFER_BIT;
    barrier.src_access_mask = VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT;
    barrier.dst_access_mask = VK_ACCESS_TRANSFER_READ_BIT;
    barrier.old_layout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
    barrier.new_layout = VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL;
    commandBuffer.image_memory_barrier(colorImageView, barrier);

    barrier.src_stage_mask = VK_PIPELINE_STAGE_BOTTOM_OF_PIPE_BIT;
    barrier.dst_stage_mask = VK_PIPELINE_STAGE_TRANSFER_BIT;
    barrier.src_access_mask = 0;
    barrier.dst_access_mask = VK_ACCESS_TRANSFER_WRITE_BIT;
    barrier.old_layout = VK_IMAGE_LAYOUT_UNDEFINED;
    barrier.new_layout = VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL;
    commandBuffer.image_memory_barrier(swapChainView, barrier);

    VkExtent3D  colorImageExtent = colorImageView.get_image().get_extent();
    VkExtent3D  swapChainExtent = swapChainView.get_image().get_extent();
    VkImageBlit blit{};
    blit.srcOffsets[0] = VkOffset3D{0, 0, 0};
    blit.srcOffsets[1] = VkOffset3D{(int32_t)colorImageExtent.width, (int32_t)colorImageExtent.height, 0};
    blit.dstOffsets[0] = VkOffset3D{0, 0, 0};
    blit.dstOffsets[1] = VkOffset3D{(int32_t)swapChainExtent.width, (int32_t)swapChainExtent.height, 0};
    blit.srcSubresource.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
    blit.srcSubresource.layerCount = 1;
    blit.dstSubresource.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
    blit.dstSubresource.layerCount = 1;

    commandBuffer.blit_image(colorImageView.get_image(), swapChainView.get_image(), {blit});

    barrier.src_stage_mask = VK_PIPELINE_STAGE_TRANSFER_BIT;
    barrier.dst_stage_mask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
    barrier.src_access_mask = VK_ACCESS_TRANSFER_READ_BIT;
    barrier.dst_access_mask = VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT;
    barrier.old_layout = VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL;
    barrier.new_layout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
    commandBuffer.image_memory_barrier(colorImageView, barrier);

    barrier.src_stage_mask = VK_PIPELINE_STAGE_TRANSFER_BIT;
    barrier.dst_stage_mask = VK_PIPELINE_STAGE_BOTTOM_OF_PIPE_BIT;
    barrier.src_access_mask = VK_ACCESS_TRANSFER_WRITE_BIT;
    barrier.dst_access_mask = 0;
    barrier.old_layout = VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL;
    barrier.new_layout = VK_IMAGE_LAYOUT_PRESENT_SRC_KHR;
    commandBuffer.image_memory_barrier(swapChainView, barrier);
}

void character_render::input_event(const vkb::InputEvent & input_event) {
    VulkanSample::input_event(input_event);
}

void character_render::draw_gui() {
    VulkanSample::draw_gui();

    gui->show_options_window(
        [this]() {

        if (ImGui::CollapsingHeader("Camera Options")) {
            float speed = m_freeCamera->get_speed();
            ImGui::SliderFloat("Camera Move Speed", &speed, 0.1, 5.0);
            m_freeCamera->set_speed(speed);
            const auto& cameraRTS = m_mainCamera->get_node()->get_transform();
            const auto& cameraPosition = cameraRTS.get_translation();
            ImColor green(0.0f, 1.0f, 0.0f);
            ImGui::SameLine();
            ImGui::TextColored(green, "Main Camera, position : %.2f, %.2f, %.2f.",
                cameraPosition.x, cameraPosition.y, cameraPosition.z);
        }
        
        if (ImGui::CollapsingHeader("Light Options")) {
            auto& lights = scene->get_components<sg::Light>();
            auto lightProperties(lights[0]->get_properties());
            ImGui::DragFloat3("Direction", &lightProperties.direction.r, 0.01, -1.0f, 1.0f);
            ImGui::SameLine();
            ImGui::DragFloat("Intensity", &lightProperties.intensity, 0.1, 0.1, 10.0);
            lightProperties.direction = glm::normalize(lightProperties.direction);
            lights[0]->set_properties(lightProperties);
        }

        ImGui::Separator();
        ImGui::Text("Render Options :");
        ImGui::BeginChild("Render Options", ImVec2(0, ImGui::GetFontSize() * 3.0f));
        {
            ImGui::Checkbox("ScreenSpace SSS", &m_useScreenSpaceSSS);
            ImGui::Checkbox("Color Bleed AO", &m_useColorBleedAO);
            ImGui::Checkbox("Double Specular", &m_useDoubleSpecular);
            ImGui::Checkbox("SSAO", &m_useSSAO);
        }
        ImGui::EndChild();
    }, 6);
}

void character_render::finish() {
    VulkanSample::finish();
    GraphicContext::DestroyGraphicBuffer();
    ShaderProgram::g_shaderSources.clear();
    ShaderProgram::g_shaderModules.clear();
}