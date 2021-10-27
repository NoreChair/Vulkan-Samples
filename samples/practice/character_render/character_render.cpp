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
#include "scene_graph/components/perspective_camera.h"

#include "graphic_context.h"
#include "shadow_pass.h"
#include "main_pass.h"
#include "subsurface_pass.h"

#include <thread>
#include <algorithm>

using namespace vkb;
using namespace vkb::core;
using namespace GraphicContext;
using namespace RenderSetting;

const RenderTarget::CreateFunc character_render::swap_chain_create_func = [](Image &&swapchain_image) -> std::unique_ptr<RenderTarget> {
    const auto& physicalDevice = swapchain_image.get_device().get_gpu().get_handle();
    std::vector<Image> images;
    images.push_back(std::move(swapchain_image));
    return std::make_unique<RenderTarget>(std::move(images));
};

glm::vec3 shadowCenter(1.5, 7.78, -190.0);


std::unique_ptr<vkb::Application> create_character_render() {
    return std::make_unique<character_render>();
}

character_render::character_render() {
    set_name(k_name);
}

character_render::~character_render() {}

bool character_render::prepare(vkb::Platform & platform) {
    if (!VulkanSample::prepare(platform)) {
        return false;
    }

    prepare_msaa_mode();
    auto& extent = get_render_context().get_surface_extent();
    InitGraphicBuffer(get_device(), extent.width, extent.height);
    InitRenderSetting();
    prepare_resources();
    prepare_scene();

    // init render resource
    ShadowPass::Init(get_device());
    SubsurfacePass::Init(get_device());
    MainPass::Init(get_device());
    //SSAO::Init();

    gui = std::make_unique<vkb::Gui>(*this, platform.get_window(), stats.get());

    return true;
}

const std::vector<const char *> character_render::get_validation_layers() {
    //return {"VK_LAYER_LUNARG_standard_validation"};
    return {};
}

void character_render::prepare_msaa_mode() {

    if (instance->is_enabled(VK_KHR_GET_PHYSICAL_DEVICE_PROPERTIES_2_EXTENSION_NAME)) {
        VkPhysicalDeviceProperties2KHR properties{};
        properties.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_PROPERTIES_2_KHR;
        VkPhysicalDeviceDepthStencilResolvePropertiesKHR resolveProperties{};
        resolveProperties.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_DEPTH_STENCIL_RESOLVE_PROPERTIES_KHR;
        properties.pNext = static_cast<void *>(&resolveProperties);
        vkGetPhysicalDeviceProperties2KHR(get_device().get_gpu().get_handle(), &properties);

        g_multiSampleCount = (properties.properties.limits.framebufferColorSampleCounts & VK_SAMPLE_COUNT_4_BIT) != 0? VK_SAMPLE_COUNT_4_BIT: VK_SAMPLE_COUNT_1_BIT;

        if (resolveProperties.supportedDepthResolveModes == 0) {
            LOGW("No depth stencil resolve modes supported");
            g_depthResolveSupported = false;
            g_depthResolveMode = VK_RESOLVE_MODE_NONE;
        } else {
            g_depthResolveSupported = true;
            // All possible modes are listed here from most to least preferred as default
            std::vector<VkResolveModeFlagBits> modes = {VK_RESOLVE_MODE_SAMPLE_ZERO_BIT, VK_RESOLVE_MODE_MIN_BIT,
                                                        VK_RESOLVE_MODE_MAX_BIT, VK_RESOLVE_MODE_AVERAGE_BIT};
            for (auto &mode : modes) {
                if (resolveProperties.supportedDepthResolveModes & mode) {
                    g_depthResolveMode = mode;
                }
            }
        }
    } else {
        VkPhysicalDeviceProperties properties{};
        vkGetPhysicalDeviceProperties(get_device().get_gpu().get_handle(), &properties);
        g_multiSampleCount = (properties.limits.framebufferColorSampleCounts & VK_SAMPLE_COUNT_4_BIT) != 0 ? VK_SAMPLE_COUNT_4_BIT : VK_SAMPLE_COUNT_1_BIT;
    }
}

void character_render::prepare_resources() {
    auto& device = get_render_context().get_device();

    Timer timer;
    timer.start();
    //#define LOAD_IN_MAIN_THREAD
    auto thread_count = std::thread::hardware_concurrency();
    thread_count = thread_count == 0 ? 1 : thread_count - 1;
#ifndef LOAD_IN_MAIN_THREAD
    ctpl::thread_pool thread_pool(thread_count);
#else
    thread_count = 1;
#endif
    // load texture
    struct LoadTextureInfo {
        std::string name;
        bool isColor;
        bool genMipmap;
    };

    std::vector<LoadTextureInfo> textures{
        {"T_Color.png", true, true},
        {"T_N.png", false, true},
        {"T_AO_SSS_CA.png", false, true},
        {"T_SR.png", false, true},
        {"T_SkinMicroNormal.png", false, true},
        {"output_skybox.ktx", true, false},
        {"output_iem.ktx", true, false},
        {"output_pmrem.ktx", true, false}
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
            if (textures[image_index].genMipmap) {
                image->generate_mipmaps();
            }
            bool isCubeMap = image->get_layers() == 6;
            image->create_vk_image(get_device(), isCubeMap ? VK_IMAGE_VIEW_TYPE_CUBE : VK_IMAGE_VIEW_TYPE_2D, isCubeMap ? VK_IMAGE_CREATE_CUBE_COMPATIBLE_BIT : 0);
            LOGI("Loadedimage #{} ({})", image_index, textures[image_index].name.c_str());
            return image;
        });
        image_component_futures.push_back(std::move(fut));
    }
#else
    const std::string image_path = "character/";
    for (size_t image_index = 0; image_index < image_count; image_index++) {
        std::unique_ptr<sg::Image> image = sg::Image::load(textures[image_index].name, image_path + textures[image_index].name, textures[image_index].isColor);
        LOGI("Loadedimage #{} ({})", image_index, textures[image_index].name.c_str());
        if (textures[image_index].genMipmap) {
            image->generate_mipmaps();
        }
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
        {"sss_irradiance.vert", "", VK_SHADER_STAGE_VERTEX_BIT},
        {"sss_irradiance.frag", "", VK_SHADER_STAGE_FRAGMENT_BIT},
        {"sss_blur.vert", "", VK_SHADER_STAGE_VERTEX_BIT},
        {"sss_blur.frag", "", VK_SHADER_STAGE_FRAGMENT_BIT},
        {"sss_blur_accum.frag", "", VK_SHADER_STAGE_FRAGMENT_BIT},
        {"sky.vert", "", VK_SHADER_STAGE_VERTEX_BIT},
        {"sky.frag", "", VK_SHADER_STAGE_FRAGMENT_BIT}
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
                GraphicResources::g_shaderSources.emplace(shaders[i].name, std::move(source));
                GraphicResources::g_shaderModules.emplace(shaders[i].name, &moudle);
                shaderLoadedCount++;
            }
            cv.notify_one();
#ifndef LOAD_IN_MAIN_THREAD
        });
#endif
    }

    // load from gltf
    load_scene("character/Head.gltf");

    GLTFLoader loader{device};
    auto unitCube = loader.read_simple_model_from_file("character/cube.gltf", 0);
    m_unitCube = unitCube.get();
    scene->add_component(std::move(unitCube));

    // Sync, update images
#ifndef LOAD_IN_MAIN_THREAD
    for (auto &fut : image_component_futures) {
        image_components.push_back(fut.get());
    }
#endif
    for (size_t i = 0; i < image_components.size(); i++) {
        GraphicResources::g_sceneTextures[i] = &image_components[i]->get_vk_image_view();
    }

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

    auto &camera_node = vkb::add_free_camera(*scene, "main_camera", get_render_context().get_surface_extent(), 0.5);
    m_freeCamera = (vkb::sg::FreeCamera*)&camera_node.get_component<vkb::sg::Script>();
    m_mainCamera = &camera_node.get_component<vkb::sg::Camera>();
    m_mainCamera->set_far_plane(1000.0f);
    m_mainCamera->set_near_plane(0.01f);
    m_mainCamera->get_node()->get_transform().set_translation({0.0, 6.78, -170.0});

    auto& lights = scene->get_components<sg::Light>();
    auto  iter = std::find_if(lights.begin(), lights.end(), [](sg::Light *iter) -> bool { return iter->get_light_type() == sg::LightType::Directional; });
    assert(iter != lights.end());

    // sun direction
    sg::Light* directLight = *iter;
    sg::LightProperties sunLight;
    sunLight.direction = glm::normalize(glm::vec3(0.697, -0.557, -0.452));
    sunLight.intensity = 7.5f;
    directLight->set_properties(sunLight);

    // add shadow camera
    auto lightNode = directLight->get_node();
    auto shadowCamera = std::make_unique<ShadowCamera>("main_light_camera");
    shadowCamera->set_node(*lightNode);
    m_lightCamera = shadowCamera.get();
    glm::vec2 shadowMapExtent = glm::vec2(g_shadowImage->get_extent().width, g_shadowImage->get_extent().height);
    lightNode->get_transform().set_translation(shadowCenter - sunLight.direction * 64.0f);
    m_lightCamera->set_up(sunLight.direction, shadowCenter, glm::vec3(32, 32, 64), shadowMapExtent, 16);
    scene->add_component(std::move(shadowCamera), *lightNode);
}

void character_render::prepare_render_context() {
    auto &properties = render_context->get_swapchain().get_properties();
    render_context->prepare(1, swap_chain_create_func);
}

void character_render::request_gpu_features(vkb::PhysicalDevice & gpu) {
}

void character_render::resize(const uint32_t width, const uint32_t height) {
    VulkanSample::resize(width, height);
    device->wait_idle();
    ResizeGraphicBuffer(get_device(), width, height);
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
    std::vector<VkClearValue> zeroClearValue{initializers::clear_color_value(0.0f, 0.0f, 0.0f, 0.0f), initializers::clear_depth_stencil_value(0.0f, 0.0f)};
    std::vector<VkClearValue> shadowClearValue{initializers::clear_depth_stencil_value(1.0f, 0.0f)};

    VkExtent2D     extent = get_render_context().get_surface_extent();
    RenderContext &context = get_render_context();
    CommandBuffer &commandBuffer = context.begin();
    commandBuffer.begin(VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT);

    auto& targetView = const_cast<core::ImageView&>(render_context->get_active_frame().get_render_target().get_views()[0]);

    ImageMemoryBarrier barrier{};

    // shadow pass
    {
        barrier.src_stage_mask = VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT;
        barrier.dst_stage_mask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
        barrier.src_access_mask = 0;
        barrier.dst_access_mask = VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT;
        barrier.old_layout = VK_IMAGE_LAYOUT_UNDEFINED;
        barrier.new_layout = VK_IMAGE_LAYOUT_DEPTH_ATTACHMENT_OPTIMAL;
        commandBuffer.image_memory_barrier(g_shadowImage->get_views()[0], barrier);

        std::vector<LoadStoreInfo> loadStoreInfos;
        loadStoreInfos.emplace_back(LoadStoreInfo{VK_ATTACHMENT_LOAD_OP_CLEAR, VK_ATTACHMENT_STORE_OP_STORE});

        std::vector<SubpassInfo> subPassInfos;
        subPassInfos.emplace_back(SubpassInfo{{}, {}, {}, false, 0, VK_RESOLVE_MODE_NONE});

        auto &renderPass = device->get_resource_cache().request_render_pass(g_shadowImage->get_attachments(), loadStoreInfos, subPassInfos);
        auto &frameBuffer = device->get_resource_cache().request_framebuffer(*g_shadowImage, renderPass);

        set_viewport_and_scissor(commandBuffer, g_shadowImage->get_extent());
        commandBuffer.begin_render_pass(*g_shadowImage, renderPass, frameBuffer, shadowClearValue);

        ShadowPass::Draw(context, commandBuffer, m_lightCamera, &opaqueNodes);

        commandBuffer.end_render_pass();
    }

    // subsurface scattering pass
    {
        SubsurfacePass::Draw(context, commandBuffer, m_mainCamera, &opaqueNodes, scene.get());
    }

    // main pass
    {
        bool msaaEnable = g_multiSampleCount != VK_SAMPLE_COUNT_1_BIT;

        barrier.src_stage_mask = VK_PIPELINE_STAGE_BOTTOM_OF_PIPE_BIT;
        barrier.dst_stage_mask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
        barrier.src_access_mask = 0;
        barrier.dst_access_mask = VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT;
        barrier.old_layout = VK_IMAGE_LAYOUT_UNDEFINED;
        barrier.new_layout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
        commandBuffer.image_memory_barrier(targetView, barrier);

        if (msaaEnable) {
            commandBuffer.image_memory_barrier(*g_sceneColorMSView, barrier);

            barrier.old_layout = VK_IMAGE_LAYOUT_UNDEFINED;
            barrier.new_layout = VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL;
            barrier.src_access_mask = 0;
            barrier.dst_access_mask = VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_READ_BIT | VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT;
            barrier.src_stage_mask = VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT;
            barrier.dst_stage_mask = VK_PIPELINE_STAGE_EARLY_FRAGMENT_TESTS_BIT | VK_PIPELINE_STAGE_LATE_FRAGMENT_TESTS_BIT;
            commandBuffer.image_memory_barrier(*g_sceneDepthMSView, barrier);
        } else {
            barrier.old_layout = VK_IMAGE_LAYOUT_UNDEFINED;
            barrier.new_layout = VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL;
            barrier.src_access_mask = 0;
            barrier.dst_access_mask = VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_READ_BIT | VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT;
            barrier.src_stage_mask = VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT;
            barrier.dst_stage_mask = VK_PIPELINE_STAGE_EARLY_FRAGMENT_TESTS_BIT | VK_PIPELINE_STAGE_LATE_FRAGMENT_TESTS_BIT;
            commandBuffer.image_memory_barrier(*g_sceneDepthView, barrier);
        }

        barrier.src_stage_mask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
        barrier.dst_stage_mask = VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT;
        barrier.src_access_mask = VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT;
        barrier.dst_access_mask = VK_ACCESS_SHADER_READ_BIT;
        barrier.old_layout = VK_IMAGE_LAYOUT_DEPTH_ATTACHMENT_OPTIMAL;
        barrier.new_layout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
        commandBuffer.image_memory_barrier(g_shadowImage->get_views()[0], barrier);

        std::vector<SubpassInfo> subPassInfos;
        std::vector<Attachment> attachments;
        std::vector<vkb::core::ImageView*> imgs;
        std::vector<LoadStoreInfo> loadStoreInfos;

        if (msaaEnable) {
            subPassInfos.emplace_back(SubpassInfo{{}, {0}, {2}, false, 0, VK_RESOLVE_MODE_NONE});

            attachments.emplace_back(Attachment{g_sceneColorMS->get_format(), g_sceneColorMS->get_sample_count(), g_sceneColorMS->get_usage()});
            imgs.push_back(g_sceneColorMSView.get());
            loadStoreInfos.emplace_back(LoadStoreInfo{VK_ATTACHMENT_LOAD_OP_CLEAR, VK_ATTACHMENT_STORE_OP_DONT_CARE});

            attachments.emplace_back(Attachment{g_sceneDepthMS->get_format(), g_sceneDepthMS->get_sample_count(), g_sceneDepthMS->get_usage()});
            imgs.push_back(g_sceneDepthMSView.get());
            loadStoreInfos.emplace_back(LoadStoreInfo{VK_ATTACHMENT_LOAD_OP_CLEAR, VK_ATTACHMENT_STORE_OP_DONT_CARE});
        } else {
            subPassInfos.emplace_back(SubpassInfo{{}, {1}, {}, false, 0, VK_RESOLVE_MODE_NONE});

            attachments.emplace_back(Attachment{g_sceneDepth->get_format(),g_sceneDepth->get_sample_count(),g_sceneDepth->get_usage()});
            imgs.push_back(g_sceneDepthView.get());
            loadStoreInfos.emplace_back(LoadStoreInfo{VK_ATTACHMENT_LOAD_OP_CLEAR, VK_ATTACHMENT_STORE_OP_DONT_CARE});
        }

        attachments.emplace_back(Attachment{targetView.get_image().get_format(), targetView.get_image().get_sample_count(), targetView.get_image().get_usage()});
        imgs.push_back(&targetView);
        loadStoreInfos.emplace_back(LoadStoreInfo{VK_ATTACHMENT_LOAD_OP_DONT_CARE, VK_ATTACHMENT_STORE_OP_STORE});

        auto &renderPass = device->get_resource_cache().request_render_pass(attachments, loadStoreInfos, subPassInfos);
        auto &frameBuffer = device->get_resource_cache().request_framebuffer(imgs, renderPass);

        set_viewport_and_scissor(commandBuffer, get_render_context().get_surface_extent());
        commandBuffer.begin_render_pass(extent, renderPass, frameBuffer, zeroClearValue);

        MainPass::Draw(context, commandBuffer, m_mainCamera, m_lightCamera, &opaqueNodes, scene.get());
        MainPass::DrawSky(context, commandBuffer, m_mainCamera, m_unitCube);

        // draw gui
        if (gui) {
            gui->draw(commandBuffer);
        }

        commandBuffer.end_render_pass();
    }

    //blit_and_present(commandBuffer);
    commandBuffer.end();
    context.submit(commandBuffer);
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
            ImGui::SameLine();
            if (ImGui::Button("Reset")) {
                m_freeCamera->get_node().get_transform().set_translation({0.0, 6.78, -170.0});
                m_freeCamera->get_node().get_transform().set_rotation(glm::quat(1.0, 0.0, 0.0, 0.0));
            }
        }

        ImGui::Separator();
        if (ImGui::CollapsingHeader("Light Options")) {
            auto& lights = scene->get_components<sg::Light>();
            auto lightProperties(lights[0]->get_properties());
            ImGui::DragFloat3("Direction", &lightProperties.direction.r, 0.01, -1.0f, 1.0f);
            ImGui::SameLine();
            ImGui::DragFloat("Intensity", &lightProperties.intensity, 0.1, 0.1, 10.0);
            ImGui::ColorEdit3("Color", &lightProperties.color.r);
            lightProperties.direction = glm::normalize(lightProperties.direction);
            lights[0]->set_properties(lightProperties);

            glm::vec3 up(0.0f, 1.0f, 0.0f);
            float cosin = glm::dot(lightProperties.direction, up);
            if (glm::abs(cosin) > 0.99) {
                up.y = 0.0f;
                up.z = -glm::sign(cosin);
            }
            auto& transform = lights[0]->get_node()->get_transform();
            glm::quat rotation = glm::toQuat(glm::lookAt(glm::vec3(0.0), lightProperties.direction, up));
            transform.set_rotation(rotation);
            transform.set_translation(shadowCenter - lightProperties.direction * 64.0f);
            glm::vec2 shadowMapExtent = glm::vec2(g_shadowImage->get_extent().width, g_shadowImage->get_extent().height);
            m_lightCamera->set_up(lightProperties.direction, shadowCenter, glm::vec3(32, 32, 64), shadowMapExtent, 16);
        }

        ImGui::Separator();
        if (ImGui::CollapsingHeader("Render Options")) {
            ImGui::BeginChild("", ImVec2(0, ImGui::GetFontSize() * 4.0f));
            {
                ImGui::Checkbox("Only SSS", &g_onlySSS);  ImGui::SameLine();
                ImGui::Checkbox("Only Shadow", &g_onlyShadow);
                ImGui::Checkbox("ScreenSpace SSS", &g_useScreenSpaceSSS);  ImGui::SameLine();
                ImGui::Checkbox("Color Bleed AO", &g_useColorBleedAO);  ImGui::SameLine();
                ImGui::Checkbox("Double Specular", &g_useDoubleSpecular);
                ImGui::DragFloat3("Shadow Bias", g_shadowBias, 0.1f, -100.0f, 100.0f);
                ImGui::DragFloat("Normal Bias", &g_shadowNormalBias, 0.01f, 0.0f, 10.0f);
                ImGui::SliderFloat("Roughtness", &g_roughness, 0.01, 1.0);
                ImGui::SliderFloat("Metalness", &g_metalness, 0.01, 1.0);
                ImGui::SliderFloat("SSS Level", &g_sssLevel, 0.0, 1000.0);
                ImGui::SliderFloat("SSS Correction", &g_sssCorrection, 0.0, 10000.0);
                ImGui::SliderFloat("SSS Max DD", &g_sssMaxDD, 0.001, 1.0);
            }
            ImGui::EndChild();
        }
    }, 7);
}

void character_render::finish() {
    VulkanSample::finish();
    DestroyGraphicBuffer();
    GraphicResources::g_shaderSources.clear();
    GraphicResources::g_shaderModules.clear();
}