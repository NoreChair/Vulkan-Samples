#include "gpu_occlusion.h"

#include "gltf_loader.h"
#include "ctpl_stl.h"

#include "common/utils.h"
#include "scene_graph/node.h"
#include "scene_graph/components/mesh.h"
#include "scene_graph/components/sub_mesh.h"
#include "scene_graph/components/pbr_material.h"


#include "api_vulkan_sample.h"
#include "graphic_context.h"
#include "render_utils.h"

#include "main_pass.h"

#include <thread>
#include <algorithm>
#include <atomic>

using namespace vkb;
using namespace vkb::core;
using namespace GraphicContext;
using namespace GraphicResources;
using namespace RenderSetting;

const RenderTarget::CreateFunc gpu_occlusion::swap_chain_create_func = [](Image&& swapchain_image) -> std::unique_ptr<RenderTarget> {
    const auto& physicalDevice = swapchain_image.get_device().get_gpu().get_handle();
    std::vector<Image> images;
    images.push_back(std::move(swapchain_image));
    return std::make_unique<RenderTarget>(std::move(images));
};

ctpl::thread_pool s_threadPool;
std::vector<vkb::sg::Mesh*> s_fullMeshes;

gpu_occlusion::gpu_occlusion() {
    set_name(k_name);
}

gpu_occlusion::~gpu_occlusion() {}

bool gpu_occlusion::prepare(vkb::Platform& platform) {
    if (!VulkanSample::prepare(platform)) {
        return false;
    }

    auto& extent = get_render_context().get_surface_extent();

    prepare_resources();
    prepare_scene();

    GraphicContext::InitAll(*device, extent.width, extent.height);
    MainPass::Init(*device);

    gui = std::make_unique<vkb::Gui>(*this, platform.get_window(), stats.get());
    return true;
}

void gpu_occlusion::prepare_render_context() {
    render_context->prepare(1, swap_chain_create_func);
}

void gpu_occlusion::request_gpu_features(vkb::PhysicalDevice& gpu) {
    if (gpu.get_features().textureCompressionASTC_LDR) {
        LOGI("Support ASTC compression");
    } else {
        LOGI("Not support ASTC compression");
    }
}

void gpu_occlusion::resize(const uint32_t width, const uint32_t height) {
    VulkanSample::resize(width, height);
    device->wait_idle();
    GraphicContext::Resize(*device, width, height);
}

void gpu_occlusion::update(float delta_time) {
    update_scene(delta_time);
    update_gui(delta_time);
    render(delta_time);
}

void gpu_occlusion::input_event(const vkb::InputEvent& input_event) {
    VulkanSample::input_event(input_event);
}

void gpu_occlusion::draw_gui() {}

void gpu_occlusion::finish() {
    GraphicContext::ReleaseAll();
    s_threadPool.stop();
}

void gpu_occlusion::prepare_resources() {
    const std::string scenePath = "rocks/Terrain.gltf";
    GLTFLoader loader{*device};
    scene = loader.read_scene_from_file(scenePath);

    if (!scene) {
        LOGE("Cannot load scene: {}", scenePath.c_str());
        throw std::runtime_error("Cannot load scene: " + scenePath);
    }

    m_timer.start();

    auto threadCount = std::thread::hardware_concurrency();
    threadCount = threadCount == 0 ? 1 : glm::max(threadCount - 1, 1u);
    s_threadPool.resize(threadCount);

    // load shader
    struct GShader {
        std::string name;
        VkShaderStageFlagBits flags;
    };

    std::vector<GShader> shaders{
        {"debug_draw.vert", VK_SHADER_STAGE_VERTEX_BIT},
        {"debug_draw.frag", VK_SHADER_STAGE_FRAGMENT_BIT},
        {"depth_only_instance.vert", VK_SHADER_STAGE_VERTEX_BIT},
        {"depth_only_instance.frag", VK_SHADER_STAGE_FRAGMENT_BIT},
        {"depth_only.vert", VK_SHADER_STAGE_VERTEX_BIT},
        {"depth_only.frag", VK_SHADER_STAGE_FRAGMENT_BIT},
        {"rock.vert", VK_SHADER_STAGE_VERTEX_BIT},
        {"rock.frag", VK_SHADER_STAGE_FRAGMENT_BIT},
        {"sky.vert", VK_SHADER_STAGE_VERTEX_BIT},
        {"sky.frag", VK_SHADER_STAGE_FRAGMENT_BIT}
    };

    std::vector<std::future<std::unique_ptr<ShaderSource>>> shader_futures;
    for (int i = 0; i < shaders.size(); ++i) {
        auto future = s_threadPool.push([this, i, shaders](int) {
            const std::string shader_path = "gpu_occlusion/";
            auto source = std::make_unique<ShaderSource>(shader_path + shaders[i].name);
            LOGI("Loaded shader #{} ({})", i, shaders[i].name.c_str());
            return source;
        });
        shader_futures.push_back(std::move(future));
    }

    auto unitCube = loader.read_simple_model_from_file("scenes/unit_cube.gltf", 0);
    m_unitCube = unitCube.get();
    scene->add_component(std::move(unitCube));

    for (int i = 0; i < shaders.size(); ++i) {
        auto source = shader_futures[i].get();
        ShaderModule& moudle = get_device().get_resource_cache().request_shader_module(shaders[i].flags, *source);
        moudle.set_resource_mode("GlobalUniform", vkb::ShaderResourceMode::Dynamic);
        moudle.set_resource_mode("LocalUniform", vkb::ShaderResourceMode::Dynamic);
        g_shaderModules.emplace(shaders[i].name, &moudle);
        source.reset();
    }

    LOGI("Load shader and model use {} seconds.", vkb::to_string(m_timer.stop()));
}

void gpu_occlusion::prepare_scene() {
    // Extract mesh
    s_fullMeshes.clear();
    const auto meshes = scene->get_components<vkb::sg::Mesh>();
    s_fullMeshes.reserve(meshes.size() / 3);
    for (size_t i = 0; i < meshes.size(); i++) {
        if ((meshes[i]->get_name().find("LOD1") == std::string::npos) && (meshes[i]->get_name().find("LOD2") == std::string::npos)) {
            s_fullMeshes.emplace_back(meshes[i]);
        }
    }

    for (auto mesh : s_fullMeshes) {
        for (auto node : mesh->get_nodes()) {
            auto& bounds = node->get_component<vkb::sg::Mesh>().get_bounds();
            auto worldBound = std::make_unique<vkb::sg::AABB>(bounds.get_min(), bounds.get_max());
            glm::mat4 worldMatrix = glm::scale(node->get_transform().get_scale());
            worldBound->transform(worldMatrix);
            node->set_component(*worldBound);
            scene->add_component(std::move(worldBound));
        }
    }

    // camera
    auto &camera_node = vkb::add_free_camera(*scene, "main_camera", get_render_context().get_surface_extent(), 0.5);
    m_mainCamera = &camera_node.get_component<vkb::sg::Camera>();
    m_mainCamera->set_far_plane(1000.0f);
    m_mainCamera->set_near_plane(0.01f);

    // light
    auto& lights = scene->get_components<sg::Light>();
    auto  iter = std::find_if(lights.begin(), lights.end(), [](sg::Light *iter) -> bool { return iter->get_light_type() == sg::LightType::Directional; });
    assert(iter != lights.end());

    sg::Light* directLight = *iter;
    sg::LightProperties sunLight;
    sunLight.direction = glm::normalize(glm::vec3(0.697, -0.557, -0.452));
    sunLight.intensity = 7.5f;
    directLight->set_properties(sunLight);
}

void gpu_occlusion::render(float delta_time) {
    // There have two ways use gpu occlusion query:
    // 1. use subpass to copy occluder depth to visible buffer, and qeury before next frame draw begin.
    // 2. draw half depth buffer with occluder then draw occludee to query whether it be culled.
    // Extension:
    // 1. condition render
    // 2. instead of draw hight precision model, use AABB or LOD lowest to query

    m_timer.start();

    glm::vec4 frustum[6];
    RenderUtils::ExtractPlanes(m_mainCamera->get_projection() * m_mainCamera->get_view(), frustum);

    RenderUtils::SortedMeshes opaqueNodes;
    RenderUtils::SortedMeshes transparentNodes;
    RenderUtils::SortedMeshes cullOffNodes;
    sg::Transform &mainCameraRTS = m_mainCamera->get_node()->get_transform();
    glm::vec3     cameraForward = glm::mat3(mainCameraRTS.get_world_matrix()) * glm::vec3(0.0, 0.0, 1.0);

    std::vector<std::reference_wrapper<const sg::AABB>> boundingBoxs;
    uint32_t nodeCount = 0;

    std::vector<sg::Node*> visibleNode;
    for (auto var : s_fullMeshes) {
        const auto &bounds = var->get_bounds();
        for (auto node : var->get_nodes()) {
            nodeCount++;
            const auto& aabb = node->get_component<sg::AABB>();
            if (RenderUtils::FrustumAABB(frustum, aabb.get_min(), aabb.get_max())) {
                boundingBoxs.emplace_back(std::ref(aabb));
                glm::vec3 delta = aabb.get_center() - mainCameraRTS.get_world_translation();
                float     distance = glm::dot(cameraForward, delta);

                for (auto &sub_mesh : node->get_component<sg::Mesh>().get_submeshes()) {
                    if (sub_mesh->get_material()->alpha_mode == vkb::sg::AlphaMode::Blend) {
                        transparentNodes.emplace(distance, std::make_pair(node, sub_mesh));
                    } else if (sub_mesh->get_material()->alpha_mode == vkb::sg::AlphaMode::Mask) {
                        cullOffNodes.emplace(distance, std::make_pair(node, sub_mesh));
                    } else {
                        opaqueNodes.emplace(distance, std::make_pair(node, sub_mesh));
                    }
                }
            }
        }
    }
    LOGI("All count {}, visible count {}", vkb::to_string(nodeCount), vkb::to_string(boundingBoxs.size()));
    m_timer.lap();

    auto& commandBuffer = render_context->begin();
    commandBuffer.begin(VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT);

    ImageMemoryBarrier barrier{};
    barrier.src_stage_mask = VK_PIPELINE_STAGE_BOTTOM_OF_PIPE_BIT;
    barrier.dst_stage_mask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
    barrier.src_access_mask = 0;
    barrier.dst_access_mask = VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT;
    barrier.old_layout = VK_IMAGE_LAYOUT_UNDEFINED;
    barrier.new_layout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
    commandBuffer.image_memory_barrier(render_context->get_active_frame().get_render_target().get_views()[0], barrier);

    MainPass::BeginPass(*render_context.get(), commandBuffer);

    MainPass::DrawOpaque(*render_context, commandBuffer, m_mainCamera, &opaqueNodes, scene.get());

    MainPass::DrawDebugAABB(*render_context, commandBuffer, m_mainCamera, m_unitCube, boundingBoxs);

    if (gui) {
        gui->draw(commandBuffer);
    }

    MainPass::EndPass(commandBuffer);

    barrier.src_stage_mask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
    barrier.dst_stage_mask = VK_PIPELINE_STAGE_BOTTOM_OF_PIPE_BIT;
    barrier.src_access_mask = VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT;
    barrier.dst_access_mask = 0;
    barrier.old_layout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
    barrier.new_layout = VK_IMAGE_LAYOUT_PRESENT_SRC_KHR;
    commandBuffer.image_memory_barrier(render_context->get_active_frame().get_render_target().get_views()[0], barrier);

    commandBuffer.end();
    render_context->submit(commandBuffer);

    m_timer.stop();
}

std::unique_ptr<vkb::Application> create_gpu_occlusion() {
    return std::make_unique<gpu_occlusion>();
}
