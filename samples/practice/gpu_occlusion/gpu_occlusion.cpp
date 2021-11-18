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
#include "occlusion_cull_pass.h"

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

    const auto& extent = get_render_context().get_surface_extent();

    prepare_resources();
    prepare_scene();

    GraphicContext::InitAll(*device, extent.width, extent.height);
    OcclusionCullPass::Init(*device);
    MainPass::Init(*device);

    gui = std::make_unique<vkb::Gui>(*this, platform.get_window(), stats.get(), 16.0f);
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
    device->wait_idle();
    VulkanSample::resize(width, height);
    GraphicContext::Resize(*device, width, height);
    gui->resize(width, height);

    m_firstFrame = true;
    memset(g_visibleResultBuffer.get(), 255, sizeof(uint32_t) * g_maxVisibleQueryCount * 3);
    g_visibleNodeMap[0].clear();
    g_visibleNodeMap[1].clear();
    g_visibleNodeMap[2].clear();
}

QueryMode previousMode = g_queryMode;

void gpu_occlusion::update(float delta_time) {

    m_profilerData.Profile();

    update_scene(delta_time);
    update_gui(delta_time);
    render(delta_time);

    if (previousMode != g_queryMode) {
        m_firstFrame = true;
        memset(g_visibleResultBuffer.get(), 255, sizeof(uint32_t) * g_maxVisibleQueryCount * 3);
        g_visibleNodeMap[0].clear();
        g_visibleNodeMap[1].clear();
        g_visibleNodeMap[2].clear();
    }
    previousMode = g_queryMode;
}

void gpu_occlusion::input_event(const vkb::InputEvent& input_event) {
    VulkanSample::input_event(input_event);
}

void gpu_occlusion::draw_gui() {
    VulkanSample::draw_gui();

    bool windowVisible = ImGui::Begin("Options", nullptr, ImGuiWindowFlags_MenuBar);
    if (!windowVisible) {
        ImGui::End();
        return;
    }
    ImGui::Text("Draw Statis:");
    ImGui::TextWrapped("  All drawable count: %d Actual draw count: %d ",
        m_profilerData.sceneDrawCount, m_profilerData.actualDrawCount);
    ImGui::TextWrapped("  Frustum culling %.2f ms  After frustum cull: %d \n  Occlusion culling avg %.2f ms wait %.4f ms, test %d, survive %d",
        m_profilerData.avgFrustumCullTime, m_profilerData.frustumVisibleCount,
        m_profilerData.avgOcclusionCullTime, m_profilerData.avgOcclusionWaitTime, m_profilerData.occlusionTestCount, m_profilerData.occlusionVisibleCount);
    ImGui::TextWrapped("  Build draw command %.2f ms  Frame time %.2f ms", m_profilerData.avgRenderDrawTime, frame_time);

    ImGui::Separator();
    ImGui::Checkbox("Show AABB", &m_profilerData.drawAABB);
    if (ImGui::CollapsingHeader("Render Options")) {
        if (ImGui::BeginChild("Render Option List")) {
            ImGui::Combo("Occlusion Cull Mode", (int*)&g_queryMode, "None\0Immediate\0Delay\0");
            //if (g_queryMode != 0) {
            //    ImGui::Combo("Proxy Mode", (int*)&g_proxyMode, "Full\0AABB\0");
            //}

            ImGui::EndChild();
        }
    }
    ImGui::End();
}


void gpu_occlusion::finish() {
    GraphicContext::ReleaseAll();
    s_threadPool.stop();
}

void gpu_occlusion::prepare_resources() {
    const std::string scenePath = "rocks/GameObject.gltf";
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
        {"grass.vert", VK_SHADER_STAGE_VERTEX_BIT},
        {"grass.frag", VK_SHADER_STAGE_FRAGMENT_BIT},
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
        for (const auto node : mesh->get_nodes()) {
            auto& bounds = node->get_component<vkb::sg::Mesh>().get_bounds();
            auto worldBound = std::make_unique<vkb::sg::AABB>(bounds.get_min(), bounds.get_max());
            auto mat = node->get_transform().get_world_matrix();
            worldBound->transform(mat);
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
    const auto& lights = scene->get_components<sg::Light>();
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
    RenderUtils::ExtractPlanes(vulkan_style_projection(m_mainCamera->get_projection()) * m_mainCamera->get_view(), frustum);

    RenderUtils::SortedMeshes opaqueNodes;
    RenderUtils::SortedMeshes transparentNodes;
    RenderUtils::SortedMeshes cullOffNodes;
    sg::Transform &mainCameraRTS = m_mainCamera->get_node()->get_transform();
    glm::mat4 cameraMatrix = mainCameraRTS.get_world_matrix();
    glm::vec3 cameraForward = -glm::vec3(cameraMatrix[0][2], cameraMatrix[1][2], cameraMatrix[2][2]);

    m_profilerData.sceneDrawCount = 0;
    m_profilerData.frustumVisibleCount = 0;
    m_profilerData.occlusionVisibleCount = 0;
    m_profilerData.occlusionTestCount = 0;

    std::vector<std::reference_wrapper<const sg::AABB>> opaqueBoundingBoxs;
    std::vector<std::reference_wrapper<const sg::AABB>> alphaTestBoundingBoxs;

    for (auto var : s_fullMeshes) {
        const auto &bounds = var->get_bounds();
        for (auto node : var->get_nodes()) {
            m_profilerData.sceneDrawCount++;
            const auto& aabb = node->get_component<sg::AABB>();
            if (RenderUtils::FrustumAABB(frustum, aabb.get_min(), aabb.get_max())) {
                m_profilerData.frustumVisibleCount++;
                glm::vec3 delta = aabb.get_center() - mainCameraRTS.get_world_translation();
                float     distance = glm::dot(cameraForward, delta);

                for (auto &sub_mesh : node->get_component<sg::Mesh>().get_submeshes()) {
                    if (sub_mesh->get_material()->alpha_mode == vkb::sg::AlphaMode::Opaque) {
                        opaqueNodes.emplace(distance, std::make_pair(node, sub_mesh));
                        opaqueBoundingBoxs.emplace_back(std::ref(aabb));
                    } else if (sub_mesh->get_material()->alpha_mode == vkb::sg::AlphaMode::Mask) {
                        cullOffNodes.emplace(distance, std::make_pair(node, sub_mesh));
                        alphaTestBoundingBoxs.emplace_back(std::ref(aabb));
                    } else {
                        transparentNodes.emplace(distance, std::make_pair(node, sub_mesh));
                    }
                }
            }
        }
    }

    m_profilerData.frustumCullTime += (float)m_timer.elapsed() * 1e3f;
    m_profilerData.actualDrawCount = m_profilerData.frustumVisibleCount;
    m_timer.lap();

    auto& commandBuffer = render_context->begin();

    if (g_queryMode != RenderSetting::QueryMode::None) {
        auto& queryCommandBuffer = render_context->get_active_frame().request_command_buffer(render_context->get_queue());
        queryCommandBuffer.begin(VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT);
        OcclusionCullPass::BeginPass(*render_context, queryCommandBuffer);

        if (g_proxyMode == RenderSetting::ProxyMode::Full) {
            OcclusionCullPass::DrawModel(*render_context, queryCommandBuffer, m_mainCamera, &opaqueNodes, true);
            OcclusionCullPass::DrawModel(*render_context, queryCommandBuffer, m_mainCamera, &cullOffNodes, true);
        } else if (g_proxyMode == RenderSetting::ProxyMode::AABB) {
            OcclusionCullPass::DrawProxy(*render_context, queryCommandBuffer, m_mainCamera, m_unitCube, &opaqueNodes, true);
            OcclusionCullPass::DrawProxy(*render_context, queryCommandBuffer, m_mainCamera, m_unitCube, &cullOffNodes, true);
        }

        int queryCount = OcclusionCullPass::EndPass(queryCommandBuffer);
        vkCmdCopyQueryPoolResults(queryCommandBuffer.get_handle(), g_queryPool[render_context->get_active_frame_index()]->get_handle(),
            0, queryCount, g_visibleReadbackBuffer->get_handle(), render_context->get_active_frame_index() * sizeof(uint32_t) * g_maxVisibleQueryCount,
            sizeof(uint32_t), VK_QUERY_RESULT_WAIT_BIT);
        queryCommandBuffer.end();

        VkFence fence = render_context->submit(render_context->get_queue(), {&queryCommandBuffer});

        Timer time;
        time.start();
        if (g_queryMode == RenderSetting::QueryMode::Immediate) {
            vkWaitForFences(device->get_handle(), 1, &fence, true, UINT64_MAX);
            VkResult result = g_queryPool[render_context->get_active_frame_index()]->get_results(0, queryCount, sizeof(uint32_t)* queryCount, g_visibleResultBuffer.get(), sizeof(uint32_t), VK_QUERY_RESULT_WAIT_BIT);

            m_profilerData.occlusionWaitTime += (float)time.stop() * 1e3f;
            if (result != VK_SUCCESS) {
                LOGE("Vulkan Error : Occlusion test return unsuccess result : {}", vkb::to_string(result));
                VK_CHECK(result);
            }

            int visibleCount = 0;
            for (int i = 0; i < queryCount; i++) {
                if (g_visibleResultBuffer[i] != 0) {
                    visibleCount++;
                }
            }
            m_profilerData.occlusionTestCount = queryCount;
            m_profilerData.occlusionVisibleCount = visibleCount;
            m_profilerData.actualDrawCount = visibleCount;
        } else if (g_queryMode == RenderSetting::QueryMode::Delay) {
            if (!m_firstFrame) {
                int lastFrameIndex = (render_context->get_active_frame_index() + (3 - g_delayFrame)) % 3;
                uint32_t lastFrameQueryCount = (uint32_t)g_visibleNodeMap[lastFrameIndex].size();
                // TODO : this is no a safe way to get the query result, we can't make sure that previous frame's command all done on gpu, unless we wait fence
                // It will make a flickering result
                uint32_t* resultPointer = (uint32_t*)g_visibleReadbackBuffer->map();
                resultPointer += lastFrameIndex * g_maxVisibleQueryCount;
                m_profilerData.occlusionWaitTime += (float)time.stop() * 1e3f;

                int visibleCount = 0;
                for (uint32_t i = 0; i < lastFrameQueryCount; i++) {
                    if (resultPointer[i] != 0) {
                        visibleCount++;
                    }
                }
                m_profilerData.occlusionTestCount = lastFrameQueryCount;
                m_profilerData.occlusionVisibleCount = visibleCount;
                m_profilerData.actualDrawCount = visibleCount;
            }
        }
        float t = (float)m_timer.elapsed() * 1e3f;
        m_profilerData.occlusionCullTime += t;
        m_timer.lap();
    }

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

    MainPass::DrawOpaque(*render_context, commandBuffer, m_mainCamera, &opaqueNodes, scene.get(), true);
    MainPass::DrawAlphaTest(*render_context, commandBuffer, m_mainCamera, &cullOffNodes, scene.get(), true);

    if (m_profilerData.drawAABB) {
        MainPass::DrawDebugAABB(*render_context, commandBuffer, m_mainCamera, m_unitCube, opaqueBoundingBoxs, glm::vec3(0.0, 1.0, 0.0));
        MainPass::DrawDebugAABB(*render_context, commandBuffer, m_mainCamera, m_unitCube, alphaTestBoundingBoxs, glm::vec3(0.0, 0.0, 1.0));
    }
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

    m_firstFrame = false;
    m_profilerData.renderDrawTime += (float)m_timer.elapsed() * 1e3f;
    m_timer.stop();
    m_profilerData.frameCount++;
    render_context->submit(commandBuffer);
}

std::unique_ptr<vkb::Application> create_gpu_occlusion() {
    return std::make_unique<gpu_occlusion>();
}
