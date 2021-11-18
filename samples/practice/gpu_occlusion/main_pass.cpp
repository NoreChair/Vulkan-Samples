#include "main_pass.h"
#include "graphic_context.h"
#include "common/vk_initializers.h"
#include "scene_graph/components/pbr_material.h"

using namespace vkb;
using namespace GraphicContext;
using namespace GraphicResources;
using namespace RenderSetting;

namespace MainPass {
    PipelineState rockPipelineState;
    PipelineState grassPipelineState;
    PipelineState skyPipelineState;
    PipelineState debugDrawState;

    PipelineLayout* rockLayout{nullptr};
    PipelineLayout* grassLayout{nullptr};
    PipelineLayout* skyLayout{nullptr};
    PipelineLayout* debugDrawLayout{nullptr};

    void Init(vkb::Device& device) {
        ColorBlendState           defaultColorState;
        ColorBlendAttachmentState defaultAttaState;
        defaultColorState.attachments.push_back(defaultAttaState);
        rockPipelineState.set_color_blend_state(defaultColorState);
        grassPipelineState.set_color_blend_state(defaultColorState);

        RasterizationState rasterizationState;
        rasterizationState.cull_mode = VK_CULL_MODE_FRONT_BIT;
        DepthStencilState depthState;
        depthState.depth_compare_op = VK_COMPARE_OP_GREATER_OR_EQUAL;
        skyPipelineState.set_rasterization_state(rasterizationState);
        skyPipelineState.set_depth_stencil_state(depthState);
        skyPipelineState.set_color_blend_state(defaultColorState);

        RasterizationState rasterState;
        rasterState.polygon_mode = VK_POLYGON_MODE_LINE;
        rasterState.cull_mode = VK_CULL_MODE_NONE;
        debugDrawState.set_rasterization_state(rasterState);
        debugDrawState.set_color_blend_state(defaultColorState);
        depthState.depth_write_enable = false;
        debugDrawState.set_depth_stencil_state(depthState);

        VertexInputState vertexInputState;
        vertexInputState.bindings.emplace_back(VkVertexInputBindingDescription{0, sizeof(float) * 3, VK_VERTEX_INPUT_RATE_VERTEX});
        vertexInputState.attributes.emplace_back(VkVertexInputAttributeDescription{0, 0, VK_FORMAT_R32G32B32_SFLOAT, 0});
        vertexInputState.bindings.emplace_back(VkVertexInputBindingDescription{1, sizeof(glm::vec3) * 3, VK_VERTEX_INPUT_RATE_INSTANCE});
        vertexInputState.attributes.emplace_back(VkVertexInputAttributeDescription{1, 1, VK_FORMAT_R32G32B32_SFLOAT, 0});
        vertexInputState.attributes.emplace_back(VkVertexInputAttributeDescription{2, 1, VK_FORMAT_R32G32B32_SFLOAT, sizeof(glm::vec3)});
        vertexInputState.attributes.emplace_back(VkVertexInputAttributeDescription{3, 1, VK_FORMAT_R32G32B32_SFLOAT, sizeof(glm::vec3) * 2});
        debugDrawState.set_vertex_input_state(vertexInputState);

        std::vector<vkb::ShaderModule*> moudles;
        moudles.push_back(GraphicResources::g_shaderModules.find("rock.vert")->second);
        moudles.push_back(GraphicResources::g_shaderModules.find("rock.frag")->second);
        rockLayout = &device.get_resource_cache().request_pipeline_layout(moudles);

        moudles.clear();
        moudles.push_back(GraphicResources::g_shaderModules.find("grass.vert")->second);
        moudles.push_back(GraphicResources::g_shaderModules.find("grass.frag")->second);
        grassLayout = &device.get_resource_cache().request_pipeline_layout(moudles);

        moudles.clear();
        moudles.push_back(GraphicResources::g_shaderModules.find("sky.vert")->second);
        moudles.push_back(GraphicResources::g_shaderModules.find("sky.frag")->second);
        skyLayout = &device.get_resource_cache().request_pipeline_layout(moudles);

        moudles.clear();
        moudles.push_back(GraphicResources::g_shaderModules.find("debug_draw.vert")->second);
        moudles.push_back(GraphicResources::g_shaderModules.find("debug_draw.frag")->second);
        debugDrawLayout = &device.get_resource_cache().request_pipeline_layout(moudles);
    }

    void BeginPass(vkb::RenderContext& context, vkb::CommandBuffer& commandBuffer) {
        auto& targetView = const_cast<core::ImageView&>(context.get_active_frame().get_render_target().get_views()[0]);
        const auto& extent3d = targetView.get_image().get_extent();
        VkExtent2D extent{extent3d.width, extent3d.height};

        std::vector<LoadStoreInfo> loadStoreInfos;
        loadStoreInfos.emplace_back(LoadStoreInfo{VK_ATTACHMENT_LOAD_OP_CLEAR, VK_ATTACHMENT_STORE_OP_STORE});
        loadStoreInfos.emplace_back(LoadStoreInfo{VK_ATTACHMENT_LOAD_OP_CLEAR, VK_ATTACHMENT_STORE_OP_DONT_CARE});

        std::vector<SubpassInfo> subPassInfos;
        subPassInfos.emplace_back(SubpassInfo{{}, {0}, {}, false, 0, VK_RESOLVE_MODE_NONE});

        std::vector<Attachment> attachments;
        attachments.emplace_back(Attachment{targetView.get_format(), targetView.get_image().get_sample_count(), targetView.get_image().get_usage()});
        attachments.emplace_back(Attachment{g_sceneDepthView->get_format(), g_sceneDepth->get_sample_count(), g_sceneDepth->get_usage()});

        std::vector<core::ImageView*> imageViews;
        imageViews.push_back(&targetView);
        imageViews.push_back(g_sceneDepthView.get());

        auto &renderPass = context.get_device().get_resource_cache().request_render_pass(attachments, loadStoreInfos, subPassInfos);
        auto &frameBuffer = context.get_device().get_resource_cache().request_framebuffer(imageViews, renderPass);

        std::vector<VkClearValue> clearValue{initializers::clear_color_value(0.0f, 0.0f, 0.0f, 0.0f), initializers::clear_depth_stencil_value(0.0f, 0)};

        VkViewport viewport{};
        viewport.width = static_cast<float>(extent.width);
        viewport.height = static_cast<float>(extent.height);
        viewport.minDepth = 0.0f;
        viewport.maxDepth = 1.0f;
        commandBuffer.set_viewport(0, {viewport});

        VkRect2D scissor{};
        scissor.extent = extent;
        commandBuffer.set_scissor(0, {scissor});

        commandBuffer.begin_render_pass(extent, renderPass, frameBuffer, clearValue);
    }

    void DrawOpaque(vkb::RenderContext& context, vkb::CommandBuffer& commandBuffer, vkb::sg::Camera* camera, RenderUtils::SortedMeshes *subMeshes, vkb::sg::Scene* scene, bool query) {
        RenderUtils::BindPipelineState(commandBuffer, rockPipelineState);

        sg::Light* mainLight = scene->get_components<sg::Light>()[0];
        auto &render_frame = context.get_active_frame();

        struct {
            glm::mat4 viewProject;
            glm::vec4 lightDirIntensity;
            glm::vec4 lightColor;
            glm::vec4 cameraPos;
        } global;

        global.viewProject = RenderUtils::VulkanStyleProjection(camera->get_projection()) * camera->get_view();
        global.lightDirIntensity = glm::vec4(mainLight->get_properties().direction, mainLight->get_properties().intensity);
        global.lightColor = glm::vec4(mainLight->get_properties().color, 1.0);
        global.cameraPos = -camera->get_view()[3];

        auto globalUniform = render_frame.allocate_buffer(VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT, sizeof(global), 0);
        globalUniform.update(global);
        commandBuffer.bind_buffer(globalUniform.get_buffer(), globalUniform.get_offset(), globalUniform.get_size(), 0, 0, 0);

        commandBuffer.bind_pipeline_layout(*rockLayout);

        for (auto iter = subMeshes->begin(); iter != subMeshes->end(); iter++) {
            auto node = iter->second.first;
            auto submesh = iter->second.second;

            if (query && g_queryMode != QueryMode::None) {
                int frameIndex = context.get_active_frame_index();
                int lastFrameIndex = (frameIndex + (3 - g_delayFrame)) % 3;
                std::unordered_map<sg::Node*, uint32_t>& visibleMap = g_visibleNodeMap[g_queryMode == QueryMode::Immediate ? frameIndex : lastFrameIndex];
                if (visibleMap.find(node) != visibleMap.end()) {
                    int index = visibleMap[node];
                    if (g_queryMode == QueryMode::Delay) {
                        index += lastFrameIndex * g_maxVisibleQueryCount;
                    }
                    if (g_visibleResultBuffer[index] == 0) {
                        continue;
                    }
                }
            }

            auto material = (sg::PBRMaterial*) submesh->get_material();

            struct {
                glm::mat4 model;
                glm::mat4 invModel;
                glm::vec4 baseColor;
            } uniform;

            auto &transform = node->get_transform();
            uniform.model = transform.get_world_matrix();
            uniform.invModel = glm::inverse(transform.get_world_matrix());
            uniform.baseColor = material->base_color_factor;

            auto  allocation = render_frame.allocate_buffer(VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT, sizeof(uniform), 0);
            allocation.update(uniform);
            commandBuffer.bind_buffer(allocation.get_buffer(), allocation.get_offset(), allocation.get_size(), 0, 1, 0);

            DescriptorSetLayout &descriptorSetLayout = rockLayout->get_descriptor_set_layout(0);
            for (auto &texture : submesh->get_material()->textures) {
                auto layoutBinding = descriptorSetLayout.get_layout_binding(texture.first);
                if (layoutBinding != nullptr) {
                    commandBuffer.bind_image(texture.second->get_image()->get_vk_image_view(),
                        texture.second->get_sampler()->vk_sampler, 0, layoutBinding->binding, 0);
                }
            }

            if (RenderUtils::BindVertexInput(commandBuffer, *rockLayout, submesh)) {
                commandBuffer.draw_indexed(submesh->vertex_indices, 1, 0, 0, 0);
            } else {
                commandBuffer.draw(submesh->vertices_count, 1, 0, 0);
            }
        }
    }

    void DrawAlphaTest(vkb::RenderContext & context, vkb::CommandBuffer & commandBuffer, vkb::sg::Camera * camera, RenderUtils::SortedMeshes * subMeshes, vkb::sg::Scene * scene, bool query) {
        RenderUtils::BindPipelineState(commandBuffer, grassPipelineState);

        sg::Light* mainLight = scene->get_components<sg::Light>()[0];
        auto &render_frame = context.get_active_frame();

        struct {
            glm::mat4 viewProject;
            glm::vec4 lightDirIntensity;
            glm::vec4 lightColor;
            glm::vec4 cameraPos;
        } global;

        global.viewProject = RenderUtils::VulkanStyleProjection(camera->get_projection()) * camera->get_view();
        global.lightDirIntensity = glm::vec4(mainLight->get_properties().direction, mainLight->get_properties().intensity);
        global.lightColor = glm::vec4(mainLight->get_properties().color, 1.0);
        global.cameraPos = -camera->get_view()[3];

        auto globalUniform = render_frame.allocate_buffer(VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT, sizeof(global), 0);
        globalUniform.update(global);
        commandBuffer.bind_buffer(globalUniform.get_buffer(), globalUniform.get_offset(), globalUniform.get_size(), 0, 0, 0);

        commandBuffer.bind_pipeline_layout(*grassLayout);

        for (auto iter = subMeshes->begin(); iter != subMeshes->end(); iter++) {
            auto node = iter->second.first;
            auto submesh = iter->second.second;

            if (query && g_queryMode != QueryMode::None) {
                int frameIndex = context.get_active_frame_index();
                int lastFrameIndex = (frameIndex + (3 - g_delayFrame)) % 3;
                std::unordered_map<sg::Node*, uint32_t>& visibleMap = g_visibleNodeMap[g_queryMode == QueryMode::Immediate ? frameIndex : lastFrameIndex];
                if (visibleMap.find(node) != visibleMap.end()) {
                    int index = visibleMap[node];
                    if (g_queryMode == QueryMode::Delay) {
                        index += lastFrameIndex * g_maxVisibleQueryCount;
                    }
                    if (g_visibleResultBuffer[index] == 0) {
                        continue;
                    }
                }
            }

            auto material = (sg::PBRMaterial*) submesh->get_material();

            struct {
                glm::mat4 model;
                glm::mat4 invModel;
                glm::vec4 baseColor;
            } uniform;

            auto &transform = node->get_transform();
            uniform.model = transform.get_world_matrix();
            uniform.invModel = glm::inverse(transform.get_world_matrix());
            uniform.baseColor = material->base_color_factor;

            auto  allocation = render_frame.allocate_buffer(VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT, sizeof(uniform), 0);
            allocation.update(uniform);
            commandBuffer.bind_buffer(allocation.get_buffer(), allocation.get_offset(), allocation.get_size(), 0, 1, 0);

            DescriptorSetLayout &descriptorSetLayout = rockLayout->get_descriptor_set_layout(0);
            for (auto &texture : submesh->get_material()->textures) {
                auto layoutBinding = descriptorSetLayout.get_layout_binding(texture.first);
                if (layoutBinding != nullptr) {
                    commandBuffer.bind_image(texture.second->get_image()->get_vk_image_view(),
                        texture.second->get_sampler()->vk_sampler, 0, layoutBinding->binding, 0);
                }
            }

            if (RenderUtils::BindVertexInput(commandBuffer, *rockLayout, submesh)) {
                commandBuffer.draw_indexed(submesh->vertex_indices, 1, 0, 0, 0);
            } else {
                commandBuffer.draw(submesh->vertices_count, 1, 0, 0);
            }
        }
    }

    void DrawDebugAABB(vkb::RenderContext & context, vkb::CommandBuffer & commandBuffer, vkb::sg::Camera * camera, vkb::sg::SubMesh * debugMesh, const std::vector<std::reference_wrapper<const vkb::sg::AABB>>& aabbs, glm::vec3 color) {
        if (aabbs.size() == 0) {
            return;
        }

        RenderUtils::BindPipelineState(commandBuffer, debugDrawState);
        commandBuffer.bind_pipeline_layout(*debugDrawLayout);

        struct {
            glm::mat4 view_proj;
        }global;
        global.view_proj = RenderUtils::VulkanStyleProjection(camera->get_projection()) * camera->get_view();

        struct Instance {
            glm::vec3 translate;
            glm::vec3 scale;
            glm::vec3 color;
        };
        std::vector<Instance> instanceData;
        instanceData.reserve(aabbs.size());

        for (int i = 0; i < aabbs.size(); i++) {
            const auto& aabb = aabbs[i].get();
            instanceData.emplace_back(Instance{aabb.get_center(),(aabb.get_max() - aabb.get_min()) * 0.5f, color});
        }

        auto &rendeFrame = context.get_active_frame();

        auto globalUniform = rendeFrame.allocate_buffer(VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT, sizeof(global), 0);
        globalUniform.update(global);
        commandBuffer.bind_buffer(globalUniform.get_buffer(), globalUniform.get_offset(), globalUniform.get_size(), 0, 0, 0);

        auto instanceBuffer = rendeFrame.allocate_buffer(VK_BUFFER_USAGE_VERTEX_BUFFER_BIT, sizeof(Instance) * aabbs.size());
        instanceBuffer.update((uint8_t *)instanceData.data(), (uint32_t)instanceData.size() * sizeof(Instance), 0);

        std::vector<std::reference_wrapper<const core::Buffer>> buffers;
        auto vert_iter = debugMesh->vertex_buffers.find(std::string("position"));
        buffers.emplace_back(std::reference_wrapper<const core::Buffer>(std::ref(vert_iter->second)));
        buffers.emplace_back(std::reference_wrapper<const core::Buffer>(std::ref(instanceBuffer.get_buffer())));
        commandBuffer.bind_vertex_buffers(0, buffers, {0, instanceBuffer.get_offset()});
        if (debugMesh->vertex_indices != 0) {
            commandBuffer.bind_index_buffer(*debugMesh->index_buffer, debugMesh->index_offset, debugMesh->index_type);
            commandBuffer.draw_indexed(debugMesh->vertex_indices, (uint32_t)aabbs.size(), 0, 0, 0);
        } else {
            commandBuffer.draw(debugMesh->vertices_count, (uint32_t)aabbs.size(), 0, 0);
        }

    }

    void DrawSky(vkb::RenderContext& context, vkb::CommandBuffer& commandBuffer, vkb::sg::Camera* camera, vkb::sg::SubMesh* skyMesh, vkb::sg::Material* material) {
        RenderUtils::BindPipelineState(commandBuffer, skyPipelineState);
        commandBuffer.bind_pipeline_layout(*skyLayout);

        struct {
            glm::mat4 view_proj;
        }global;

        glm::mat4 v = camera->get_view();
        v[3] = glm::vec4(0.0, 0.0, 0.0, 1.0);
        global.view_proj = RenderUtils::VulkanStyleProjection(camera->get_projection()) * v;

        auto &rendeFrame = context.get_active_frame();
        auto globalUniform = rendeFrame.allocate_buffer(VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT, sizeof(global), 0);
        globalUniform.update(global);
        commandBuffer.bind_buffer(globalUniform.get_buffer(), globalUniform.get_offset(), globalUniform.get_size(), 0, 0, 0);
        commandBuffer.bind_image(material->textures[0]->get_image()->get_vk_image_view(), *GraphicContext::g_linearClampSampler, 0, 1, 0);

        if (RenderUtils::BindVertexInput(commandBuffer, *skyLayout, skyMesh)) {
            commandBuffer.draw_indexed(skyMesh->vertex_indices, 1, 0, 0, 0);
        } else {
            commandBuffer.draw(skyMesh->vertices_count, 1, 0, 0);
        }
    }

    void EndPass(vkb::CommandBuffer& commandBuffer) {
        commandBuffer.end_render_pass();
    }
}
