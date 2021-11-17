#include "occlusion_cull_pass.h"
#include "graphic_context.h"
#include "common/vk_initializers.h"

namespace OcclusionCullPass {
    using namespace GraphicContext;
    using namespace GraphicResources;
    using namespace RenderSetting;
    using namespace vkb;

    PipelineState simplePipeline;

    PipelineLayout* simpleLayout{nullptr};

    int queryIndex = 0;

    void Init(Device & device) {
        std::vector<ShaderModule*> moudles{ g_shaderModules.find("depth_only.vert")->second, g_shaderModules.find("depth_only.frag")->second};
        simpleLayout = &device.get_resource_cache().request_pipeline_layout(moudles);
        moudles.clear();
    }

    void BeginPass(RenderContext & context, CommandBuffer & commandBuffer) {
        const auto& extent3d = g_visibleBuffer->get_extent();
        VkExtent2D extent{extent3d.width, extent3d.height};

        std::vector<LoadStoreInfo> loadStoreInfos;
        loadStoreInfos.emplace_back(LoadStoreInfo{VK_ATTACHMENT_LOAD_OP_CLEAR, VK_ATTACHMENT_STORE_OP_DONT_CARE});

        std::vector<SubpassInfo> subPassInfos;
        subPassInfos.emplace_back(SubpassInfo{{}, {}, {}, false, 0, VK_RESOLVE_MODE_NONE});

        std::vector<Attachment> attachments;
        attachments.emplace_back(Attachment{g_visibleBufferView->get_format(), g_visibleBuffer->get_sample_count(), g_visibleBuffer->get_usage()});

        std::vector<core::ImageView*> imageViews;
        imageViews.push_back(g_visibleBufferView.get());

        auto &renderPass = context.get_device().get_resource_cache().request_render_pass(attachments, loadStoreInfos, subPassInfos);
        auto &frameBuffer = context.get_device().get_resource_cache().request_framebuffer(imageViews, renderPass);

        std::vector<VkClearValue> clearValue{initializers::clear_depth_stencil_value(0.0f, 0.0f)};

        VkViewport viewport{};
        viewport.width = static_cast<float>(extent.width);
        viewport.height = static_cast<float>(extent.height);
        viewport.minDepth = 0.0f;
        viewport.maxDepth = 1.0f;
        commandBuffer.set_viewport(0, {viewport});

        VkRect2D scissor{};
        scissor.extent = extent;
        commandBuffer.set_scissor(0, {scissor});

        commandBuffer.reset_query_pool(*(g_queryPool[context.get_active_frame_index()]), 0, g_maxVisibleQueryCount);
        queryIndex = 0;
        commandBuffer.begin_render_pass(extent, renderPass, frameBuffer, clearValue);
    }

    void DrawModel(RenderContext& context, CommandBuffer& commandBuffer, sg::Camera* camera, RenderUtils::SortedMeshes *subMeshes, bool query) {
        RenderUtils::BindPipelineState(commandBuffer, simplePipeline);
        commandBuffer.bind_pipeline_layout(*simpleLayout);

        auto &rendeFrame = context.get_active_frame();
        for (auto iter = subMeshes->begin(); iter != subMeshes->end(); iter++) {
            if (queryIndex >= g_maxVisibleQueryCount) {
                LOGE("Error : occlusion query count out of max limitation {}", vkb::to_string(g_maxVisibleQueryCount));
                continue;
            }

            auto node = iter->second.first;
            auto submesh = iter->second.second;

            struct {
                glm::mat4 viewProject;
                glm::mat4 model;
            } global;

            global.viewProject = RenderUtils::VulkanStyleProjection(camera->get_projection()) * camera->get_view();
            global.model = node->get_transform().get_world_matrix();

            auto globalUniform = rendeFrame.allocate_buffer(VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT, sizeof(global), 0);
            globalUniform.update(global);
            commandBuffer.bind_buffer(globalUniform.get_buffer(), globalUniform.get_offset(), globalUniform.get_size(), 0, 0, 0);

            if (query) {
                commandBuffer.begin_query(*(g_queryPool[context.get_active_frame_index()]), queryIndex, 0);
            }

            if (RenderUtils::BindVertexInput(commandBuffer, *simpleLayout, submesh)) {
                commandBuffer.draw_indexed(submesh->vertex_indices, 1, 0, 0, 0);
            } else {
                commandBuffer.draw(submesh->vertices_count, 1, 0, 0);
            }
            if (query) {
                commandBuffer.end_query(*(g_queryPool[context.get_active_frame_index()]), queryIndex++);
            }
        }
    }

    void DrawProxy(RenderContext& context, CommandBuffer& commandBuffer, sg::Camera* camera, sg::SubMesh* proxy, RenderUtils::SortedMeshes *subMeshes, bool query) {
        RenderUtils::BindPipelineState(commandBuffer, simplePipeline);
        commandBuffer.bind_pipeline_layout(*simpleLayout);

        for (auto iter = subMeshes->begin(); iter != subMeshes->end(); iter++) {
            auto node = iter->second.first;
            const auto& aabb = node->get_component<sg::AABB>();

            struct {
                glm::mat4 viewProject;
                glm::mat4 model;
            } global;
            global.viewProject = RenderUtils::VulkanStyleProjection(camera->get_projection()) * camera->get_view();
            global.model = glm::translate(aabb.get_center()) * glm::scale(aabb.get_scale() * 0.5f);

            auto &render_frame = context.get_active_frame();
            auto globalUniform = render_frame.allocate_buffer(VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT, sizeof(global), 0);
            globalUniform.update(global);
            commandBuffer.bind_buffer(globalUniform.get_buffer(), globalUniform.get_offset(), globalUniform.get_size(), 0, 0, 0);

            if (query) {
                commandBuffer.begin_query(*(g_queryPool[context.get_active_frame_index()]), queryIndex, 0);
            }
            if (RenderUtils::BindVertexInput(commandBuffer, *simpleLayout, proxy)) {
                commandBuffer.draw_indexed(proxy->vertex_indices, 1, 0, 0, 0);
            } else {
                commandBuffer.draw(proxy->vertices_count, 1, 0, 0);
            }
            if (query) {
                commandBuffer.end_query(*(g_queryPool[context.get_active_frame_index()]), queryIndex++);
            }
        }
    }

    int EndPass(CommandBuffer & commandBuffer) {
        commandBuffer.end_render_pass();
        return queryIndex;
    }
}