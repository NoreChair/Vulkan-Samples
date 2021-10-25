#include "shadow_pass.h"
#include "graphic_context.h"

namespace ShadowPass {
    using namespace vkb;

    PipelineState pipelineState;
    PipelineLayout* pipelineLayout{nullptr};

    void Init(vkb::Device& device) {
        DepthStencilState depthState;
        depthState.depth_compare_op = VK_COMPARE_OP_LESS;
        pipelineState.set_depth_stencil_state(depthState);

        RasterizationState rasterState;
        rasterState.depth_bias_enable = true;
        pipelineState.set_rasterization_state(rasterState);

        std::vector<vkb::ShaderModule*> moudles;
        moudles.push_back(GraphicResources::g_shaderModules.find("depth_only.vert")->second);
        moudles.push_back(GraphicResources::g_shaderModules.find("depth_only.frag")->second);
        pipelineLayout = &device.get_resource_cache().request_pipeline_layout(moudles);
    }

    void Draw(vkb::RenderContext& context, vkb::CommandBuffer& commandBuffer, ShadowCamera* camera, RenderUtils::SortedMeshes *submeshs) {
        RenderUtils::BindPipelineState(commandBuffer, pipelineState);

        commandBuffer.set_depth_bias(RenderSetting::g_shadowBias[0], RenderSetting::g_shadowBias[1], RenderSetting::g_shadowBias[2]);
        commandBuffer.bind_pipeline_layout(*pipelineLayout);

        for (auto iter = submeshs->begin(); iter != submeshs->end(); iter++) {
            auto node = iter->second.first;
            auto submesh = iter->second.second;

            struct {
                glm::mat4 model;
                glm::mat4 view_project;
                float normal_bias;
            } uniform;

            auto &render_frame = context.get_active_frame();
            auto allocation = render_frame.allocate_buffer(VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT, sizeof(uniform), 0);
            auto &transform = node->get_transform();

            uniform.model = transform.get_world_matrix();
            uniform.view_project = RenderUtils::VulkanStyleProjection(camera->get_projection()) * camera->get_view();
            uniform.normal_bias = RenderSetting::g_shadowNormalBias;

            allocation.update(uniform);
            commandBuffer.bind_buffer(allocation.get_buffer(), allocation.get_offset(), allocation.get_size(), 0, 0, 0);

            if (RenderUtils::BindVertexInput(commandBuffer, *pipelineLayout, submesh)) {
                commandBuffer.draw_indexed(submesh->vertex_indices, 1, 0, 0, 0);
            } else {
                commandBuffer.draw(submesh->vertices_count, 1, 0, 0);
            }
        }
    }
}