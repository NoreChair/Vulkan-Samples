#include "main_pass.h"
#include "graphic_context.h"
#include "render_utils.h"

namespace MainPass {
    using namespace vkb;

    PipelineState pipelineState;
    PipelineLayout* pipelineLayout{nullptr};

    void Init(vkb::Device& device) {
        //DepthStencilState defaultDepthState;
        //pipelineState.set_depth_stencil_state(defaultDepthState);
        //RasterizationState rasterState;
        //pipelineState.set_rasterization_state(rasterState);
        ColorBlendState           defaultColorState;
        ColorBlendAttachmentState defaultAttaState;
        defaultColorState.attachments.push_back(defaultAttaState);
        pipelineState.set_color_blend_state(defaultColorState);

        std::vector<vkb::ShaderModule*> moudles;
        moudles.push_back(ShaderProgram::g_shaderModules.find("character.vert")->second);
        moudles.push_back(ShaderProgram::g_shaderModules.find("character.frag")->second);
        pipelineLayout = &device.get_resource_cache().request_pipeline_layout(moudles);
    }

    void Draw(vkb::RenderContext& context, vkb::CommandBuffer& commandBuffer, vkb::sg::Camera* camera, RenderUtils::SortedMeshes *submeshs, vkb::sg::Scene* scene) {
        RenderUtils::BindPipelineState(commandBuffer, pipelineState);

        const auto& images = scene->get_components<sg::Image>();
        sg::Image* color{nullptr}; sg::Image* normal{nullptr}; sg::Image* params{nullptr};
        for (int i = 0; i < images.size(); ++i) {
            if (images.at(i)->get_name() == "T_Color.png") {
                color = images.at(i);
            }
            if (images.at(i)->get_name() == "T_N.png") {
                normal = images.at(i);
            }
            if (images.at(i)->get_name() == "T_AO_SSS_CA.png") {
                params = images.at(i);
            }
            if (color && normal && params) {
                break;
            }
        }
        assert(color && normal && params);

        sg::Light* mainLight = scene->get_components<sg::Light>()[0];

        for (auto iter = submeshs->begin(); iter != submeshs->end(); iter++) {
            auto node = iter->second.first;
            auto submesh = iter->second.second;

            commandBuffer.bind_pipeline_layout(*pipelineLayout);

            struct {
                glm::mat4 model;
                glm::mat4 view_project;
                glm::mat4 inv_model;
                glm::vec4 light_dir_intensity;
            } uniform;

            auto &render_frame = context.get_active_frame();
            auto  allocation = render_frame.allocate_buffer(VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT, sizeof(uniform), 0);
            auto &transform = node->get_transform();
            
            uniform.model = transform.get_world_matrix();
            uniform.view_project = RenderUtils::VulkanStyleProjection(camera->get_projection()) * camera->get_view();
            uniform.inv_model = glm::inverse(transform.get_world_matrix());
            uniform.light_dir_intensity = glm::vec4(mainLight->get_properties().direction, mainLight->get_properties().intensity);

            allocation.update(uniform);
            commandBuffer.bind_buffer(allocation.get_buffer(), allocation.get_offset(), allocation.get_size(), 0, 0, 0);

            commandBuffer.bind_image(color->get_vk_image_view(), *GraphicContext::g_linearClampSampler, 0, 1, 0);
            commandBuffer.bind_image(normal->get_vk_image_view(), *GraphicContext::g_linearClampSampler, 0, 2, 0);
            commandBuffer.bind_image(params->get_vk_image_view(), *GraphicContext::g_linearClampSampler, 0, 3, 0);

            if (RenderUtils::BindVertexInput(commandBuffer, *pipelineLayout, submesh)) {
                commandBuffer.draw_indexed(submesh->vertex_indices, 1, 0, 0, 0);
            } else {
                commandBuffer.draw(submesh->vertices_count, 1, 0, 0);
            }
        }
    }
}