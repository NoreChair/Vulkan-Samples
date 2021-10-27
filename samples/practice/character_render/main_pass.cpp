#include "main_pass.h"
#include "graphic_context.h"

namespace MainPass {
    using namespace vkb;

    PipelineState pipelineState;
    PipelineState skyPipelineState;
    PipelineLayout* pipelineLayout{nullptr};
    PipelineLayout* skyLayout{nullptr};

    void Init(vkb::Device& device) {
        ColorBlendState           defaultColorState;
        ColorBlendAttachmentState defaultAttaState;
        MultisampleState          multiSampleState;
        multiSampleState.rasterization_samples = RenderSetting::g_multiSampleCount;
        defaultColorState.attachments.push_back(defaultAttaState);
        pipelineState.set_color_blend_state(defaultColorState);
        pipelineState.set_multisample_state(multiSampleState);

        RasterizationState rasterizationState;
        rasterizationState.cull_mode = VK_CULL_MODE_FRONT_BIT;
        DepthStencilState depthState;
        depthState.depth_compare_op = VK_COMPARE_OP_GREATER_OR_EQUAL;
        skyPipelineState.set_rasterization_state(rasterizationState);
        skyPipelineState.set_depth_stencil_state(depthState);
        skyPipelineState.set_color_blend_state(defaultColorState);
        skyPipelineState.set_multisample_state(multiSampleState);

        std::vector<vkb::ShaderModule*> moudles;
        moudles.push_back(GraphicResources::g_shaderModules.find("character.vert")->second);
        moudles.push_back(GraphicResources::g_shaderModules.find("character.frag")->second);
        pipelineLayout = &device.get_resource_cache().request_pipeline_layout(moudles);

        moudles.clear();
        moudles.push_back(GraphicResources::g_shaderModules.find("sky.vert")->second);
        moudles.push_back(GraphicResources::g_shaderModules.find("sky.frag")->second);
        skyLayout = &device.get_resource_cache().request_pipeline_layout(moudles);
    }

    void Draw(vkb::RenderContext& context, vkb::CommandBuffer& commandBuffer, vkb::sg::Camera* camera, ShadowCamera* shadowCamera, RenderUtils::SortedMeshes *submeshs, vkb::sg::Scene* scene) {
        using namespace GraphicResources;
        RenderUtils::BindPipelineState(commandBuffer, pipelineState);

        sg::Light* mainLight = scene->get_components<sg::Light>()[0];
        auto &render_frame = context.get_active_frame();

        commandBuffer.bind_pipeline_layout(*pipelineLayout);

        struct {
            glm::mat4 view_project;
            glm::mat4 shadow_vp;
            glm::vec4 light_dir_intensity;
            glm::vec4 light_color;
            glm::vec4 camera_pos_ws;
            glm::vec4 shadow_extent;
            float pbr_roughtness;
            float pbr_metalness;
            uint32_t use_color_bleed_AO;
            uint32_t use_double_specular;
            uint32_t use_sss;
            uint32_t only_SSS;
            uint32_t only_shadow;
        } global;

        global.view_project = RenderUtils::VulkanStyleProjection(camera->get_projection()) * camera->get_view();
        global.shadow_vp = RenderUtils::VulkanStyleProjection(shadowCamera->get_projection()) * shadowCamera->get_view();
        global.light_dir_intensity = glm::vec4(mainLight->get_properties().direction, mainLight->get_properties().intensity);
        global.light_color = glm::vec4(mainLight->get_properties().color, 1.0);
        global.camera_pos_ws = -camera->get_view()[3];
        global.shadow_extent.x = (float)GraphicContext::g_shadowImage->get_extent().width;
        global.shadow_extent.y = (float)GraphicContext::g_shadowImage->get_extent().height;
        global.shadow_extent.zw = glm::vec2(1.0f) / global.shadow_extent.xy;
        global.pbr_roughtness = RenderSetting::g_roughness;
        global.pbr_metalness = RenderSetting::g_metalness;
        global.use_color_bleed_AO = RenderSetting::g_useColorBleedAO ? 1 : 0;
        global.use_double_specular = RenderSetting::g_useDoubleSpecular ? 1 : 0;
        global.use_sss = RenderSetting::g_useScreenSpaceSSS ? 1 : 0;
        global.only_SSS = RenderSetting::g_onlySSS ? 1 : 0;
        global.only_shadow = RenderSetting::g_onlyShadow ? 1 : 0;

        auto globalUniform = render_frame.allocate_buffer(VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT, sizeof(global), 0);
        globalUniform.update(global);
        commandBuffer.bind_buffer(globalUniform.get_buffer(), globalUniform.get_offset(), globalUniform.get_size(), 0, 0, 0);

        commandBuffer.bind_image(*g_sceneTextures[Albedo], *GraphicContext::g_linearClampSampler, 0, 2, 0);
        commandBuffer.bind_image(*g_sceneTextures[Normal], *GraphicContext::g_linearClampSampler, 0, 3, 0);
        commandBuffer.bind_image(*g_sceneTextures[Occlusion], *GraphicContext::g_linearClampSampler, 0, 4, 0);
        commandBuffer.bind_image(*g_sceneTextures[Detail], *GraphicContext::g_linearClampSampler, 0, 5, 0);
        commandBuffer.bind_image(*GraphicContext::g_characterSSSView, *GraphicContext::g_linearClampSampler, 0, 6, 0);
        commandBuffer.bind_image(GraphicContext::g_shadowImage->get_views()[0], *GraphicContext::g_shadowSampler, 0, 7, 0);
        commandBuffer.bind_image(*g_sceneTextures[Irradiance], *GraphicContext::g_linearClampSampler, 0, 8, 0);
        commandBuffer.bind_image(*g_sceneTextures[Radiance], *GraphicContext::g_linearClampSampler, 0, 9, 0);

        for (auto iter = submeshs->begin(); iter != submeshs->end(); iter++) {
            auto node = iter->second.first;
            auto submesh = iter->second.second;

            struct {
                glm::mat4 model;
                glm::mat4 inv_model;
            } uniform;

            auto &transform = node->get_transform();
            uniform.model = transform.get_world_matrix();
            uniform.inv_model = glm::inverse(transform.get_world_matrix());

            auto  allocation = render_frame.allocate_buffer(VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT, sizeof(uniform), 0);
            allocation.update(uniform);
            commandBuffer.bind_buffer(allocation.get_buffer(), allocation.get_offset(), allocation.get_size(), 0, 1, 0);

            if (RenderUtils::BindVertexInput(commandBuffer, *pipelineLayout, submesh)) {
                commandBuffer.draw_indexed(submesh->vertex_indices, 1, 0, 0, 0);
            } else {
                commandBuffer.draw(submesh->vertices_count, 1, 0, 0);
            }
        }
    }

    void DrawSky(vkb::RenderContext& context, vkb::CommandBuffer& commandBuffer, vkb::sg::Camera* camera, vkb::sg::SubMesh* skyMesh) {
        RenderUtils::BindPipelineState(commandBuffer, skyPipelineState);
        commandBuffer.bind_pipeline_layout(*skyLayout);

        struct{
            glm::mat4 view_proj;
        }global;

        glm::mat4 v =camera->get_view();
        v[3] = glm::vec4(0.0, 0.0, 0.0, 1.0);
        global.view_proj = RenderUtils::VulkanStyleProjection(camera->get_projection()) * v;

        auto &render_frame = context.get_active_frame();
        auto globalUniform = render_frame.allocate_buffer(VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT, sizeof(global), 0);
        globalUniform.update(global);
        commandBuffer.bind_buffer(globalUniform.get_buffer(), globalUniform.get_offset(), globalUniform.get_size(), 0, 0, 0);
        commandBuffer.bind_image(*GraphicResources::g_sceneTextures[GraphicResources::SkyBox], *GraphicContext::g_linearClampSampler, 0, 1, 0);

        if (RenderUtils::BindVertexInput(commandBuffer, *skyLayout, skyMesh)) {
            commandBuffer.draw_indexed(skyMesh->vertex_indices, 1, 0, 0, 0);
        } else {
            commandBuffer.draw(skyMesh->vertices_count, 1, 0, 0);
        }
    }
}