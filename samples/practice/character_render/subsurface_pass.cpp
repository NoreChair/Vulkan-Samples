#include "subsurface_pass.h"
#include "graphic_context.h"
#include "common/vk_initializers.h"
#include "scene_graph/components/perspective_camera.h"
#include <vector>
namespace SubsurfacePass {

    /*                                    Gaussian Kernel                                          */
    class Gaussian {
    public:
        static std::vector<Gaussian> gaussianSum(float variances[], glm::vec3 weights[], int nVariances) {
            std::vector<Gaussian> gaussians;
            for (int i = 0; i < nVariances; i++) {
                float variance = i == 0 ? variances[i] : variances[i] - variances[i - 1];
                gaussians.push_back(Gaussian(variance, weights, i));
            }
            return gaussians;
        }

        float getWidth() const { return width; }
        glm::vec4 getWeight() const { return weight; }

        static const std::vector<Gaussian> SKIN;
        static const std::vector<Gaussian> MARBLE;
        static const std::vector<Gaussian> TRUE_SKIN;

    private:
        Gaussian() {}
        Gaussian(float variance, glm::vec3 weights[], int n) : width(sqrt(variance)) {
            glm::vec3 total = glm::vec3(0.0f, 0.0f, 0.0f);
            for (int i = 0; i < n + 2; i++) {
                total += weights[i];
            }

            weight = glm::vec4(weights[n + 1], 1.0f);
            weight.rgb *= glm::vec3(1.0f)/ total.rgb;
        }

        float width;
        glm::vec4 weight;
    };

    class SkinGaussianSum : public std::vector<Gaussian> {
    public:
        SkinGaussianSum() {
            // We use the unblurred image as an aproximation to the first 
            // gaussian because it is too narrow to be noticeable. The weight
            // of the unblurred image is the first one.
            glm::vec3 weights[] = {
                glm::vec3(0.240516183695f, 0.447403391891f, 0.615796108321f),
                glm::vec3(0.115857499765f, 0.366176401412f, 0.343917471552f),
                glm::vec3(0.183619017698f, 0.186420206697f, 0.0f),
                glm::vec3(0.460007298842f, 0.0f, 0.0402864201267f)
            };
            float variances[] = {0.0516500425655f, 0.271928080903f, 2.00626388153f};
            std::vector<Gaussian> &gaussianSum = *this;
            gaussianSum = Gaussian::gaussianSum(variances, weights, 3);
        }
    };

    class MarbleGaussianSum : public std::vector<Gaussian> {
    public:
        MarbleGaussianSum() {
            // In this case the first gaussian is wide and thus we cannot
            // approximate it with the unblurred image. For this reason the
            // first weight is set to zero.
            glm::vec3 weights[] = {
                glm::vec3(0.0f, 0.0f, 0.0f),
                glm::vec3(0.0544578254963f, 0.12454890956f, 0.217724878147f),
                glm::vec3(0.243663230592f, 0.243532369381f, 0.18904245481f),
                glm::vec3(0.310530428621f, 0.315816663292f, 0.374244725886f),
                glm::vec3(0.391348515291f, 0.316102057768f, 0.218987941157f)
            };
            float variances[] = {0.0362208693441f, 0.114450574559f, 0.455584392509f, 3.48331959682f};
            std::vector<Gaussian> &gaussianSum = *this;
            gaussianSum = Gaussian::gaussianSum(variances, weights, 4);
        }
    };

    class TrueSkinGaussianSum : public std::vector<Gaussian> {
    public:
        TrueSkinGaussianSum() {
            // This 6-gaussian sum is included for comparison purposes
            float variances[] = {0.0484f, 0.187f, 0.567f, 1.99f, 7.41f};
            glm::vec3 weights[] = {
                glm::vec3(0.233f, 0.455f, 0.649f),
                glm::vec3(0.100f, 0.336f, 0.344f),
                glm::vec3(0.118f, 0.198f, 0.0f),
                glm::vec3(0.113f, 0.007f, 0.007f),
                glm::vec3(0.358f, 0.004f, 0.0f),
                glm::vec3(0.078f, 0.0f, 0.0f)
            };

            std::vector<Gaussian> &gaussianSum = *this;
            gaussianSum = Gaussian::gaussianSum(variances, weights, 6);
        }
    };

    const std::vector<Gaussian> Gaussian::SKIN = SkinGaussianSum();
    const std::vector<Gaussian> Gaussian::MARBLE = MarbleGaussianSum();
    const std::vector<Gaussian> Gaussian::TRUE_SKIN = TrueSkinGaussianSum();

    using namespace vkb;
    using namespace GraphicContext;

    PipelineState irradiancePipeline;
    PipelineState blurPipeline;
    PipelineState blurAccumPipeline;

    PipelineLayout* irradianceLayout;
    PipelineLayout* blurLayout;
    PipelineLayout* blurAccumLayout;

    void Init(vkb::Device& device){

        ColorBlendState           colorState;
        ColorBlendAttachmentState attaState;
        colorState.attachments.push_back(attaState);
        colorState.attachments.push_back(attaState);

        DepthStencilState         depthState;
        depthState.stencil_test_enable = true;
        depthState.front.compare_op = VK_COMPARE_OP_ALWAYS;
        depthState.front.pass_op = VK_STENCIL_OP_REPLACE;
        depthState.front.fail_op = VK_STENCIL_OP_KEEP;
        depthState.front.depth_fail_op = VK_STENCIL_OP_KEEP;
        depthState.back.compare_op = VK_COMPARE_OP_NEVER;
        depthState.back.pass_op = VK_STENCIL_OP_KEEP;
        depthState.back.fail_op = VK_STENCIL_OP_KEEP;
        depthState.back.depth_fail_op = VK_STENCIL_OP_KEEP;

        irradiancePipeline.set_color_blend_state(colorState);
        irradiancePipeline.set_depth_stencil_state(depthState);

        depthState.front.compare_op = VK_COMPARE_OP_EQUAL;
        depthState.front.pass_op = VK_STENCIL_OP_KEEP;

        depthState.depth_compare_op = VK_COMPARE_OP_LESS;
        depthState.depth_write_enable = VK_FALSE;

        colorState.attachments.clear();
        colorState.attachments.push_back(attaState);
        blurPipeline.set_color_blend_state(colorState);
        blurPipeline.set_depth_stencil_state(depthState);

        colorState.attachments.clear();
        colorState.attachments.push_back(attaState);
        attaState.blend_enable = VK_TRUE;
        attaState.src_color_blend_factor = VK_BLEND_FACTOR_CONSTANT_COLOR;
        // hack : driver's bug, dst color blend factor will set to be src alpha blend factor
        attaState.src_alpha_blend_factor = VK_BLEND_FACTOR_ONE_MINUS_CONSTANT_COLOR;
        attaState.dst_color_blend_factor = VK_BLEND_FACTOR_ONE_MINUS_CONSTANT_COLOR;
        attaState.dst_alpha_blend_factor = VK_BLEND_FACTOR_ZERO;
        colorState.attachments.push_back(attaState);
        blurAccumPipeline.set_color_blend_state(colorState);
        blurAccumPipeline.set_depth_stencil_state(depthState);

        std::vector<vkb::ShaderModule*> moudles;
        moudles.push_back(GraphicResources::g_shaderModules.find("sss_irradiance.vert")->second);
        moudles.push_back(GraphicResources::g_shaderModules.find("sss_irradiance.frag")->second);
        irradianceLayout = &device.get_resource_cache().request_pipeline_layout(moudles);

        moudles.clear();
        moudles.push_back(GraphicResources::g_shaderModules.find("sss_blur.vert")->second);
        moudles.push_back(GraphicResources::g_shaderModules.find("sss_blur.frag")->second);
        blurLayout = &device.get_resource_cache().request_pipeline_layout(moudles);

        moudles.clear();
        moudles.push_back(GraphicResources::g_shaderModules.find("sss_blur.vert")->second);
        moudles.push_back(GraphicResources::g_shaderModules.find("sss_blur_accum.frag")->second);
        blurAccumLayout = &device.get_resource_cache().request_pipeline_layout(moudles);
    }
    
    void DrawDiffuse(RenderContext& context, CommandBuffer& commandBuffer, sg::Camera* camera, RenderUtils::SortedMeshes *submeshs, sg::Scene* scene) {
        RenderUtils::BindPipelineState(commandBuffer, irradiancePipeline);

        const auto& images = scene->get_components<sg::Image>();
        sg::Image* color{nullptr};
        for (int i = 0; i < images.size(); ++i) {
            if (images.at(i)->get_name() == "T_Color.png") {
                color = images.at(i);
            }
            if (color) {
                break;
            }
        }

        sg::Light* mainLight = scene->get_components<sg::Light>()[0];
        auto &render_frame = context.get_active_frame();

        commandBuffer.bind_pipeline_layout(*irradianceLayout);

        struct {
            glm::mat4 view_project;
            glm::vec4 light_dir_intensity;
            glm::vec4 light_color;
        } global;

        global.view_project = RenderUtils::VulkanStyleProjection(camera->get_projection()) * camera->get_view();
        global.light_dir_intensity = glm::vec4(mainLight->get_properties().direction, mainLight->get_properties().intensity);
        global.light_color = glm::vec4(mainLight->get_properties().color, 1.0);

        auto globalUniform = render_frame.allocate_buffer(VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT, sizeof(global), 0);
        globalUniform.update(global);
        commandBuffer.bind_buffer(globalUniform.get_buffer(), globalUniform.get_offset(), globalUniform.get_size(), 0, 0, 0);
        commandBuffer.bind_image(color->get_vk_image_view(), *g_linearClampSampler, 0, 2, 0);

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

            if (RenderUtils::BindVertexInput(commandBuffer, *irradianceLayout, submesh)) {
                commandBuffer.draw_indexed(submesh->vertex_indices, 1, 0, 0, 0);
            } else {
                commandBuffer.draw(submesh->vertices_count, 1, 0, 0);
            }
        }
    }

    float ToNDC(float d, const glm::mat4& proj) {
        float rcpd = 1.0f / glm::max(d,0.001f);
        float clipz = proj[2][2] + proj[2][3] * rcpd;
        float ndcz = clipz * rcpd * -1.0f;
        return ndcz;
    }

    void Blur(RenderContext& context, CommandBuffer& commandBuffer, int pass, const Gaussian& gaussian, const glm::mat4& proj) {
        bool firstPass = pass == 0;
        //  reverse z
        float depth = firstPass? 0.0f : glm::clamp(ToNDC(0.5f * gaussian.getWidth() * RenderSetting::g_sssLevel, proj), 0.0f, 1.0f);

        commandBuffer.bind_vertex_buffers(0, {*g_fullScreenTriangle}, {0});

        VkExtent2D extent{g_characterSSS->get_extent().width, g_characterSSS->get_extent().height};
        auto &render_frame = context.get_active_frame();

        struct{
            glm::vec2 stepSize;
            float correction;
            float maxdd;
            float depth;
        }sss;

        sss.stepSize = RenderSetting::g_sssLevel * gaussian.getWidth() * glm::vec2(1.0) / glm::vec2(extent.width, extent.height);
        sss.correction = RenderSetting::g_sssCorrection;
        sss.maxdd = RenderSetting::g_sssMaxDD;
        sss.depth = depth;

        auto allocation = render_frame.allocate_buffer(VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT, sizeof(sss));
        allocation.update(sss);

        ImageMemoryBarrier barrier{};
        barrier.src_stage_mask = VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT;
        barrier.dst_stage_mask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
        barrier.src_access_mask = VK_ACCESS_SHADER_READ_BIT;
        barrier.dst_access_mask = VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT;
        barrier.old_layout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
        barrier.new_layout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
        commandBuffer.image_memory_barrier(*g_transientBlurHView, barrier);

        barrier.src_stage_mask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
        barrier.dst_stage_mask = VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT;
        barrier.src_access_mask = VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT;
        barrier.dst_access_mask = VK_ACCESS_SHADER_READ_BIT;
        barrier.old_layout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
        barrier.new_layout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
        commandBuffer.image_memory_barrier(*g_transientBlurVView, barrier);

        VkAttachmentLoadOp loadOp = firstPass ? VK_ATTACHMENT_LOAD_OP_CLEAR : VK_ATTACHMENT_LOAD_OP_LOAD;
        std::vector<VkClearValue> clearValue{initializers::clear_color_value(0.0f, 0.0f, 0.0f, 0.0f)};

        // horizontal
        {
            std::vector<LoadStoreInfo> loadStoreInfos;
            loadStoreInfos.emplace_back(LoadStoreInfo{loadOp, VK_ATTACHMENT_STORE_OP_STORE});
            loadStoreInfos.emplace_back(LoadStoreInfo{VK_ATTACHMENT_LOAD_OP_LOAD, VK_ATTACHMENT_STORE_OP_STORE});

            std::vector<SubpassInfo> subPassInfos;
            subPassInfos.emplace_back(SubpassInfo{{}, {0}, {}, false, 0, VK_RESOLVE_MODE_NONE});

            std::vector<core::ImageView*> imageViews;
            imageViews.push_back(g_transientBlurHView.get());
            imageViews.push_back(g_characterDepthStencilView.get());

            std::vector<Attachment> attachments;
            attachments.emplace_back(Attachment{g_transientBlurH->get_format(),g_transientBlurH->get_sample_count(), g_transientBlurH->get_usage()});
            attachments.emplace_back(Attachment{g_characterDepthStencil->get_format(),g_characterDepthStencil->get_sample_count(), g_characterDepthStencil->get_usage()});

            auto &renderPass = context.get_device().get_resource_cache().request_render_pass(attachments, loadStoreInfos, subPassInfos);
            auto &frameBuffer = context.get_device().get_resource_cache().request_framebuffer(imageViews, renderPass);

            commandBuffer.begin_render_pass(extent, renderPass, frameBuffer, clearValue);
            RenderUtils::BindPipelineState(commandBuffer, blurPipeline);
            commandBuffer.bind_pipeline_layout(*blurLayout);
            commandBuffer.bind_buffer(allocation.get_buffer(), allocation.get_offset(), allocation.get_size(), 0, 0, 0);
            commandBuffer.bind_image(firstPass ? *g_characterSSSView : *g_transientBlurVView, *g_linearClampSampler, 0, 1, 0);
            commandBuffer.bind_image(*g_linearDepthView, *g_linearClampSampler, 0, 2, 0);
            commandBuffer.draw(3, 1, 0, 0);
            commandBuffer.end_render_pass();
        }

        barrier.src_stage_mask = VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT;
        barrier.dst_stage_mask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
        barrier.src_access_mask = VK_ACCESS_SHADER_READ_BIT;
        barrier.dst_access_mask = VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT;
        barrier.old_layout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
        barrier.new_layout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
        commandBuffer.image_memory_barrier(*g_transientBlurVView, barrier);
        if (firstPass) {
            commandBuffer.image_memory_barrier(*g_characterSSSView, barrier);
        }

        barrier.src_stage_mask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
        barrier.dst_stage_mask = VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT;
        barrier.src_access_mask = VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT;
        barrier.dst_access_mask = VK_ACCESS_SHADER_READ_BIT;
        barrier.old_layout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
        barrier.new_layout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
        commandBuffer.image_memory_barrier(*g_transientBlurHView, barrier);

        // vertical and accumulate 
        {
            std::vector<LoadStoreInfo> loadStoreInfos;
            loadStoreInfos.emplace_back(LoadStoreInfo{loadOp, VK_ATTACHMENT_STORE_OP_STORE});
            loadStoreInfos.emplace_back(LoadStoreInfo{VK_ATTACHMENT_LOAD_OP_LOAD, VK_ATTACHMENT_STORE_OP_STORE});
            loadStoreInfos.emplace_back(LoadStoreInfo{VK_ATTACHMENT_LOAD_OP_LOAD, VK_ATTACHMENT_STORE_OP_STORE});

            std::vector<SubpassInfo> subPassInfos;
            subPassInfos.emplace_back(SubpassInfo{{}, {0,1}, {}, false, 0, VK_RESOLVE_MODE_NONE});

            std::vector<core::ImageView*> imageViews;
            imageViews.push_back(g_transientBlurVView.get());
            imageViews.push_back(g_characterSSSView.get());
            imageViews.push_back(g_characterDepthStencilView.get());

            std::vector<Attachment> attachments;
            attachments.emplace_back(Attachment{g_transientBlurV->get_format(),g_transientBlurV->get_sample_count(), g_transientBlurV->get_usage()});
            attachments.emplace_back(Attachment{g_characterSSS->get_format(),g_characterSSS->get_sample_count(), g_characterSSS->get_usage()});
            attachments.emplace_back(Attachment{g_characterDepthStencil->get_format(),g_characterDepthStencil->get_sample_count(), g_characterDepthStencil->get_usage()});

            auto &renderPass = context.get_device().get_resource_cache().request_render_pass(attachments, loadStoreInfos, subPassInfos);
            auto &frameBuffer = context.get_device().get_resource_cache().request_framebuffer(imageViews, renderPass);

            commandBuffer.begin_render_pass(extent, renderPass, frameBuffer, clearValue);
            RenderUtils::BindPipelineState(commandBuffer, blurAccumPipeline);

            glm::vec4 weight = gaussian.getWeight();
            std::array<float, 4> blendConstants{weight.r, weight.g, weight.b, weight.a};
            commandBuffer.set_blend_constants(blendConstants);
            commandBuffer.bind_pipeline_layout(*blurAccumLayout);
            commandBuffer.bind_buffer(allocation.get_buffer(), allocation.get_offset(), allocation.get_size(), 0, 0, 0);
            commandBuffer.bind_image(*g_transientBlurHView, *g_linearClampSampler, 0, 1, 0);
            commandBuffer.bind_image(*g_linearDepthView, *g_linearClampSampler, 0, 2, 0);
            commandBuffer.draw(3, 1, 0, 0);
            commandBuffer.end_render_pass();
        }
    }

    void SetViewPortAndScissor(CommandBuffer& commandBuffer, const VkExtent2D& extent) {
        VkViewport viewport{};
        viewport.width = static_cast<float>(extent.width);
        viewport.height = static_cast<float>(extent.height);
        viewport.minDepth = 0.0f;
        viewport.maxDepth = 1.0f;
        commandBuffer.set_viewport(0, {viewport});

        VkRect2D scissor{};
        scissor.extent = extent;
        commandBuffer.set_scissor(0, {scissor});
    }

    void Draw(RenderContext& context, CommandBuffer& commandBuffer, sg::Camera* camera, RenderUtils::SortedMeshes *submeshs, sg::Scene* scene) {
        VkExtent2D extent{g_characterSSS->get_extent().width, g_characterSSS->get_extent().height};
        SetViewPortAndScissor(commandBuffer, extent);

        // draw diffuse only
        {
            std::vector<LoadStoreInfo> loadStoreInfos;
            loadStoreInfos.emplace_back(LoadStoreInfo{VK_ATTACHMENT_LOAD_OP_CLEAR, VK_ATTACHMENT_STORE_OP_STORE});
            loadStoreInfos.emplace_back(LoadStoreInfo{VK_ATTACHMENT_LOAD_OP_CLEAR, VK_ATTACHMENT_STORE_OP_STORE});
            loadStoreInfos.emplace_back(LoadStoreInfo{VK_ATTACHMENT_LOAD_OP_CLEAR, VK_ATTACHMENT_STORE_OP_STORE});

            std::vector<SubpassInfo> subPassInfos;
            subPassInfos.emplace_back(SubpassInfo{{}, {0,1}, {}, false, 0, VK_RESOLVE_MODE_NONE});

            std::vector<core::ImageView*> imageViews;
            imageViews.push_back(g_characterSSSView.get());
            imageViews.push_back(g_linearDepthView.get());
            imageViews.push_back(g_characterDepthStencilView.get());

            std::vector<Attachment> attachments;
            attachments.emplace_back(Attachment{g_characterSSS->get_format(),g_characterSSS->get_sample_count(), g_characterSSS->get_usage()});
            attachments.emplace_back(Attachment{g_linearDepth->get_format(),g_linearDepth->get_sample_count(), g_linearDepth->get_usage()});
            attachments.emplace_back(Attachment{g_characterDepthStencil->get_format(),g_characterDepthStencil->get_sample_count(), g_characterDepthStencil->get_usage()});

            auto &renderPass = context.get_device().get_resource_cache().request_render_pass(attachments, loadStoreInfos, subPassInfos);
            auto &frameBuffer = context.get_device().get_resource_cache().request_framebuffer(imageViews, renderPass);

            std::vector<VkClearValue> clearValue{initializers::clear_color_value(0,0,0,0), initializers::clear_color_value(0,0,0,0), initializers::clear_depth_stencil_value(0.0f, 0)};
            commandBuffer.begin_render_pass(extent, renderPass, frameBuffer, clearValue);
            DrawDiffuse(context, commandBuffer, camera, submeshs, scene);
            commandBuffer.end_render_pass();
        }

        ImageMemoryBarrier barrier{};
        barrier.src_stage_mask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
        barrier.dst_stage_mask = VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT;
        barrier.src_access_mask = VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT;
        barrier.dst_access_mask = VK_ACCESS_SHADER_READ_BIT;
        barrier.old_layout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
        barrier.new_layout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
        commandBuffer.image_memory_barrier(*g_characterSSSView, barrier);
        commandBuffer.image_memory_barrier(*g_linearDepthView, barrier);

        const std::vector<Gaussian>& gaussians = Gaussian::SKIN;
        //auto* perspective = (sg::PerspectiveCamera*)camera;
        //glm::mat4 proj = glm::perspective(perspective->get_field_of_view(), perspective->get_aspect_ratio(), perspective->get_near_plane(), perspective->get_far_plane());

        // sum of gaussian
        for (int i = 0; i < gaussians.size(); i++) {
            Blur(context, commandBuffer, i, Gaussian::SKIN.at(i), RenderUtils::VulkanStyleProjection(camera->get_projection()));
        }

        barrier.src_stage_mask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
        barrier.dst_stage_mask = VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT;
        barrier.src_access_mask = VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT;
        barrier.dst_access_mask = VK_ACCESS_SHADER_READ_BIT;
        barrier.old_layout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
        barrier.new_layout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
        commandBuffer.image_memory_barrier(*g_characterSSSView, barrier);
    }
}