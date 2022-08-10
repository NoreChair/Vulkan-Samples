#include "TAA.h"
#include "HammersleyHaltonPoints.h"
#include "ShaderProgram.h"
#include "GraphicContext.h"

bool TAA::s_samplePointInited = false;
glm::vec2 TAA::s_hammersleyPoint[k_jittlerSampleCount] = {};

TAA::TAA(vkb::Device* d, vkb::RenderContext* c) : device(d), context(c) {
    if (!s_samplePointInited) {
        s_samplePointInited = true;
        Hammersley((float*)s_hammersleyPoint, k_jittlerSampleCount);
    }
}

TAA::~TAA() {}

void TAA::prepare() {
    std::function<void(std::string&, vkb::PipelineState&)> requestPipeline = [this](std::string& name, vkb::PipelineState& pipeline) {
        vkb::PipelineLayout &pipelineLayout = device->get_resource_cache().request_pipeline_layout(ShaderProgram::Find(name)->GetShaderModules());
        pipeline.set_pipeline_layout(pipelineLayout);
        device->get_resource_cache().request_compute_pipeline(pipeline);
    };

    requestPipeline(std::string("velocity_gen"), velocityGenPipeline);
    requestPipeline(std::string("temporal_resolve"), temporalReslovePipeline);
    requestPipeline(std::string("sharpen"), sharpenPipeline);
}

glm::vec2 TAA::get_jitter(uint32_t frameIndex) {
    uint32_t jitterSampleIndex = frameIndex % k_jittlerSampleCount;
    return s_hammersleyPoint[jitterSampleIndex];
}

void TAA::clear_history(bool clear) {
    clearHistoryFlag = clear;
}

void TAA::gen_velocity_buffer(vkb::CommandBuffer &commandBuffer, vkb::sg::Camera* camera, int srcIndex){
    // remove jitter to generate velocity buffer
    // we don't want jitter to be counted as motion
    auto prevProjWithoutJitter = prevProj;
    prevProjWithoutJitter[2][0] = 0.0f;
    prevProjWithoutJitter[2][1] = 0.0f;

    auto extent = GraphicContext::velocityImage->get_extent();

    struct{
        glm::mat4 invCurViewProj;
        glm::mat4 prevViewPorj;
        glm::vec4 extent;
        float n;
        float f;
    }uniforms{
        glm::inverse(prevProjWithoutJitter * prevView),
        camera->get_projection_without_jitter() * camera->get_view(),
        glm::vec4(extent.width, extent.height, 1.0 / extent.width, 1.0 / extent.height),
        camera->get_near_plane(),
        camera->get_far_plane()
    };

    bind_pipeline(commandBuffer, velocityGenPipeline);
    auto allocation = context->get_active_frame().allocate_buffer(VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT, sizeof(uniforms));
    allocation.update(uniforms);

    commandBuffer.bind_buffer(allocation.get_buffer(), allocation.get_offset(), allocation.get_size(), 0, 0, 0);
    commandBuffer.bind_image(*GraphicContext::linearDepthImageView[srcIndex], 0, 1, 0);
    commandBuffer.bind_image(*GraphicContext::velocityImageView, 0, 2, 0);
    commandBuffer.dispatch((extent.width + 16) / 16, (extent.height + 16) / 16, 1);

    vkb::ImageMemoryBarrier barrier{};
    barrier.src_stage_mask = VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT;
    barrier.dst_stage_mask = VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT;
    barrier.src_access_mask = VK_ACCESS_SHADER_WRITE_BIT;
    barrier.dst_access_mask = VK_ACCESS_SHADER_READ_BIT;
    barrier.old_layout = VK_IMAGE_LAYOUT_GENERAL;
    barrier.new_layout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
    commandBuffer.image_memory_barrier(*GraphicContext::velocityImageView, barrier);
}

void TAA::reslove(vkb::CommandBuffer &commandBuffer, int srcIndex, int targetIndex, float sharpen) {

    auto extent = GraphicContext::velocityImage->get_extent();
    glm::vec4 ext = glm::vec4(extent.width, extent.height, 1.0 / extent.width, 1.0 / extent.height);

    {
        struct{
            glm::vec4 extent;
            glm::vec2 viewportJitter;
            float speedLimiter; // e.g. 4 pixel as threshould so the speed limiter is 1/4 = 0.25, better use 1/128 on 1080p
            float discardHistory; // 0.0 mean history buffer are expired mostly happen on beginning, otherwise use 1.0
        }uniforms{
            ext,
            glm::vec2(prevProj[2][0], prevProj[2][1]),
            1.0f / 128.0f,
            clearHistoryFlag ? 0.0 : 1.0
        };

        bind_pipeline(commandBuffer, temporalReslovePipeline);
        auto allocation = context->get_active_frame().allocate_buffer(VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT, sizeof(uniforms));
        allocation.update(uniforms);

        commandBuffer.bind_buffer(allocation.get_buffer(), allocation.get_offset(), allocation.get_size(), 0, 0, 0);
        commandBuffer.bind_image(*GraphicContext::linearDepthImageView[srcIndex], *GraphicContext::pointClampSampler, 0, 1, 0);
        commandBuffer.bind_image(*GraphicContext::linearDepthImageView[targetIndex], *GraphicContext::pointClampSampler, 0, 2, 0);
        commandBuffer.bind_image(*GraphicContext::hdrColorImageView, *GraphicContext::pointClampSampler, 0, 3, 0);
        commandBuffer.bind_image(*GraphicContext::temporalBlendImageView[srcIndex], *GraphicContext::linearClampSampler, 0, 4, 0);
        commandBuffer.bind_image(*GraphicContext::temporalBlendImageView[targetIndex], 0, 5, 0);
        commandBuffer.bind_image(*GraphicContext::velocityImageView, *GraphicContext::pointClampSampler, 0, 6, 0);
        commandBuffer.dispatch((extent.width + 8) / 8, (extent.height + 8) / 8, 1);
    }

    // prepare to dispatch sharpen image
    vkb::ImageMemoryBarrier barrier{};
    barrier.src_stage_mask = VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT;
    barrier.dst_stage_mask = VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT;
    barrier.src_access_mask = VK_ACCESS_SHADER_WRITE_BIT;
    barrier.dst_access_mask = VK_ACCESS_SHADER_READ_BIT;
    barrier.old_layout = VK_IMAGE_LAYOUT_GENERAL;
    barrier.new_layout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
    commandBuffer.image_memory_barrier(*GraphicContext::temporalBlendImageView[targetIndex], barrier);

    barrier.src_access_mask = VK_ACCESS_SHADER_READ_BIT;
    barrier.dst_access_mask = VK_ACCESS_SHADER_WRITE_BIT;
    barrier.old_layout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
    barrier.new_layout = VK_IMAGE_LAYOUT_GENERAL;
    commandBuffer.image_memory_barrier(*GraphicContext::hdrColorImageView, barrier);

    {
        // sharpen
        struct
        {
            glm::vec4 extent;
            glm::vec2 weight;
        }uniforms{
            ext,
            glm::vec2(1.0f + sharpen, sharpen * 0.25f)
        };

        bind_pipeline(commandBuffer, sharpenPipeline);
        auto allocation = context->get_active_frame().allocate_buffer(VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT, sizeof(uniforms));
        allocation.update(uniforms);

        commandBuffer.bind_buffer(allocation.get_buffer(), allocation.get_offset(), allocation.get_size(), 0, 0, 0);
        commandBuffer.bind_image(*GraphicContext::temporalBlendImageView[targetIndex], *GraphicContext::linearClampSampler, 0, 1, 0);
        commandBuffer.bind_image(*GraphicContext::hdrColorImageView, 0, 2, 0);
        commandBuffer.dispatch((extent.width + 8) / 8, (extent.height + 8) / 8, 1);
    }

}

void TAA::bind_pipeline(vkb::CommandBuffer &commandBuffer, vkb::PipelineState& pipelineState){
    commandBuffer.set_color_blend_state(pipelineState.get_color_blend_state());
    commandBuffer.set_depth_stencil_state(pipelineState.get_depth_stencil_state());
    commandBuffer.set_input_assembly_state(pipelineState.get_input_assembly_state());
    commandBuffer.set_rasterization_state(pipelineState.get_rasterization_state());
    commandBuffer.set_viewport_state(pipelineState.get_viewport_state());
    commandBuffer.set_multisample_state(pipelineState.get_multisample_state());
    commandBuffer.bind_pipeline_layout(const_cast<vkb::PipelineLayout &>(pipelineState.get_pipeline_layout()));
}
