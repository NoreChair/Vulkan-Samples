#include "SSR.h"
#include "ShaderProgram.h"
#include "GraphicContext.h"
#include "scene_graph\node.h"
#include "scene_graph\components\perspective_camera.h"
#include "TAA.h"

using namespace GraphicContext;

glm::mat4 VulkanProjection(const glm::mat4 &proj)
{
	// Flip Y in clipspace. X = -1, Y = -1 is topLeft in Vulkan.
	glm::mat4 mat = proj;
	mat[1][1] *= -1;

	return mat;
}


SSR::SSR(vkb::Device * d, vkb::RenderContext * c) : device(d), context(c)
{
}

void SSR::prepare()
{
    std::function<void(std::string&, vkb::PipelineState&)> requestPipeline = [this](std::string& name, vkb::PipelineState& pipeline) {
        vkb::PipelineLayout &pipelineLayout = device->get_resource_cache().request_pipeline_layout(ShaderProgram::Find(name)->GetShaderModules());
        pipeline.set_pipeline_layout(pipelineLayout);
        device->get_resource_cache().request_compute_pipeline(pipeline);
    };
    requestPipeline(std::string("ray_allocate"), rayAllocationPipeline);
    requestPipeline(std::string("prepare_indirect"), prepareIndirectPipeline);
    requestPipeline(std::string("ray_marching"), rayMarchingPipeline);
}

void SSR::dispatch(vkb::CommandBuffer &commandBuffer, vkb::sg::Camera* camera, int srcIndex, int frameIndex)
{
    auto extent = GraphicContext::rayIndexImage->get_extent();
    auto& cameraTransform = camera->get_node()->get_transform();
    auto& cameraPos = cameraTransform.get_translation();
    auto& cameraRot = glm::mat3_cast(cameraTransform.get_rotation());
    float aspect = ((vkb::sg::PerspectiveCamera*)camera)->get_aspect_ratio();
    float nearPlane = camera->get_near_plane();
    float farPlane = camera->get_far_plane();
    float yLength = farPlane / camera->get_projection_without_jitter()[1][1];
    float xLength = yLength * aspect;

    glm::vec3 right = cameraRot[0];
    glm::vec3 up = cameraRot[1];
    glm::vec3 forward = cameraRot[2];

    struct{
        uint32_t rayCount{0};
    }counter;
    auto rayCounter = context->get_active_frame().allocate_buffer(VK_BUFFER_USAGE_STORAGE_BUFFER_BIT, 4);
    rayCounter.update(counter);

    int groupX = (extent.width + 15)/16;
    int groupY = (extent.height + 15)/16;

    uint32_t jitterCount = 0;
    const glm::vec2* points = TAA::GetHammersleyPoints(&jitterCount);
    glm::vec4 screenParam = glm::vec4(extent.width, extent.height, 1.0f/extent.width, 1.0f/extent.height);

    // ray allocate
    {
        struct{
	        glm::vec4 extent;
	        glm::vec4 cameraPosition;
	        glm::vec4 cameraProjectionCorner;
	        glm::vec4 cameraProjectionHorizon;
	        glm::vec4 cameraProjectionVertical;
	        glm::vec2 cameraClip;               // near, far
            glm::vec2 jitter;
	        float     ssrFadingDistance;        // must be less than far plane
            float     depthLevel;
        }uniforms{
            screenParam,
            glm::vec4(cameraPos, 1.0f),
            glm::vec4(forward * farPlane - right * xLength - up * yLength, 1.0f),
            glm::vec4(right * xLength * 2.0f, 1.0f),
            glm::vec4(up * yLength * 2.0f, 1.0f),
            glm::vec2(nearPlane, farPlane),
            points[frameIndex % jitterCount],
            100.0f,
            2.0f
        };

        bind_pipeline(commandBuffer, rayAllocationPipeline);
        auto allocation = context->get_active_frame().allocate_buffer(VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT, sizeof(uniforms));
        allocation.update(uniforms);

        commandBuffer.bind_buffer(allocation.get_buffer(), allocation.get_offset(), allocation.get_size(), 0, 0, 0);
        commandBuffer.bind_buffer(rayCounter.get_buffer(), rayCounter.get_offset(), rayCounter.get_size(), 0, 1, 0);
        commandBuffer.bind_buffer(*raysBuffer, 0, raysBuffer->get_size(), 0, 2, 0);
        commandBuffer.bind_image(*linearDepthImageView[srcIndex], *pointClampSampler, 0, 3, 0);
        commandBuffer.bind_image(*sceneNormalImageView, *linearClampSampler, 0, 4, 0);
        commandBuffer.bind_image(*sceneParamsImageView, *linearClampSampler, 0, 5, 0);
        commandBuffer.bind_image(*rayIndexImageView, 0, 6, 0);
        commandBuffer.dispatch(groupX, groupY, 1);

        vkb::BufferMemoryBarrier barrier{};
        barrier.src_stage_mask = VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT;
        barrier.src_access_mask = VK_ACCESS_SHADER_WRITE_BIT;
        barrier.dst_stage_mask = VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT;
        barrier.dst_access_mask = VK_ACCESS_SHADER_READ_BIT;
        commandBuffer.buffer_memory_barrier(rayCounter.get_buffer(), rayCounter.get_offset(), rayCounter.get_size(), barrier);
        commandBuffer.buffer_memory_barrier(*raysBuffer, 0, raysBuffer->get_size(), barrier);
    }

    // indirect draw
    {
        bind_pipeline(commandBuffer, prepareIndirectPipeline);
        commandBuffer.bind_buffer(rayCounter.get_buffer(), rayCounter.get_offset(), rayCounter.get_size(), 0, 0, 0);
        commandBuffer.bind_buffer(*indirectBuffer, 0, indirectBuffer->get_size(), 0, 1, 0);
        commandBuffer.dispatch(1, 1, 1);

        vkb::BufferMemoryBarrier barrier{};
        barrier.src_stage_mask = VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT;
        barrier.src_access_mask = VK_ACCESS_SHADER_WRITE_BIT;
        barrier.dst_stage_mask = VK_PIPELINE_STAGE_DRAW_INDIRECT_BIT;
        barrier.dst_access_mask = VK_ACCESS_INDIRECT_COMMAND_READ_BIT;
        commandBuffer.buffer_memory_barrier(*indirectBuffer, 0, indirectBuffer->get_size(), barrier);
    }

    {
        struct{
            glm::mat4  viewMat;
	        glm::mat4  projMat;
	        glm::vec4  extent;
	        glm::vec2  cameraClip;
	        float maxReflectanceDistance;
	        float pixelStride;
	        float jitter;
            float depthLevel;
        }uniforms{
            camera->get_view(),
            VulkanProjection(camera->get_projection()),
            screenParam,
            glm::vec2(nearPlane, farPlane),
            100.0f,
            1.0f,
            ((double) rand() / (RAND_MAX)),
            2.0f
        };

        bind_pipeline(commandBuffer, rayMarchingPipeline);
        auto allocation = context->get_active_frame().allocate_buffer(VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT, sizeof(uniforms));
        allocation.update(uniforms);
        commandBuffer.bind_buffer(allocation.get_buffer(), allocation.get_offset(), allocation.get_size(), 0, 0, 0);
        commandBuffer.bind_buffer(rayCounter.get_buffer(), rayCounter.get_offset(), rayCounter.get_size(), 0, 1, 0);
        commandBuffer.bind_buffer(*raysBuffer, 0, raysBuffer->get_size(), 0, 2, 0);
        commandBuffer.bind_image(*hdrColorImageView, *linearClampSampler, 0, 3, 0);
        commandBuffer.bind_image(*linearDepthImageView[srcIndex], *linearClampSampler, 0, 4, 0);
        commandBuffer.bind_image(*sceneNormalImageView, *linearClampSampler, 0, 5, 0);
        commandBuffer.bind_image(*reflectImageView, 0, 6, 0);
        commandBuffer.bind_image(*rayIndexImageView, 0, 7, 0);
        commandBuffer.dispatch_indirect(*indirectBuffer, 0);
    }

}

void SSR::bind_pipeline(vkb::CommandBuffer & commandBuffer, vkb::PipelineState & pipelineState)
{
    commandBuffer.set_color_blend_state(pipelineState.get_color_blend_state());
    commandBuffer.set_depth_stencil_state(pipelineState.get_depth_stencil_state());
    commandBuffer.set_input_assembly_state(pipelineState.get_input_assembly_state());
    commandBuffer.set_rasterization_state(pipelineState.get_rasterization_state());
    commandBuffer.set_viewport_state(pipelineState.get_viewport_state());
    commandBuffer.set_multisample_state(pipelineState.get_multisample_state());
    commandBuffer.bind_pipeline_layout(const_cast<vkb::PipelineLayout &>(pipelineState.get_pipeline_layout()));
}
