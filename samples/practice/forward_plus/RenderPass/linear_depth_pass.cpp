#include "linear_depth_pass.h"
#include "buffer_pool.h"
#include "GraphicContext.h"
#include "ShaderProgram.h"

using namespace GraphicContext;

linear_depth_pass::linear_depth_pass(vkb::Device *d, vkb::RenderContext *ctx) :
    compute_pass(d),
    context{ctx}
{
    std::function<void(std::string&, vkb::PipelineState&)> requestPipeline = [this](std::string& name, vkb::PipelineState& pipeline) {
        vkb::PipelineLayout &pipelineLayout = device->get_resource_cache().request_pipeline_layout(ShaderProgram::Find(name)->GetShaderModules());
        pipeline.set_pipeline_layout(pipelineLayout);
        device->get_resource_cache().request_compute_pipeline(pipeline);
    };
    requestPipeline(std::string("depth_down_sample"), depthChainPipeline);
}

void linear_depth_pass::dispatch(vkb::CommandBuffer & command_buffer, vkb::sg::Camera * camera, int srcIndex) {
    render_camera = camera;
    scene_depth = sceneDepthImageView.get();
    linear_depth = linearDepthImageView[srcIndex].get();
    srcID = srcIndex;
    dispatch(command_buffer);
}

void linear_depth_pass::dispatch(vkb::CommandBuffer &command_buffer)
{
	VkExtent3D extent = scene_depth->get_image().get_extent();

	bind_pipeline(command_buffer);
	command_buffer.bind_input(*scene_depth, 0, 0, 0);
	command_buffer.bind_input(*linear_depth, 0, 1, 0);
	struct
	{
		float    nearPlane;
		float    farPlane;
		uint32_t width;
		uint32_t height;
	} uniforms{render_camera->get_near_plane(), render_camera->get_far_plane(), extent.width, extent.height};

	vkb::BufferAllocation allocation = context->get_active_frame().allocate_buffer(VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT, sizeof(uniforms));
	allocation.update(uniforms);
	command_buffer.bind_buffer(allocation.get_buffer(), allocation.get_offset(), allocation.get_offset(), 0, 2, 0);
	command_buffer.dispatch((uint32_t)glm::ceil(extent.width / 16.0f), (uint32_t)glm::ceil(extent.height / 16.0f), 1);

    VkImageSubresourceRange range{};
    range.aspectMask = VK_IMAGE_ASPECT_DEPTH_BIT;
    range.baseMipLevel = 0;
    range.levelCount = 1;
    range.baseArrayLayer = 0;
    range.layerCount = 1;

    vkb::ImageMemoryBarrier barrier{};
	barrier.src_stage_mask  = VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT;
	barrier.dst_stage_mask  = VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT;
	barrier.src_access_mask = VK_ACCESS_SHADER_WRITE_BIT;
	barrier.dst_access_mask = VK_ACCESS_SHADER_READ_BIT;
    barrier.old_layout      = VK_IMAGE_LAYOUT_GENERAL;
	barrier.new_layout      = VK_IMAGE_LAYOUT_GENERAL;
	command_buffer.image_memory_barrier(*linear_depth, barrier);

    // Depth Chain
	command_buffer.bind_pipeline_layout(const_cast<vkb::PipelineLayout &>(depthChainPipeline.get_pipeline_layout()));

    uint32_t mipCount = linear_depth->get_image().get_mip_count();
    command_buffer.bind_image(*linear_depth, *linearClampSampler, 0, 0, 0);

    for (int i = 1; i < mipCount; i++)
    {
        int w = glm::max(extent.width >> i, 1u);
        int h = glm::max(extent.height >> i, 1u);
        command_buffer.push_constants(i);
        command_buffer.bind_image(*depthChainImageViews[srcID][i - 1], 0, 1, 0);
        command_buffer.dispatch((w + 15) / 16,(h + 15) / 16, 1);

        range.baseMipLevel = i;
        command_buffer.image_memory_barrier(*linear_depth, barrier, range);
    }

}
