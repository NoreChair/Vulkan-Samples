#include "bloom_pass.h"

#include "GraphicContext.h"
#include "ShaderProgram.h"

using namespace GraphicContext;

bloom_pass::bloom_pass(vkb::Device *d, vkb::RenderContext *c) :
    compute_pass(d),
    renderContext(c)
{
}

void bloom_pass::prepare()
{
	std::function<void(std::string &, vkb::PipelineState &)> requestPipeline = [this](std::string &name, vkb::PipelineState &pipeline) {
		vkb::PipelineLayout &pipelineLayout = device->get_resource_cache().request_pipeline_layout(ShaderProgram::Find(name)->GetShaderModules());
		pipeline.set_pipeline_layout(pipelineLayout);
		device->get_resource_cache().request_compute_pipeline(pipeline);
	};

	requestPipeline(std::string("bloom_extract"), bloomExtractPipeline);
	requestPipeline(std::string("bloom_blur"), bloomBlurPipeline);
}

void bloom_pass::dispatch(vkb::CommandBuffer &commandBuffer)
{
	auto      extent = GraphicContext::hdrColorImage->get_extent();
	glm::vec4 ext    = glm::vec4(extent.width, extent.height, 1.0 / extent.width, 1.0 / extent.height);

	// extract brightness pixels and prepare to blur
	{
		vkb::ImageMemoryBarrier imageBarrier{};
		imageBarrier.src_stage_mask  = VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT;
		imageBarrier.dst_stage_mask  = VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT;
		imageBarrier.src_access_mask = VK_ACCESS_SHADER_READ_BIT;
		imageBarrier.dst_access_mask = VK_ACCESS_SHADER_WRITE_BIT;
		imageBarrier.old_layout      = VK_IMAGE_LAYOUT_UNDEFINED;        // we can discard old's content
		imageBarrier.new_layout      = VK_IMAGE_LAYOUT_GENERAL;
		commandBuffer.image_memory_barrier(*bloomChainImageView[0], imageBarrier);

		struct
		{
			glm::vec4 extent;
			float     bloomThreshold;
		} uniforms{
		    ext,
		    bloomThreshold
        };

		bind_pipeline(commandBuffer, bloomExtractPipeline);
		auto allocation = renderContext->get_active_frame().allocate_buffer(VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT, sizeof(uniforms));
		allocation.update(uniforms);

		commandBuffer.bind_buffer(allocation.get_buffer(), allocation.get_offset(), allocation.get_size(), 0, 0, 0);
		commandBuffer.bind_buffer(*exposureBuffer, 0, exposureBuffer->get_size(), 0, 1, 0);
		commandBuffer.bind_image(*hdrColorImageView, *linearClampSampler, 0, 2, 0);
		commandBuffer.bind_image(*bloomChainImageView[0], 0, 3, 0);
		commandBuffer.dispatch((extent.width + 16) / 16, (extent.height + 16) / 16, 1);
	}

	for (int i = 1; i < 4; i++)
	{
		BlurAndDownSample(commandBuffer, i);
	}

    // prepare to sample
    {
        vkb::ImageMemoryBarrier imageBarrier{};
	    imageBarrier.src_stage_mask  = VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT;
	    imageBarrier.dst_stage_mask  = VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT;
	    imageBarrier.src_access_mask = VK_ACCESS_SHADER_WRITE_BIT;
	    imageBarrier.dst_access_mask = VK_ACCESS_SHADER_READ_BIT;
	    imageBarrier.old_layout      = VK_IMAGE_LAYOUT_GENERAL;
	    imageBarrier.new_layout      = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
	    commandBuffer.image_memory_barrier(*bloomChainImageView[3], imageBarrier);
    }
}

void bloom_pass::BlurAndDownSample(vkb::CommandBuffer &commandBuffer, int step)
{
	vkb::ImageMemoryBarrier imageBarrier{};
	imageBarrier.src_stage_mask  = VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT;
	imageBarrier.dst_stage_mask  = VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT;
	imageBarrier.src_access_mask = VK_ACCESS_SHADER_WRITE_BIT;
	imageBarrier.dst_access_mask = VK_ACCESS_SHADER_READ_BIT;
	imageBarrier.old_layout      = VK_IMAGE_LAYOUT_GENERAL;
	imageBarrier.new_layout      = VK_IMAGE_LAYOUT_GENERAL;
	commandBuffer.image_memory_barrier(*bloomChainImageView[step - 1], imageBarrier);

    imageBarrier.src_stage_mask  = VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT;
	imageBarrier.dst_stage_mask  = VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT;
	imageBarrier.src_access_mask = 0;
	imageBarrier.dst_access_mask = VK_ACCESS_SHADER_WRITE_BIT;
	imageBarrier.old_layout      = VK_IMAGE_LAYOUT_UNDEFINED;
	imageBarrier.new_layout      = VK_IMAGE_LAYOUT_GENERAL;
	commandBuffer.image_memory_barrier(*bloomChainImageView[step], imageBarrier);

    auto sourceExt = bloomChainImage[step - 1]->get_extent();
	auto targetExt = bloomChainImage[step]->get_extent();

	struct
	{
		glm::vec4 source;
        glm::vec4 target;
	} uniforms{
        glm::vec4(sourceExt.width, sourceExt.height, 1.0 / sourceExt.width, 1.0 / sourceExt.height),
	    glm::vec4(targetExt.width, targetExt.height, 1.0 / targetExt.width, 1.0 / targetExt.height)
    };

	bind_pipeline(commandBuffer, bloomBlurPipeline);
	auto allocation = renderContext->get_active_frame().allocate_buffer(VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT, sizeof(uniforms));
	allocation.update(uniforms);

	commandBuffer.bind_buffer(allocation.get_buffer(), allocation.get_offset(), allocation.get_size(), 0, 0, 0);
	commandBuffer.bind_image(*bloomChainImageView[step - 1], *linearClampSampler, 0, 1, 0);
	commandBuffer.bind_image(*bloomChainImageView[step], 0, 2, 0);
	commandBuffer.dispatch((sourceExt.width + 8) / 8, (sourceExt.height + 8) / 8, 1);
}
