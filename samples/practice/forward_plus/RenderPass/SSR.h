#pragma once

#include "common/vk_common.h"
#include "platform/platform.h"
#include "scene_graph/components/camera.h"

class SSR
{
  public:
	SSR(vkb::Device *d, vkb::RenderContext *c);
	~SSR() = default;

	void prepare();
    void dispatch(vkb::CommandBuffer &commandBuffer, vkb::sg::Camera* camera, int srcIndex, int frameIndex);

  private:
	void bind_pipeline(vkb::CommandBuffer &commandBuffer, vkb::PipelineState &pipelineState);

  private:
    vkb::Device        *device{nullptr};
	vkb::RenderContext *context{nullptr};
    vkb::PipelineState rayAllocationPipeline;
    vkb::PipelineState prepareIndirectPipeline;
    vkb::PipelineState rayMarchingPipeline;
};
