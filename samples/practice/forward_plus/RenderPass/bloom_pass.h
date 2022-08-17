#pragma once

#include "rendering\render_context.h"

#include "compute_pass.h"

class bloom_pass : public compute_pass
{
  public:
	bloom_pass(vkb::Device *d, vkb::RenderContext *c);
	virtual ~bloom_pass() = default;

	inline void set_bloom_threshold(float value)
	{
		bloomThreshold = value;
	}

	virtual void prepare();

	virtual void dispatch(vkb::CommandBuffer &command_buffer);

  private:
    void BlurAndDownSample(vkb::CommandBuffer &command_buffer, int step);

  private:
	float               bloomThreshold = 1.0;
	vkb::RenderContext *renderContext  = nullptr;
	vkb::PipelineState  bloomExtractPipeline{};
	vkb::PipelineState  bloomBlurPipeline{};
};
