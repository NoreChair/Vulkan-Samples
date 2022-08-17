#pragma once

#include "core/command_buffer.h"
#include "core/device.h"
#include "core/shader_module.h"
#include "rendering/pipeline_state.h"

class compute_pass
{
  public:
	compute_pass(vkb::Device *d);

	virtual ~compute_pass() = default;

	compute_pass(const compute_pass &) = delete;

	compute_pass(compute_pass &&) = default;

	compute_pass &operator=(const compute_pass &) = delete;

	compute_pass &operator=(compute_pass &&) = delete;

	virtual void prepare(std::vector<vkb::ShaderModule *> &shader_module);

	virtual void dispatch(vkb::CommandBuffer &command_buffer) = 0;

  protected:
	void bind_pipeline(vkb::CommandBuffer &command_buffer);
    void bind_pipeline(vkb::CommandBuffer &commandBuffer, vkb::PipelineState& pipelineState);

	vkb::Device *      device{nullptr};
	vkb::PipelineState pipeline_state{};
};