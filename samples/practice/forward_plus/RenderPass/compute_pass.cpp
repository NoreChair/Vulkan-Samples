#include "compute_pass.h"
#include "core/pipeline_layout.h"
#include "resource_cache.h"

compute_pass::compute_pass(vkb::Device *d) :
    device(d)
{
}

void compute_pass::prepare(std::vector<vkb::ShaderModule *> &shader_module)
{
	vkb::PipelineLayout &pipelineLayout = device->get_resource_cache().request_pipeline_layout(shader_module);

	pipeline_state.set_pipeline_layout(pipelineLayout);
	device->get_resource_cache().request_compute_pipeline(pipeline_state);
}

void compute_pass::bind_pipeline(vkb::CommandBuffer &command_buffer)
{
	command_buffer.set_color_blend_state(pipeline_state.get_color_blend_state());
	command_buffer.set_depth_stencil_state(pipeline_state.get_depth_stencil_state());
	command_buffer.set_input_assembly_state(pipeline_state.get_input_assembly_state());
	command_buffer.set_rasterization_state(pipeline_state.get_rasterization_state());
	command_buffer.set_viewport_state(pipeline_state.get_viewport_state());
	command_buffer.set_multisample_state(pipeline_state.get_multisample_state());
	command_buffer.bind_pipeline_layout(const_cast<vkb::PipelineLayout &>(pipeline_state.get_pipeline_layout()));
}

void compute_pass::bind_pipeline(vkb::CommandBuffer &commandBuffer, vkb::PipelineState& pipelineState){
    commandBuffer.set_color_blend_state(pipelineState.get_color_blend_state());
    commandBuffer.set_depth_stencil_state(pipelineState.get_depth_stencil_state());
    commandBuffer.set_input_assembly_state(pipelineState.get_input_assembly_state());
    commandBuffer.set_rasterization_state(pipelineState.get_rasterization_state());
    commandBuffer.set_viewport_state(pipelineState.get_viewport_state());
    commandBuffer.set_multisample_state(pipelineState.get_multisample_state());
    commandBuffer.bind_pipeline_layout(const_cast<vkb::PipelineLayout &>(pipelineState.get_pipeline_layout()));
}