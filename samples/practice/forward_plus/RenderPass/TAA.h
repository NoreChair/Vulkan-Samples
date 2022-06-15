#pragma once

#include "ShaderProgram.h"
#include "common/vk_common.h"
#include "platform/platform.h"
#include "scene_graph/components/camera.h"

// [Temporal Antialiasing In Uncharted 4]
// [An Excursion In Temporal Supersampling]
class TAA
{
  public:
	TAA(vkb::Device *d, vkb::RenderContext *c);
	~TAA();

	void      prepare();
	glm::vec2 get_jitter(uint32_t frameIndex);
	void      clear_history();
	void      gen_velocity_buffer(vkb::CommandBuffer &commandBuffer, vkb::sg::Camera* camera, int srcIndex);
	void      reslove(vkb::CommandBuffer &commandBuffer, int srcIndex, int targetIndex, float sharpen);
	void      set_blend_count(int v){temporalBlendCount = v;}
    void      set_previous_view_proj(glm::mat4 view, glm::mat4 projection) {
        prevView = view;
        prevProj = projection;
    }
  private:
    void bind_pipeline(vkb::CommandBuffer &commandBuffer, vkb::PipelineState& pipelineState);

  private:
	const static uint32_t k_jittlerSampleCount = 8;
	static bool           s_samplePointInited;
	static glm::vec2      s_hammersleyPoint[k_jittlerSampleCount];

	bool                clearHistoryFlag{false};
	int                 temporalBlendCount{4};
    glm::mat4           prevView;
    glm::mat4           prevProj;
    vkb::Device        *device{nullptr};
	vkb::RenderContext *context{nullptr};
    vkb::PipelineState velocityGenPipeline;
    vkb::PipelineState temporalReslovePipeline;
    vkb::PipelineState sharpenPipeline;
};
