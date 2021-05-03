/*
 * Forward plus rendering
 */
#pragma once

#include "common/vk_common.h"
#include "core/shader_module.h"
#include "platform/platform.h"
#include "scene_graph/components/camera.h"
#include "vulkan_sample.h"
#include <initializer_list>
#include <unordered_map>

#if defined(DEBUG) || defined(_DEBUG)
#	define DEBUG_ASSERT(exp, ...) \
		{                          \
			if (!(exp))            \
				LOGD(__VA_ARGS__)  \
			assert(exp);           \
		}
#else
#	define DEBUG_ASSERT(exp, ...) 0
#endif

#define MAX_LIGHTS_COUNT 128

class vkb::sg::Node;
class vkb::sg::SubMesh;

class forward_plus : public vkb::VulkanSample
{
	enum RenderOrder : uint32_t
	{
		Opaque,
		AlphaBlend,
		PostProcess
	};

	struct alignas(16) GlobalUniform
	{
		glm::mat4 model;
		glm::mat4 view_project;
		glm::vec3 camera_position;
	};

	struct LightBuffer
	{
		glm::vec3 position;          // xyz=pos
		glm::vec3 color;             // xyz=rgb
		glm::vec3 coneDir;           // xyz=direction if light source is spot
		glm::vec3 coneAngles;        // x=1.0f/(cos(inner)-cos(outer)), y=cos(inner), z=cos(outer/2)
		float     radius;
		float     intensity;
		uint32_t  lightType;
		uint32_t  padding;
	};

	struct RenderPassEntry
	{
		std::shared_ptr<vkb::RenderTarget>     renderTarget;
		vkb::Framebuffer *                     frameBuffer;
		vkb::RenderPass *                      renderPass;
		std::map<uint32_t, vkb::PipelineState> pipelines;

		inline vkb::RenderTarget &GetRenderTarget()
		{
			return *renderTarget.get();
		}

		inline vkb::Framebuffer &GetFrameBuffer()
		{
			return *frameBuffer;
		}

		inline vkb::RenderPass &GetRenderPass()
		{
			return *renderPass;
		}

		void PrebuildPass(vkb::Device &device, std::vector<vkb::LoadStoreInfo> &loadStoreInfos, std::vector<vkb::SubpassInfo> &subPassInfos)
		{
			DEBUG_ASSERT(renderTarget != nullptr, "RenderTarget can not be nullptr");
			renderPass = &device.get_resource_cache().request_render_pass(renderTarget->get_attachments(), loadStoreInfos, subPassInfos);
			DEBUG_ASSERT(renderPass != nullptr, "RenderPass can not be nullptr");
			frameBuffer = &device.get_resource_cache().request_framebuffer(GetRenderTarget(), GetRenderPass());
			DEBUG_ASSERT(frameBuffer != nullptr, "FrameBuffer can not be nullptr");
		}

		vkb::PipelineState *PushPipeline(vkb::Device &device, uint32_t order, std::string &programName)
		{
			ShaderProgram *      program = ShaderProgram::Find(programName);
			vkb::PipelineLayout &layout  = device.get_resource_cache().request_pipeline_layout(program->GetShaderModules());

			vkb::PipelineState pipelineState;
			pipelineState.set_pipeline_layout(layout);
			pipelineState.set_render_pass(GetRenderPass());
			pipelines.emplace(std::make_pair(order, std::move(pipelineState)));
			return &pipelines[order];
		}
	};

	struct ComputePassEntry
	{
		vkb::PipelineLayout *pipelineLayout{nullptr};
	};

	struct ShaderProgram
	{
		ShaderProgram(vkb::ShaderModule *shader)
		{
			shaderModules.push_back(shader);
			programStageFlag = shader->get_stage();
		}

		ShaderProgram(std::initializer_list<vkb::ShaderModule *> &&params)
		{
			programStageFlag = (VkShaderStageFlagBits) 0;
			for (auto ptr = params.begin(); ptr != params.end(); ++ptr)
			{
				shaderModules.push_back(*ptr);
				programStageFlag = (VkShaderStageFlagBits)((int) programStageFlag | (int) (*ptr)->get_stage());
			}
		}

		std::vector<vkb::ShaderModule *> &GetShaderModules()
		{
			return shaderModules;
		}

		bool IsGraphicProgram()
		{
			return (programStageFlag & VK_SHADER_STAGE_ALL_GRAPHICS) != 0;
		}

		bool IsComputeProgram()
		{
			return (programStageFlag & VK_SHADER_STAGE_COMPUTE_BIT) != 0;
		}

		static size_t AddShaderProgram(std::string &name, std::shared_ptr<ShaderProgram> &&program)
		{
			size_t uid = std::hash<std::string>{}(name);
			AddShaderProgram(uid, std::move(program));
			return uid;
		}

		static void AddShaderProgram(size_t uid, std::shared_ptr<ShaderProgram> &&program)
		{
			DEBUG_ASSERT(shaderProgramPool.find(uid) == shaderProgramPool.end(), "Shader program uid collision");
			shaderProgramPool.emplace(uid, std::move(program));
		}

		static void RemoveShaderProgram(size_t uid)
		{
			auto iter = shaderProgramPool.find(uid);
			if (iter != shaderProgramPool.end())
			{
				shaderProgramPool.erase(iter);
			}
		}

		static ShaderProgram *Find(std::string &name)
		{
			size_t uid  = std::hash<std::string>{}(name);
			auto   iter = shaderProgramPool.find(uid);
			if (iter != shaderProgramPool.end())
			{
				return iter->second.get();
			}
			return nullptr;
		}

		static ShaderProgram *Find(size_t uid)
		{
			auto iter = shaderProgramPool.find(uid);
			if (iter != shaderProgramPool.end())
			{
				return iter->second.get();
			}
			return nullptr;
		}

		static const vkb::ShaderSource &FindShaderSource(size_t uid)
		{
			auto iter = shaderSourcePool.find(uid);
			if (iter != shaderSourcePool.end())
			{
				return iter->second;
			}
			throw std::runtime_error("Can't find shader with UID : " + uid);
		}

		static const vkb::ShaderSource &FindShaderSource(std::string &name)
		{
			size_t uid  = std::hash<std::string>{}(name);
			auto   iter = shaderSourcePool.find(uid);
			if (iter != shaderSourcePool.end())
			{
				return iter->second;
			}
			throw std::runtime_error("Can't find shader with name : " + name);
		}

		static size_t AddShaderSource(vkb::ShaderSource &&shaderSource)
		{
			size_t uid = std::hash<std::string>{}(shaderSource.get_filename());
			DEBUG_ASSERT(shaderSourcePool.find(uid) == shaderSourcePool.end(), "Shader source uid collision");
			shaderSourcePool.emplace(uid, std::move(shaderSource));
			return uid;
		}

	  private:
		static std::unordered_map<size_t, std::shared_ptr<ShaderProgram>> shaderProgramPool;
		static std::unordered_map<size_t, vkb::ShaderSource>              shaderSourcePool;
		VkShaderStageFlagBits                                             programStageFlag;
		std::vector<vkb::ShaderModule *>                                  shaderModules;
	};

	static const vkb::RenderTarget::CreateFunc swap_chain_create_func;

  public:
	forward_plus();
	virtual ~forward_plus();

  private:
	virtual bool                            prepare(vkb::Platform &platform) override;
	virtual void                            prepare_render_context() override;
	virtual const std::vector<const char *> get_validation_layers() override;
	virtual void                            request_gpu_features(vkb::PhysicalDevice &gpu) override;
	virtual void                            resize(const uint32_t width, const uint32_t height) override;
	virtual void                            update(float delta_time) override;
	virtual void                            input_event(const vkb::InputEvent &input_event) override;

	void prepare_resources();
	void prepare_camera();
	void prepare_pipelines();
	void prepare_buffer();
	void prepare_light();
	void render(float delta_time);
	void update_global_uniform_buffers(vkb::CommandBuffer &commandBuffer, vkb::sg::Node *node);
	void bind_pipeline_state(vkb::CommandBuffer &commandBuffer, vkb::PipelineState &pipeline);
	void bind_pbr_descriptor(vkb::CommandBuffer &commandBuffer, vkb::PipelineLayout &pipelineLayout, vkb::sg::SubMesh *submesh);
	bool bind_vertex_input(vkb::CommandBuffer &commandBuffer, vkb::PipelineLayout &pipelineLayout, vkb::sg::SubMesh *submesh = nullptr);
	void blit_and_present(vkb::CommandBuffer &commandBuffer);
	void get_sorted_nodes(std::multimap<float, std::pair<vkb::sg::Node *, vkb::sg::SubMesh *>> &opaque_nodes, std::multimap<float, std::pair<vkb::sg::Node *, vkb::sg::SubMesh *>> &transparent_nodes);

  private:
	const std::string k_title = "Vulkan Example";
	const std::string k_name  = "Forward Plus";

	vkb::sg::Camera *camera{nullptr};

	/*                            Rendering                        */
	bool debugDepth{false};
	bool supportBlit{false};

	glm::vec4 sunDirection = glm::normalize(glm::vec4(-1.0, -1.0, 0.0, 0.0));
	glm::vec4 sunColor     = glm::vec4(1.0, 1.0, 1.0, 1.0);

	std::shared_ptr<vkb::core::Image>  linearDepthImage{nullptr};
	std::shared_ptr<vkb::core::Buffer> lightBuffer{nullptr};
	std::shared_ptr<vkb::core::Buffer> lightGridBuffer{nullptr};
	std::shared_ptr<vkb::core::Buffer> lightMaskBuffer{nullptr};
	std::shared_ptr<vkb::core::Buffer> postProcessVB{nullptr};

	std::shared_ptr<vkb::core::ImageView> linearDepthImageView{nullptr};

	std::shared_ptr<vkb::core::Sampler> linearClampSampler{nullptr};

	RenderPassEntry  depthPrePass{};
	ComputePassEntry linearDepthPass{};
	ComputePassEntry lightGridPass{};
	RenderPassEntry  debugDepthPass{};
	RenderPassEntry  opaquePass{};
};

std::unique_ptr<vkb::Application> create_forward_plus();