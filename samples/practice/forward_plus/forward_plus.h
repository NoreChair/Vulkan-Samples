/*
 * Forward plus rendering
 */
#pragma once

#include "camera.h"
#include "platform/platform.h"
#include "vulkan_sample.h"
#include <initializer_list>
#include <unordered_map>

#if defined(DEBUG) || defined(_DEBUG)
#	define DEBUG_ASSERT(exp, ...) \
		{                          \
			assert(exp);           \
			LOGD(__VA_ARGS__)      \
		}
#else
#	define DEBUG_ASSERT(exp, ...) 0
#endif

class vkb::sg::Node;
class vkb::sg::SubMesh;

class forward_plus : public vkb::VulkanSample
{
	struct MouseButton
	{
		bool left   = false;
		bool right  = false;
		bool middle = false;
	};

	struct TouchPos
	{
		int32_t x = 0;
		int32_t y = 0;
	};

	enum RenderPassOrder : uint32_t
	{
		DepthPrePassOrder = 1000
	};

	struct alignas(16) GlobalUniform
	{
		glm::mat4 model;
		glm::mat4 view_project;
		glm::vec3 camera_position;
	};

	struct RenderPassEntry
	{
		std::shared_ptr<vkb::RenderTarget>   renderTarget;
		vkb::Framebuffer *                   frameBuffer;
		vkb::RenderPass *                    renderPass;
		std::map<size_t, vkb::PipelineState> pipelines;

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
			DEBUG_ASSERT(shaderProgramPool.find(uid) != shaderProgramPool.end(), "Shader program uid collision");
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

	  private:
		static std::unordered_map<size_t, std::shared_ptr<ShaderProgram>> shaderProgramPool;
		VkShaderStageFlagBits                                             programStageFlag;
		std::vector<vkb::ShaderModule *>                                  shaderModules;
	};

  public:
	forward_plus();
	~forward_plus();

  private:
	virtual bool prepare(vkb::Platform &platform) override;
	virtual void prepare_render_context() override;
	virtual void request_gpu_features(vkb::PhysicalDevice &gpu) override;
	virtual void resize(const uint32_t width, const uint32_t height) override;
	virtual void update(float delta_time) override;
	virtual void input_event(const vkb::InputEvent &input_event) override;

	void handle_mouse_move(int32_t x, int32_t y);
	void prepare_resources();
	void prepare_camera();
	void prepare_pipelines();
	void render(float delta_time);
	void update_global_uniform_buffers(vkb::CommandBuffer &commandBuffer, vkb::sg::Node *node);
	void bind_pipeline_state(vkb::CommandBuffer &commandBuffer, vkb::PipelineState &pipeline);
	void bind_descriptor(vkb::CommandBuffer &commandBuffer, vkb::sg::SubMesh *submesh, vkb::PipelineState &pipeline, bool bindMaterial = false);
	bool bind_vertex_input(vkb::CommandBuffer &commandBuffer, vkb::sg::SubMesh *submesh, vkb::PipelineState &pipeline);
	void blit_and_present(vkb::CommandBuffer &commandBuffer);
	void get_sorted_nodes(std::multimap<float, std::pair<vkb::sg::Node *, vkb::sg::SubMesh *>> &opaque_nodes, std::multimap<float, std::pair<vkb::sg::Node *, vkb::sg::SubMesh *>> &transparent_nodes);

  private:
	const std::string k_title = "Vulkan Example";
	const std::string k_name  = "Forward Plus";

	/*                            Camera                           */
	float       zoom        = 0;
	bool        viewUpdated = false;
	glm::vec3   rotation    = glm::vec3();
	glm::vec3   cameraPos   = glm::vec3();
	vkb::Camera camera;

	/*                            Input                           */
	// true if application has focused, false if moved to background
	bool focused   = false;
	bool touchDown = false;
	// Use to adjust mouse rotation speed
	float rotationSpeed = 1.0f;
	// Use to adjust mouse zoom speed
	float       zoomSpeed   = 1.0f;
	double      touchTimer  = 0.0;
	int64_t     lastTapTime = 0;
	glm::vec2   mousePos    = glm::vec2();
	MouseButton mouseButtons;
	TouchPos    touchPos;

	/*                            Rendering                        */
	bool supportBlit = false;
	RenderPassEntry opaquePass{};
};

std::unique_ptr<vkb::Application> create_forward_plus();