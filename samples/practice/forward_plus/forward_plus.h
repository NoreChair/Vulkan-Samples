/*
 * Forward plus rendering
 */
#pragma once

#include "common/vk_common.h"
#include "platform/platform.h"
#include "vulkan_sample.h"
#include <initializer_list>
#include <unordered_map>

#include "debug_draw_pass.h"
#include "depth_only_pass.h"
#include "light_grid_pass.h"
#include "linear_depth_pass.h"
#include "opaque_pass.h"
#include "show_depth_pass.h"

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

		static vkb::ShaderSource &FindShaderSource(size_t uid)
		{
			auto iter = shaderSourcePool.find(uid);
			if (iter != shaderSourcePool.end())
			{
				return iter->second;
			}
			throw std::runtime_error("Can't find shader with UID : " + uid);
		}

		static vkb::ShaderSource &FindShaderSource(std::string &name)
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
	virtual bool prepare(vkb::Platform &platform) override;
	virtual void prepare_render_context() override;
	virtual void request_gpu_features(vkb::PhysicalDevice &gpu) override;
	virtual void resize(const uint32_t width, const uint32_t height) override;
	virtual void update(float delta_time) override;
	virtual void input_event(const vkb::InputEvent &input_event) override;
	virtual void draw_gui();

	virtual const std::vector<const char *> get_validation_layers() override;

	void prepare_shaders();
	void prepare_scene();
	void prepare_pipelines();
	void prepare_buffer();
	void prepare_light();

	void render(float delta_time);
	void blit_and_present(vkb::CommandBuffer &commandBuffer);
	void get_sorted_nodes(std::multimap<float, std::pair<vkb::sg::Node *, vkb::sg::SubMesh *>> &opaque_nodes, std::multimap<float, std::pair<vkb::sg::Node *, vkb::sg::SubMesh *>> &transparent_nodes);

  private:
	const std::string k_title = "Vulkan Example";
	const std::string k_name  = "Forward Plus";

	vkb::sg::Camera *camera{nullptr};

	/*                            Rendering                        */
	bool drawAABB{false};
	bool drawLight{false};
	bool debugDepth{false};
	bool supportBlit{false};

	std::shared_ptr<vkb::core::Image>     linearDepthImage{nullptr};
	std::shared_ptr<vkb::core::Buffer>    lightBuffer{nullptr};
	std::shared_ptr<vkb::core::Buffer>    lightGridBuffer{nullptr};
	std::shared_ptr<vkb::core::Buffer>    lightMaskBuffer{nullptr};
	std::shared_ptr<vkb::core::Buffer>    postProcessVB{nullptr};
	std::shared_ptr<vkb::RenderTarget>    offScreenRT{nullptr};
	std::shared_ptr<vkb::core::ImageView> linearDepthImageView{nullptr};

	std::unique_ptr<vkb::sg::SubMesh> sphere_mesh{nullptr};
	std::unique_ptr<vkb::sg::SubMesh> cube_mesh{nullptr};

	std::unique_ptr<light_grid_pass>   lightGridPass{nullptr};
	std::unique_ptr<linear_depth_pass> linearDepthPass{nullptr};
	std::unique_ptr<show_depth_pass>   showDepthPass{nullptr};
	std::unique_ptr<depth_only_pass>   depthPrePass{nullptr};
	std::unique_ptr<opaque_pass>       opaquePass{nullptr};
	std::unique_ptr<debug_draw_pass>   debugDrawPass{nullptr};
};

std::unique_ptr<vkb::Application> create_forward_plus();