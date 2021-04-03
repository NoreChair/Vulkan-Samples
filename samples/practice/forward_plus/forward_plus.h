/*
 * Forward plus rendering
 */
#pragma once

#include "camera.h"
#include "platform/platform.h"
#include "vulkan_sample.h"
#include <initializer_list>
#include <unordered_map>

class forward_plus : public vkb::VulkanSample
{
	enum RenderPassOrder : uint32_t
	{
		DepthPrePassOrder = 1000
	};

	// Synchronization semaphores
	struct FrameBarrier
	{
		// Swap chain image presentation
		VkSemaphore acquired_image_ready;

		// Command buffer submission and execution
		VkSemaphore render_complete;
		VkFence     fence;
	};

	struct Models
	{
		std::unique_ptr<vkb::sg::SubMesh>              skybox;
		std::vector<std::unique_ptr<vkb::sg::SubMesh>> objects;
		std::vector<glm::mat4>                         transforms;
		int32_t                                        object_index = 0;
	};

	struct UniformBuffers
	{
		std::unique_ptr<vkb::core::Buffer> matrix;
		std::unique_ptr<vkb::core::Buffer> materialParams;
	};

	struct MatrixUniforms
	{
		glm::mat4 model_view;
		glm::mat4 project;
	};

	struct RenderPassEntry
	{
		std::unique_ptr<vkb::Framebuffer>                        frameBuffer;
		std::unique_ptr<vkb::RenderPass>                         renderPass;
		std::map<size_t, std::unique_ptr<vkb::GraphicsPipeline>> pipelines;
	};

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

	struct ShaderProgram
	{
		ShaderProgram(std::shared_ptr<vkb::ShaderModule>&& shader)
		{
			shaderModules.emplace_back(shader);
		}

		ShaderProgram(std::initializer_list<std::shared_ptr<vkb::ShaderModule>> &&params)
		{
			for (auto ptr = params.begin(); ptr != params.end(); ++ptr)
			{
				shaderModules.emplace_back(*ptr);
			}
		}

		std::vector<std::shared_ptr<vkb::ShaderModule>> shaderModules;
	};

  private:
	forward_plus();
	~forward_plus();

	virtual bool prepare(vkb::Platform &platform) override;
	virtual void request_gpu_features(vkb::PhysicalDevice &gpu) override;
	virtual void resize(const uint32_t width, const uint32_t height) override;

	virtual void input_event(const vkb::InputEvent &input_event) override;
	void         handle_mouse_move(int32_t x, int32_t y);

	void render(float delta_time);
	void prepare_offscreen_buffer();
	void load_assets();
	void prepare_pipelines();
	void prepare_uniform_buffers();
	void update_uniform_buffers();

  private:
	const std::string k_title               = "Vulkan Example";
	const std::string k_name                = "Forward Plus";
	const std::string k_vertexShaderEntry   = "VertexEntry";
	const std::string k_fragmentShaderEntry = "FragmentEntry";
	const std::string k_computeShaderEntry  = "ComputeEntry";
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
	std::unique_ptr<vkb::FencePool>           fencesPool;
	std::unique_ptr<vkb::SemaphorePool>       semaphorePool;
	std::unordered_map<size_t, ShaderProgram> shaderProgramPool;
	std::vector<FrameBarrier>                 frameBarrier;
	RenderPassEntry                           depthPass{};
};

std::unique_ptr<vkb::Application> create_forward_plus();