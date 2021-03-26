/*
 * Forward plus rendering
 */
#pragma once

#include "camera.h"
#include "vulkan_sample.h"

class forward_plus : public vkb::VulkanSample
{
	struct Texture
	{
		std::unique_ptr<vkb::sg::Image> image;
		VkSampler                       sampler;
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

	struct RenderPipeline
	{
		vkb::Pipeline       pbo;
		vkb::PipelineLayout layout;
	};

	struct Pass
	{
		vkb::Framebuffer              frameBuffer;
		vkb::RenderPass               renderPass;
		std::vector<RenderPipeline *> pipeline;
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
	void on_update_ui_overlay(vkb::Drawer &drawer);
	void create_attachment(VkFormat format, VkImageUsageFlagBits usage, vkb::Framebuffer *framebuffer);
	void prepare_offscreen_buffer();
	void load_assets();
	void setup_descriptor_pool();
	void setup_descriptor_set_layout();
	void setup_descriptor_sets();
	void prepare_pipelines();
	void prepare_uniform_buffers();
	void update_uniform_buffers();
	void update_params();
	void draw();

  private:
	const std::string title = "Vulkan Example";
	const std::string name  = "Forward Plus";

	/*                            Camera                           */
	float       zoom         = 0;
	bool        view_updated = false;
	glm::vec3   rotation     = glm::vec3();
	glm::vec3   camera_pos   = glm::vec3();
	vkb::Camera camera;

	/*                            Input                           */
	// true if application has focused, false if moved to background
	bool focused    = false;
	bool touch_down = false;
	// Use to adjust mouse rotation speed
	float rotation_speed = 1.0f;
	// Use to adjust mouse zoom speed
	float     zoom_speed    = 1.0f;
	double    touch_timer   = 0.0;
	int64_t   last_tap_time = 0;
	glm::vec2 mouse_pos;

	struct MouseButton
	{
		bool left   = false;
		bool right  = false;
		bool middle = false;
	} mouse_buttons;
	struct TouchPos
	{
		int32_t x;
		int32_t y;
	} touch_pos;
};