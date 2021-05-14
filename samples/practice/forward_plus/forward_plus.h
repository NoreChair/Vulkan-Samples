/*
 * Forward plus rendering
 */
#pragma once

#include "common/vk_common.h"
#include "platform/platform.h"
#include "vulkan_sample.h"
#include <initializer_list>
#include <unordered_map>

#include "Components/shadow_camera.h"
#include "RenderPass/debug_draw_pass.h"
#include "RenderPass/depth_only_pass.h"
#include "RenderPass/light_grid_pass.h"
#include "RenderPass/linear_depth_pass.h"
#include "RenderPass/opaque_pass.h"
#include "RenderPass/show_depth_pass.h"

class vkb::sg::Node;
class vkb::sg::SubMesh;

class forward_plus : public vkb::VulkanSample
{
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
	shadow_camera *light_camera{nullptr};

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