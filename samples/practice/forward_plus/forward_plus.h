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
#include "RenderPass/screen_shadow_pass.h"
#include "RenderPass/show_depth_pass.h"
#include "RenderPass/TAA.h"
#include "RenderPass/bloom_pass.h"

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
	virtual void finish() override;

	virtual const std::vector<const char *> get_validation_layers() override;

	void prepare_shaders();
	void prepare_scene();
	void prepare_pipelines();
	void prepare_light();

    void process_shadow(vkb::CommandBuffer& commandBuffer);
    void process_light_buffer(vkb::CommandBuffer& commandBuffer);
    void process_HDR(vkb::CommandBuffer& commandBuffer);
    void update_scene(float delta_time);
	void render(float delta_time);
	void get_sorted_nodes(glm::vec3 direction, glm::vec3 position, std::multimap<float, std::pair<vkb::sg::Node *, vkb::sg::SubMesh *>> *opaque_nodes, std::multimap<float, std::pair<vkb::sg::Node *, vkb::sg::SubMesh *>> *transparent_nodes = nullptr);
	void full_screen_draw(vkb::CommandBuffer &command_buffer, vkb::core::ImageView &target, std::function<void(vkb::CommandBuffer &, vkb::RenderContext &)> body);

  private:
	const std::string k_title = "Vulkan Example";
	const std::string k_name  = "Forward Plus";

	vkb::sg::Camera *camera{nullptr};
	shadow_camera *  light_camera{nullptr};

	/*                            Rendering                        */
	bool drawAABB{false};
	bool drawLight{false};
	bool debugDepth{false};

    bool enableBloom{true};
    bool enableTemporalAA{true};
    bool historyTAA{false};

    int sourceTargetIndex = 0;
    int destTargetIndex = 1;
    float sharpenValue = 0.25f;
    float targetLumin = 0.3f;

	std::unique_ptr<vkb::sg::SubMesh> sphere_mesh{nullptr};
	std::unique_ptr<vkb::sg::SubMesh> cube_mesh{nullptr};

	std::unique_ptr<depth_only_pass>    depthPrePass{nullptr};
	std::unique_ptr<depth_only_pass>    shadowPass{nullptr};
	std::unique_ptr<screen_shadow_pass> screenShadowPass{nullptr};
	std::unique_ptr<linear_depth_pass>  linearDepthPass{nullptr};
	std::unique_ptr<light_grid_pass>    lightGridPass{nullptr};
	std::unique_ptr<show_depth_pass>    showDepthPass{nullptr};
	std::unique_ptr<opaque_pass>        opaquePass{nullptr};
	std::unique_ptr<debug_draw_pass>    debugDrawPass{nullptr};
    std::unique_ptr<TAA>                temporalAAPass{nullptr};
    std::unique_ptr<bloom_pass>         bloomPass{nullptr};
};

std::unique_ptr<vkb::Application> create_forward_plus();