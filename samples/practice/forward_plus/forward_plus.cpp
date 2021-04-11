#include "forward_plus.h"
#include "api_vulkan_sample.h"
#include "gltf_loader.h"
#include "scene_graph/node.h"
#include "scene_graph/components/material.h"
#include "scene_graph/components/mesh.h"
#include "scene_graph/components/sub_mesh.h"

using namespace vkb;
using namespace vkb::core;

std::unordered_map<size_t, std::shared_ptr<forward_plus::ShaderProgram>> forward_plus::ShaderProgram::shaderProgramPool = std::unordered_map<size_t, std::shared_ptr<forward_plus::ShaderProgram>>();

forward_plus::forward_plus()
{
	set_name(k_name);
}

forward_plus::~forward_plus()
{
}

bool forward_plus::prepare(Platform &platform)
{
	if (!VulkanSample::prepare(platform))
	{
		return false;
	}
	uint32_t windowWidth  = platform.get_window().get_width();
	uint32_t windowHeight = platform.get_window().get_height();

	Device &refDevice = *device.get();

	prepare_resources();
	prepare_camera();

	// Create Render Image
	VkExtent3D extent{windowWidth, windowHeight, 1};
	Image      depthImage(refDevice, extent, VK_FORMAT_D32_SFLOAT, VK_IMAGE_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT | VK_IMAGE_USAGE_STORAGE_BIT | VK_IMAGE_USAGE_SAMPLED_BIT, VmaMemoryUsage::VMA_MEMORY_USAGE_GPU_ONLY, VK_SAMPLE_COUNT_1_BIT, 1, 1, VK_IMAGE_TILING_OPTIMAL, 0, 0, nullptr);
	Image      colorImage(refDevice, extent, VK_FORMAT_R8G8B8A8_SRGB, VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_SAMPLED_BIT, VmaMemoryUsage::VMA_MEMORY_USAGE_GPU_ONLY);

	// Create Render Target
	std::vector<Image> offScreenImages;
	offScreenImages.push_back(std::move(colorImage));
	offScreenImages.push_back(std::move(depthImage));
	std::shared_ptr<RenderTarget> renderTarget = std::make_shared<RenderTarget>(std::move(offScreenImages));

	// Public pipeline state
	// gltf_loader::load_scene seperate vertex data in its own buffer
	// TODO : separate skinning mesh and static mesh
	VertexInputState defaultVertexInput;
	defaultVertexInput.bindings.emplace_back(VkVertexInputBindingDescription{0, sizeof(Vertex), VK_VERTEX_INPUT_RATE_VERTEX});
	defaultVertexInput.attributes.emplace_back(VkVertexInputAttributeDescription{0, 0, VK_FORMAT_R32G32B32_SFLOAT, 0});        //Position
	defaultVertexInput.attributes.emplace_back(VkVertexInputAttributeDescription{1, 0, VK_FORMAT_R32G32B32_SFLOAT, 0});        // Normal
	defaultVertexInput.attributes.emplace_back(VkVertexInputAttributeDescription{2, 0, VK_FORMAT_R32G32_SFLOAT, 0});           // UV

	DepthStencilState defaultDepthState;        // depth test/write

	ColorBlendState defaultColorState;
	ColorBlendState depthOnlyColorState;
	{
		ColorBlendAttachmentState defaultAttaState;
		//ColorBlendAttachmentState blendAttaState;
		//blendAttaState.blend_enable           = VK_TRUE;
		//blendAttaState.src_color_blend_factor = blendAttaState.src_alpha_blend_factor = VK_BLEND_FACTOR_SRC_ALPHA;
		//blendAttaState.dst_color_blend_factor = blendAttaState.dst_alpha_blend_factor = VK_BLEND_FACTOR_ONE_MINUS_SRC_ALPHA;
		ColorBlendAttachmentState depthOnlyAttaState;
		depthOnlyAttaState.color_write_mask = 0;

		defaultColorState.attachments.push_back(defaultAttaState);
		depthOnlyColorState.attachments.push_back(depthOnlyAttaState);
	}

	{
		// Opaque
		std::vector<LoadStoreInfo> loadStoreInfos;
		loadStoreInfos.emplace_back(LoadStoreInfo{VK_ATTACHMENT_LOAD_OP_CLEAR, VK_ATTACHMENT_STORE_OP_STORE});
		loadStoreInfos.emplace_back(LoadStoreInfo{VK_ATTACHMENT_LOAD_OP_CLEAR, VK_ATTACHMENT_STORE_OP_STORE});
		std::vector<SubpassInfo> subPassInfos;
		subPassInfos.emplace_back(SubpassInfo{{}, {0}, {}, false, 0, VK_RESOLVE_MODE_NONE});
		opaquePass.renderTarget = renderTarget;
		opaquePass.renderPass   = std::make_unique<RenderPass>(refDevice, renderTarget->get_attachments(), loadStoreInfos, subPassInfos);
		opaquePass.frameBuffer  = std::make_unique<Framebuffer>(refDevice, *renderTarget.get(), *opaquePass.renderPass.get());

		{
			// Dpeth Pass
			ShaderProgram * depthProgram = ShaderProgram::Find(std::string("forward_plus/depth_only"));
			VkPipelineCache pipelineCache{};
			PipelineLayout  layout(refDevice, depthProgram->GetShaderModules());
			PipelineState   depthOnlyPipelineState{};
			depthOnlyPipelineState.set_pipeline_layout(layout);
			depthOnlyPipelineState.set_render_pass(*opaquePass.renderPass.get());
			depthOnlyPipelineState.set_vertex_input_state(defaultVertexInput);
			depthOnlyPipelineState.set_depth_stencil_state(defaultDepthState);
			depthOnlyPipelineState.set_color_blend_state(depthOnlyColorState);
			auto pbo = std::make_unique<GraphicsPipeline>(refDevice, pipelineCache, depthOnlyPipelineState);
			opaquePass.pipelines.insert(std::move(std::make_pair((uint32_t) DepthPrePassOrder, std::move(pbo))));
		}
	}
	return true;
}

void forward_plus::update(float delta_time)
{
	camera.update(delta_time);
	update_scene(delta_time);
	//update_gui(delta_time);

	std::multimap<float, std::pair<sg::Node *, sg::SubMesh *>> opaque_nodes;
	std::multimap<float, std::pair<sg::Node *, sg::SubMesh *>> transparent_nodes;
	get_sorted_nodes(opaque_nodes, transparent_nodes);

	// reverse z
	std::vector<VkClearValue> clearValue{initializers::clear_color_value(0.0, 0.0, 0.0, 0.0), initializers::clear_depth_stencil_value(1.0, 0)};
	VkExtent2D                extent  = get_render_context().get_surface_extent();
	RenderContext &           context = get_render_context();

	CommandBuffer &commandBuffer = context.begin();
	commandBuffer.begin(VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT);
	set_viewport_and_scissor(commandBuffer, extent);

	// Opaque
	{
		commandBuffer.begin_render_pass(opaquePass.GetRenderTarget(), opaquePass.GetRenderPass(), opaquePass.GetFrameBuffer(), clearValue);
		// TODO : bind pipeline and descripotr set
		// draw to render target and presen to screen
		for (auto node_it = opaque_nodes.begin(); node_it != opaque_nodes.end(); node_it++)
		{
		}

		commandBuffer.end_render_pass();
	}
	commandBuffer.end();
	context.submit(commandBuffer);
}

void forward_plus::get_sorted_nodes(std::multimap<float, std::pair<sg::Node *, sg::SubMesh *>> &opaque_nodes, std::multimap<float, std::pair<sg::Node *, sg::SubMesh *>> &transparent_nodes)
{
	auto camera_transform = camera.matrices.view;
	auto meshes           = scene->get_components<sg::Mesh>();
	for (auto &mesh : meshes)
	{
		for (auto &node : mesh->get_nodes())
		{
			auto node_transform = node->get_transform().get_world_matrix();

			const sg::AABB &mesh_bounds = mesh->get_bounds();

			sg::AABB world_bounds{mesh_bounds.get_min(), mesh_bounds.get_max()};
			world_bounds.transform(node_transform);

			float distance = glm::length(glm::vec3(camera_transform[3]) - world_bounds.get_center());

			for (auto &sub_mesh : mesh->get_submeshes())
			{
				if (sub_mesh->get_material()->alpha_mode == sg::AlphaMode::Blend)
				{
					transparent_nodes.emplace(distance, std::make_pair(node, sub_mesh));
				}
				else
				{
					opaque_nodes.emplace(distance, std::make_pair(node, sub_mesh));
				}
			}
		}
	}
}

void forward_plus::request_gpu_features(vkb::PhysicalDevice &gpu)
{
	VulkanSample::request_gpu_features(gpu);
}

void forward_plus::resize(const uint32_t width, const uint32_t height)
{
	VulkanSample::resize(width, height);
}

void forward_plus::input_event(const vkb::InputEvent &input_event)
{
	VulkanSample::input_event(input_event);

	bool gui_captures_event = false;

	if (gui)
	{
		gui_captures_event = gui->input_event(input_event);
	}

	if (!gui_captures_event)
	{
		if (input_event.get_source() == vkb::EventSource::Mouse)
		{
			const auto &mouse_button = static_cast<const vkb::MouseButtonInputEvent &>(input_event);

			handle_mouse_move(static_cast<int32_t>(mouse_button.get_pos_x()), static_cast<int32_t>(mouse_button.get_pos_y()));

			if (mouse_button.get_action() == vkb::MouseAction::Down)
			{
				switch (mouse_button.get_button())
				{
					case vkb::MouseButton::Left:
						mouseButtons.left = true;
						break;
					case vkb::MouseButton::Right:
						mouseButtons.right = true;
						break;
					case vkb::MouseButton::Middle:
						mouseButtons.middle = true;
						break;
					default:
						break;
				}
			}
			else if (mouse_button.get_action() == vkb::MouseAction::Up)
			{
				switch (mouse_button.get_button())
				{
					case vkb::MouseButton::Left:
						mouseButtons.left = false;
						break;
					case vkb::MouseButton::Right:
						mouseButtons.right = false;
						break;
					case vkb::MouseButton::Middle:
						mouseButtons.middle = false;
						break;
					default:
						break;
				}
			}
		}
		else if (input_event.get_source() == vkb::EventSource::Touchscreen)
		{
			const auto &touch_event = static_cast<const vkb::TouchInputEvent &>(input_event);

			if (touch_event.get_action() == vkb::TouchAction::Down)
			{
				touchDown         = true;
				touchPos.x        = static_cast<int32_t>(touch_event.get_pos_x());
				touchPos.y        = static_cast<int32_t>(touch_event.get_pos_y());
				mousePos.x        = touch_event.get_pos_x();
				mousePos.y        = touch_event.get_pos_y();
				mouseButtons.left = true;
			}
			else if (touch_event.get_action() == vkb::TouchAction::Up)
			{
				touchPos.x        = static_cast<int32_t>(touch_event.get_pos_x());
				touchPos.y        = static_cast<int32_t>(touch_event.get_pos_y());
				touchTimer        = 0.0;
				touchDown         = false;
				camera.keys.up    = false;
				mouseButtons.left = false;
			}
			else if (touch_event.get_action() == vkb::TouchAction::Move)
			{
				bool handled = false;
				if (gui)
				{
					ImGuiIO &io = ImGui::GetIO();
					handled     = io.WantCaptureMouse;
				}
				if (!handled)
				{
					int32_t eventX = static_cast<int32_t>(touch_event.get_pos_x());
					int32_t eventY = static_cast<int32_t>(touch_event.get_pos_y());

					float deltaX = (float) (touchPos.y - eventY) * rotationSpeed * 0.5f;
					float deltaY = (float) (touchPos.x - eventX) * rotationSpeed * 0.5f;

					camera.rotate(glm::vec3(deltaX, 0.0f, 0.0f));
					camera.rotate(glm::vec3(0.0f, -deltaY, 0.0f));

					rotation.x += deltaX;
					rotation.y -= deltaY;

					touchPos.x = eventX;
					touchPos.y = eventY;
				}
			}
		}
		else if (input_event.get_source() == vkb::EventSource::Keyboard)
		{
			const auto &key_button = static_cast<const vkb::KeyInputEvent &>(input_event);

			if (key_button.get_action() == vkb::KeyAction::Down)
			{
				switch (key_button.get_code())
				{
					case vkb::KeyCode::W:
						camera.keys.up = true;
						break;
					case vkb::KeyCode::S:
						camera.keys.down = true;
						break;
					case vkb::KeyCode::A:
						camera.keys.left = true;
						break;
					case vkb::KeyCode::D:
						camera.keys.right = true;
						break;
					default:
						break;
				}
			}
			else if (key_button.get_action() == vkb::KeyAction::Up)
			{
				switch (key_button.get_code())
				{
					case vkb::KeyCode::W:
						camera.keys.up = false;
						break;
					case vkb::KeyCode::S:
						camera.keys.down = false;
						break;
					case vkb::KeyCode::A:
						camera.keys.left = false;
						break;
					case vkb::KeyCode::D:
						camera.keys.right = false;
						break;
					default:
						break;
				}
			}
		}
	}
}

void forward_plus::handle_mouse_move(int32_t x, int32_t y)
{
	int32_t dx = (int32_t) mousePos.x - x;
	int32_t dy = (int32_t) mousePos.y - y;

	bool handled = false;

	if (gui)
	{
		ImGuiIO &io = ImGui::GetIO();
		handled     = io.WantCaptureMouse;
	}

	if (handled)
	{
		mousePos = glm::vec2((float) x, (float) y);
		return;
	}

	if (mouseButtons.left)
	{
		rotation.x += dy * 1.25f * rotationSpeed;
		rotation.y -= dx * 1.25f * rotationSpeed;
		camera.rotate(glm::vec3(dy * camera.rotation_speed, -dx * camera.rotation_speed, 0.0f));
		viewUpdated = true;
	}
	if (mouseButtons.right)
	{
		zoom += dy * .005f * zoomSpeed;
		camera.translate(glm::vec3(-0.0f, 0.0f, dy * .005f * zoomSpeed));
		viewUpdated = true;
	}
	if (mouseButtons.middle)
	{
		cameraPos.x -= dx * 0.01f;
		cameraPos.y -= dy * 0.01f;
		camera.translate(glm::vec3(-dx * 0.01f, -dy * 0.01f, 0.0f));
		viewUpdated = true;
	}
	mousePos = glm::vec2((float) x, (float) y);
}

void forward_plus::prepare_resources()
{
	Device &refDevice = *device.get();
	{
		// Load Shader
		// TODO : shader variant
		ShaderVariant            emptyVariant{};
		std::vector<std::string> shaderSourceFiles{"forward_plus/depth_only"};
		std::vector<std::string> computeSourceFiles{};
		// assmue shader source contain vertex and fragment
		for (int i = 0; i < shaderSourceFiles.size(); ++i)
		{
			ShaderSource vsSource(shaderSourceFiles[i] + ".vert");
			ShaderSource fsSource(shaderSourceFiles[i] + ".frag");
			auto         shaderVS = std::make_shared<ShaderModule>(refDevice, VK_SHADER_STAGE_VERTEX_BIT, vsSource, k_shaderEntry, emptyVariant);
			auto         shaderFS = std::make_shared<ShaderModule>(refDevice, VK_SHADER_STAGE_FRAGMENT_BIT, fsSource, k_shaderEntry, emptyVariant);

			std::initializer_list<std::shared_ptr<ShaderModule>> initList{std::move(shaderVS), std::move(shaderFS)};

			auto program = std::make_shared<ShaderProgram>(std::move(initList));
			ShaderProgram::AddShaderProgram(shaderSourceFiles[i], std::move(program));
		}

		for (int i = 0; i < computeSourceFiles.size(); i++)
		{
			ShaderSource source(computeSourceFiles[i] + ".comp");
			auto         shaderCS = std::make_shared<ShaderModule>(refDevice, VK_SHADER_STAGE_COMPUTE_BIT, source, k_shaderEntry, emptyVariant);
			auto         program  = std::make_shared<ShaderProgram>(std::move(shaderCS));
			ShaderProgram::AddShaderProgram(computeSourceFiles[i], std::move(program));
		}
	}

	{
		// Scene contain lighting/Texture(both image and sampler)/Material/Mesh/Camera
		load_scene("scenes/sponza/Sponza01.gltf");
	}
}

void forward_plus::prepare_camera()
{
	VkExtent2D extent = get_render_context().get_surface_extent();
	camera.type       = vkb::CameraType::LookAt;
	camera.set_position(glm::vec3(0.0f, 0.0f, -4.0f));
	camera.set_rotation(glm::vec3(0.0f, 180.0f, 0.0f));
	// Note: Using Revsered depth-buffer for increased precision, so Znear and Zfar are flipped
	camera.set_perspective(60.0f, (float) extent.width / (float) extent.height, 1000.0f, 0.1f);
}

void forward_plus::render(float delta_time)
{
}

void forward_plus::prepare_pipelines()
{
}

void forward_plus::prepare_uniform_buffers()
{
}

void forward_plus::update_uniform_buffers()
{
}

std::unique_ptr<vkb::Application> create_forward_plus()
{
	return std::unique_ptr<forward_plus>();
}
