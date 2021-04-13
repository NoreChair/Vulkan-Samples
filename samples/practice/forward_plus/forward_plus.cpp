#include "forward_plus.h"
#include "api_vulkan_sample.h"
#include "gltf_loader.h"
#include "scene_graph/components/material.h"
#include "scene_graph/components/mesh.h"
#include "scene_graph/components/pbr_material.h"
#include "scene_graph/components/sub_mesh.h"
#include "scene_graph/node.h"

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

	prepare_resources();
	prepare_camera();
	prepare_pipelines();

	return true;
}

void forward_plus::prepare_render_context()
{
	// so we can just copy offscreen image to swap chain image
	auto &properties = render_context->get_swapchain().get_properties();
	properties.image_usage |= VK_IMAGE_USAGE_TRANSFER_DST_BIT;
	render_context->prepare();
}

void forward_plus::update(float delta_time)
{
	camera.update(delta_time);
	update_scene(delta_time);
	//update_gui(delta_time);
	render(delta_time);
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
	supportBlit = (gpu.get_format_properties(VK_FORMAT_R8G8B8A8_SRGB).optimalTilingFeatures | (VK_FORMAT_FEATURE_BLIT_SRC_BIT & VK_FORMAT_FEATURE_BLIT_DST_BIT)) == (VK_FORMAT_FEATURE_BLIT_SRC_BIT & VK_FORMAT_FEATURE_BLIT_DST_BIT);
	assert(supportBlit);
}

void forward_plus::resize(const uint32_t width, const uint32_t height)
{
	VulkanSample::resize(width, height);
}

void forward_plus::prepare_pipelines()
{
	auto &   extent2d     = get_render_context().get_surface_extent();
	uint32_t windowWidth  = extent2d.width;
	uint32_t windowHeight = extent2d.height;

	Device &refDevice = *device.get();
	// Create Render Image
	VkExtent3D extent{windowWidth, windowHeight, 1};
	Image      depthImage(refDevice, extent, VK_FORMAT_D32_SFLOAT, VK_IMAGE_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT | VK_IMAGE_USAGE_STORAGE_BIT | VK_IMAGE_USAGE_SAMPLED_BIT, VmaMemoryUsage::VMA_MEMORY_USAGE_GPU_ONLY, VK_SAMPLE_COUNT_1_BIT, 1, 1, VK_IMAGE_TILING_OPTIMAL, 0, 0, nullptr);
	Image      colorImage(refDevice, extent, VK_FORMAT_R8G8B8A8_SRGB, VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_SAMPLED_BIT | VK_IMAGE_USAGE_TRANSFER_SRC_BIT, VmaMemoryUsage::VMA_MEMORY_USAGE_GPU_ONLY);

	// Create Render Target
	std::vector<Image> offScreenImages;
	offScreenImages.push_back(std::move(colorImage));
	offScreenImages.push_back(std::move(depthImage));
	std::shared_ptr<RenderTarget> renderTarget = std::make_shared<RenderTarget>(std::move(offScreenImages));

	DepthStencilState defaultDepthState;        // depth test/write
	ColorBlendState   defaultColorState;
	ColorBlendState   depthOnlyColorState;
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
		opaquePass.renderPass   = &refDevice.get_resource_cache().request_render_pass(renderTarget->get_attachments(), loadStoreInfos, subPassInfos);
		opaquePass.frameBuffer  = &refDevice.get_resource_cache().request_framebuffer(opaquePass.GetRenderTarget(), opaquePass.GetRenderPass());

		{
			// Dpeth Pass
			ShaderProgram *depthProgram = ShaderProgram::Find(std::string("forward_plus/depth_only"));
			PipelineLayout layout(refDevice, depthProgram->GetShaderModules());
			PipelineState  depthOnlyPipelineState;
			depthOnlyPipelineState.set_pipeline_layout(layout);
			depthOnlyPipelineState.set_render_pass(opaquePass.GetRenderPass());
			depthOnlyPipelineState.set_depth_stencil_state(defaultDepthState);
			depthOnlyPipelineState.set_color_blend_state(defaultColorState);
			opaquePass.pipelines.emplace(std::make_pair((uint32_t) DepthPrePassOrder, std::move(depthOnlyPipelineState)));
		}
	}
}

void forward_plus::prepare_resources()
{
	Device &refDevice = *device.get();

	{
		// Scene contain lighting/Texture(both image and sampler)/Material/Mesh/Camera
		load_scene("scenes/sponza/Sponza01.gltf");
	}

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
			auto &       shaderVS = refDevice.get_resource_cache().request_shader_module(VK_SHADER_STAGE_VERTEX_BIT, vsSource, emptyVariant);
			auto &       shaderFS = refDevice.get_resource_cache().request_shader_module(VK_SHADER_STAGE_FRAGMENT_BIT, fsSource, emptyVariant);

			std::initializer_list<ShaderModule *> initList{&shaderVS, &shaderFS};

			auto program = std::make_shared<ShaderProgram>(std::move(initList));
			ShaderProgram::AddShaderProgram(shaderSourceFiles[i], std::move(program));
		}

		for (int i = 0; i < computeSourceFiles.size(); i++)
		{
			ShaderSource source(computeSourceFiles[i] + ".comp");
			auto &       shaderCS = refDevice.get_resource_cache().request_shader_module(VK_SHADER_STAGE_COMPUTE_BIT, source, emptyVariant);
			auto         program  = std::make_shared<ShaderProgram>(&shaderCS);
			ShaderProgram::AddShaderProgram(computeSourceFiles[i], std::move(program));
		}
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
	std::multimap<float, std::pair<sg::Node *, sg::SubMesh *>> opaqueNodes;
	std::multimap<float, std::pair<sg::Node *, sg::SubMesh *>> transparentNodes;
	get_sorted_nodes(opaqueNodes, transparentNodes);

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
		// update and bind buffer
		bind_pipeline_state(commandBuffer, opaquePass.pipelines.at(DepthPrePassOrder));
		for (auto iter = opaqueNodes.begin(); iter != opaqueNodes.end(); iter++)
		{
			auto node    = iter->second.first;
			auto submesh = iter->second.second;
			update_global_uniform_buffers(commandBuffer, node);
			bind_descriptor(commandBuffer, submesh, opaquePass.pipelines.at(DepthPrePassOrder));
			if (bind_vertex_input(commandBuffer, submesh, opaquePass.pipelines.at(DepthPrePassOrder)))
			{
				commandBuffer.draw_indexed(submesh->vertex_indices, 1, 0, 0, 0);
			}
			else
			{
				commandBuffer.draw(submesh->vertices_count, 1, 0, 0);
			}
		}

		commandBuffer.end_render_pass();
	}

	blit_and_present(commandBuffer);
	commandBuffer.end();
	context.submit(commandBuffer);
}

void forward_plus::update_global_uniform_buffers(vkb::CommandBuffer &commandBuffer, vkb::sg::Node *node)
{
	auto &render_frame = get_render_context().get_active_frame();
	auto  allocation   = render_frame.allocate_buffer(VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT, sizeof(GlobalUniform), 0);
	auto &transform    = node->get_transform();

	GlobalUniform globalUniform;
	globalUniform.model           = transform.get_world_matrix();
	globalUniform.view_project    = vkb::vulkan_style_projection(camera.matrices.perspective) * camera.matrices.view;
	globalUniform.camera_position = camera.position;
	allocation.update(globalUniform);
	commandBuffer.bind_buffer(allocation.get_buffer(), allocation.get_offset(), allocation.get_size(), 0, 0, 0);
}

void forward_plus::bind_pipeline_state(vkb::CommandBuffer &commandBuffer, vkb::PipelineState &pipeline)
{
	commandBuffer.set_color_blend_state(pipeline.get_color_blend_state());
	commandBuffer.set_depth_stencil_state(pipeline.get_depth_stencil_state());
	commandBuffer.set_input_assembly_state(pipeline.get_input_assembly_state());
	commandBuffer.set_rasterization_state(pipeline.get_rasterization_state());
	commandBuffer.set_viewport_state(pipeline.get_viewport_state());
	commandBuffer.set_multisample_state(pipeline.get_multisample_state());
}

void forward_plus::bind_descriptor(vkb::CommandBuffer &commandBuffer, vkb::sg::SubMesh *submesh, vkb::PipelineState &pipeline, bool bindMaterial)
{
	const PipelineLayout &layout = pipeline.get_pipeline_layout();
	commandBuffer.bind_pipeline_layout(const_cast<PipelineLayout &>(layout));
	auto &constantState = pipeline.get_specialization_constant_state().get_specialization_constant_state();
	for (auto iter = constantState.begin(); iter != constantState.end(); ++iter)
	{
		commandBuffer.set_specialization_constant(iter->first, iter->second);
	}

	if (bindMaterial)
	{
		DescriptorSetLayout &descriptorSetLayout = layout.get_descriptor_set_layout(0);

		for (auto &texture : submesh->get_material()->textures)
		{
			auto layoutBinding = descriptorSetLayout.get_layout_binding(texture.first);
			if (layoutBinding != nullptr)
			{
				commandBuffer.bind_image(texture.second->get_image()->get_vk_image_view(),
				                         texture.second->get_sampler()->vk_sampler,
				                         0, layoutBinding->binding, 0);
			}
		}

		struct PBRMaterialUniform
		{
			glm::vec4 base_color_factor;

			float metallic_factor;

			float roughness_factor;
		};
		auto               pbr_material = dynamic_cast<const sg::PBRMaterial *>(submesh->get_material());
		PBRMaterialUniform pbr_material_uniform{};
		pbr_material_uniform.base_color_factor = pbr_material->base_color_factor;
		pbr_material_uniform.metallic_factor   = pbr_material->metallic_factor;
		pbr_material_uniform.roughness_factor  = pbr_material->roughness_factor;

		auto data = to_bytes(pbr_material_uniform);

		if (!data.empty())
		{
			commandBuffer.push_constants(data);
		}
	}
}

bool forward_plus::bind_vertex_input(vkb::CommandBuffer &commandBuffer, vkb::sg::SubMesh *submesh, vkb::PipelineState &pipeline)
{
	auto &pipelineLayout         = pipeline.get_pipeline_layout();
	auto  vertex_input_resources = pipelineLayout.get_resources(ShaderResourceType::Input, VK_SHADER_STAGE_VERTEX_BIT);

	VertexInputState vertex_input_state;

	for (auto &input_resource : vertex_input_resources)
	{
		sg::VertexAttribute attribute;

		if (!submesh->get_attribute(input_resource.name, attribute))
		{
			continue;
		}

		VkVertexInputAttributeDescription vertex_attribute{};
		vertex_attribute.binding  = input_resource.location;
		vertex_attribute.format   = attribute.format;
		vertex_attribute.location = input_resource.location;
		vertex_attribute.offset   = attribute.offset;

		vertex_input_state.attributes.push_back(vertex_attribute);

		VkVertexInputBindingDescription vertex_binding{};
		vertex_binding.binding = input_resource.location;
		vertex_binding.stride  = attribute.stride;

		vertex_input_state.bindings.push_back(vertex_binding);
	}

	commandBuffer.set_vertex_input_state(vertex_input_state);

	// Find submesh vertex buffers matching the shader input attribute names
	for (auto &input_resource : vertex_input_resources)
	{
		const auto &buffer_iter = submesh->vertex_buffers.find(input_resource.name);

		if (buffer_iter != submesh->vertex_buffers.end())
		{
			std::vector<std::reference_wrapper<const core::Buffer>> buffers;
			buffers.emplace_back(std::ref(buffer_iter->second));

			// Bind vertex buffers only for the attribute locations defined
			commandBuffer.bind_vertex_buffers(input_resource.location, std::move(buffers), {0});
		}
	}

	// Draw submesh indexed if indices exists
	if (submesh->vertex_indices != 0)
	{
		// Bind index buffer of submesh
		commandBuffer.bind_index_buffer(*submesh->index_buffer, submesh->index_offset, submesh->index_type);
		return true;
	}
	return false;
}

void forward_plus::blit_and_present(vkb::CommandBuffer &commandBuffer)
{
	auto &colorImageView = opaquePass.GetRenderTarget().get_views()[0];
	auto &swapChainView  = render_context->get_active_frame().get_render_target().get_views()[0];

	ImageMemoryBarrier barrier{};
	barrier.src_stage_mask  = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
	barrier.dst_stage_mask  = VK_PIPELINE_STAGE_TRANSFER_BIT;
	barrier.src_access_mask = VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT;
	barrier.dst_access_mask = VK_ACCESS_TRANSFER_READ_BIT;
	barrier.old_layout      = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
	barrier.new_layout      = VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL;
	commandBuffer.image_memory_barrier(colorImageView, barrier);

	barrier.src_stage_mask  = VK_PIPELINE_STAGE_BOTTOM_OF_PIPE_BIT;
	barrier.dst_stage_mask  = VK_PIPELINE_STAGE_TRANSFER_BIT;
	barrier.src_access_mask = 0;
	barrier.dst_access_mask = VK_ACCESS_TRANSFER_WRITE_BIT;
	barrier.old_layout      = VK_IMAGE_LAYOUT_UNDEFINED;
	barrier.new_layout      = VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL;
	commandBuffer.image_memory_barrier(swapChainView, barrier);

	VkExtent3D  colorImageExtent = colorImageView.get_image().get_extent();
	VkExtent3D  swapChainExtent  = swapChainView.get_image().get_extent();
	VkImageBlit blit{};
	blit.srcOffsets[0]             = VkOffset3D{0, 0, 0};
	blit.srcOffsets[1]             = VkOffset3D{(int32_t) colorImageExtent.width, (int32_t) colorImageExtent.height, 0};
	blit.dstOffsets[0]             = VkOffset3D{0, 0, 0};
	blit.dstOffsets[1]             = VkOffset3D{(int32_t) swapChainExtent.width, (int32_t) swapChainExtent.height, 0};
	blit.srcSubresource.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
	blit.srcSubresource.layerCount = 1;
	blit.dstSubresource.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
	blit.dstSubresource.layerCount = 1;

	commandBuffer.blit_image(colorImageView.get_image(), swapChainView.get_image(), {blit});

	barrier.src_stage_mask  = VK_PIPELINE_STAGE_TRANSFER_BIT;
	barrier.dst_stage_mask  = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
	barrier.src_access_mask = VK_ACCESS_TRANSFER_READ_BIT;
	barrier.dst_access_mask = VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT;
	barrier.old_layout      = VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL;
	barrier.new_layout      = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
	commandBuffer.image_memory_barrier(colorImageView, barrier);

	barrier.src_stage_mask  = VK_PIPELINE_STAGE_TRANSFER_BIT;
	barrier.dst_stage_mask  = VK_PIPELINE_STAGE_BOTTOM_OF_PIPE_BIT;
	barrier.src_access_mask = VK_ACCESS_TRANSFER_WRITE_BIT;
	barrier.dst_access_mask = 0;
	barrier.old_layout      = VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL;
	barrier.new_layout      = VK_IMAGE_LAYOUT_PRESENT_SRC_KHR;
	commandBuffer.image_memory_barrier(swapChainView, barrier);
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

std::unique_ptr<vkb::Application> create_forward_plus()
{
	return std::make_unique<forward_plus>();
}
