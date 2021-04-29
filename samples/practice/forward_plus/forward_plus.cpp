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

const RenderTarget::CreateFunc forward_plus::swap_chain_create_func = [](core::Image &&swapchain_image) -> std::unique_ptr<RenderTarget> {
	VkFormat                 depth_format = get_suitable_depth_format(swapchain_image.get_device().get_gpu().get_handle());
	std::vector<core::Image> images;
	images.push_back(std::move(swapchain_image));
	return std::make_unique<RenderTarget>(std::move(images));
};

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
	prepare_buffer();
	prepare_camera();
	prepare_pipelines();
	return true;
}

void forward_plus::prepare_render_context()
{
	// so we can just copy offscreen image to swap chain image
	auto &properties = render_context->get_swapchain().get_properties();
	properties.image_usage |= VK_IMAGE_USAGE_TRANSFER_DST_BIT;
	render_context->prepare(1, swap_chain_create_func);
}

const std::vector<const char *> forward_plus::get_validation_layers()
{
	return {"VK_LAYER_LUNARG_standard_validation", "VK_LAYER_NV_nsight"};
}

void forward_plus::update(float delta_time)
{
	update_scene(delta_time);
	//update_gui(delta_time);
	render(delta_time);
}

void forward_plus::get_sorted_nodes(std::multimap<float, std::pair<sg::Node *, sg::SubMesh *>> &opaque_nodes, std::multimap<float, std::pair<sg::Node *, sg::SubMesh *>> &transparent_nodes)
{
	auto camera_transform = camera->get_node()->get_component<vkb::sg::Transform>().get_translation();
	auto meshes           = scene->get_components<sg::Mesh>();
	for (auto &mesh : meshes)
	{
		for (auto &node : mesh->get_nodes())
		{
			auto node_transform = node->get_transform().get_world_matrix();

			const sg::AABB &mesh_bounds = mesh->get_bounds();

			sg::AABB world_bounds{mesh_bounds.get_min(), mesh_bounds.get_max()};
			world_bounds.transform(node_transform);

			float distance = glm::length(camera_transform - world_bounds.get_center());

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
	VkFormatFeatureFlags flags = gpu.get_format_properties(VK_FORMAT_R8G8B8A8_SRGB).optimalTilingFeatures;
	supportBlit                = (flags & VK_FORMAT_FEATURE_BLIT_SRC_BIT) != 0;
	supportBlit                = supportBlit && (flags & VK_FORMAT_FEATURE_BLIT_DST_BIT) != 0;
	assert(supportBlit);
}

void forward_plus::resize(const uint32_t width, const uint32_t height)
{
	VulkanSample::resize(width, height);
}

void forward_plus::prepare_buffer()
{
	Device &refDevice = *device.get();
	auto &  extent2d  = get_render_context().get_surface_extent();
	int     tileCount = glm::ceil(extent2d.height / 8.0) * glm::ceil(extent2d.width / 8.0);

	lightBuffer     = std::make_shared<Buffer>(refDevice, sizeof(LightBuffer) * MAX_LIGHTS_COUNT, VK_BUFFER_USAGE_STORAGE_BUFFER_BIT, VmaMemoryUsage::VMA_MEMORY_USAGE_CPU_TO_GPU);
	lightGridBuffer = std::make_shared<Buffer>(refDevice, sizeof(uint32_t) * (MAX_LIGHTS_COUNT + 4) * tileCount, VK_BUFFER_USAGE_STORAGE_BUFFER_BIT, VmaMemoryUsage::VMA_MEMORY_USAGE_GPU_ONLY);
	lightMaskBuffer = std::make_shared<Buffer>(refDevice, sizeof(uint32_t) * tileCount * 4, VK_BUFFER_USAGE_STORAGE_BUFFER_BIT, VmaMemoryUsage::VMA_MEMORY_USAGE_GPU_ONLY);
	postProcessVB   = std::make_shared<Buffer>(refDevice, sizeof(float) * 9, VK_BUFFER_USAGE_VERTEX_BUFFER_BIT, VmaMemoryUsage::VMA_MEMORY_USAGE_CPU_TO_GPU);

	float fullScreenTriangle[] = {-1.0, -1.0, 0.0, -1.0, 3.0, 0.0, 3.0, -1.0, 0.0};
	postProcessVB->update(&fullScreenTriangle, sizeof(float) * 9);
}

void forward_plus::prepare_pipelines()
{
	auto &   extent2d     = get_render_context().get_surface_extent();
	uint32_t windowWidth  = extent2d.width;
	uint32_t windowHeight = extent2d.height;

	Device &refDevice = *device.get();

	auto samplerCreateInfo         = initializers::sampler_create_info();
	samplerCreateInfo.magFilter    = VK_FILTER_LINEAR;
	samplerCreateInfo.minFilter    = VK_FILTER_LINEAR;
	samplerCreateInfo.mipmapMode   = VK_SAMPLER_MIPMAP_MODE_LINEAR;
	samplerCreateInfo.addressModeU = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
	samplerCreateInfo.addressModeV = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
	samplerCreateInfo.addressModeW = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
	samplerCreateInfo.minLod       = 0;
	samplerCreateInfo.maxLod       = VK_LOD_CLAMP_NONE;
	linearClampSampler             = std::make_shared<Sampler>(refDevice, samplerCreateInfo);

	// Create Render Image
	VkExtent3D extent{windowWidth, windowHeight, 1};
	Image      depthImage(refDevice, extent, VK_FORMAT_D32_SFLOAT, VK_IMAGE_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT | VK_IMAGE_USAGE_SAMPLED_BIT | VK_IMAGE_USAGE_STORAGE_BIT, VmaMemoryUsage::VMA_MEMORY_USAGE_GPU_ONLY);
	Image      colorImage(refDevice, extent, VK_FORMAT_R8G8B8A8_SRGB, VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_SAMPLED_BIT | VK_IMAGE_USAGE_TRANSFER_SRC_BIT, VmaMemoryUsage::VMA_MEMORY_USAGE_GPU_ONLY);
	linearDepthImage     = std::make_shared<Image>(refDevice, extent, VK_FORMAT_R32_SFLOAT, VK_IMAGE_USAGE_STORAGE_BIT | VK_IMAGE_USAGE_SAMPLED_BIT, VmaMemoryUsage::VMA_MEMORY_USAGE_GPU_ONLY);
	linearDepthImageView = std::make_shared<ImageView>(*linearDepthImage, VK_IMAGE_VIEW_TYPE_2D);

	std::vector<Image> offScreenFBO;
	offScreenFBO.emplace_back(std::move(colorImage));
	offScreenFBO.emplace_back(std::move(depthImage));
	auto offScreenRT = std::make_shared<RenderTarget>(std::move(offScreenFBO));

	DepthStencilState defaultDepthState;            // depth test/write on
	DepthStencilState postProcessDepthState;        // depth test/write off
	ColorBlendState   defaultColorState;
	ColorBlendState   depthOnlyColorState;
	{
		postProcessDepthState.depth_test_enable  = false;
		postProcessDepthState.depth_write_enable = false;
		postProcessDepthState.depth_compare_op   = VK_COMPARE_OP_ALWAYS;
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
		// Dpeth Pre Pass
		std::vector<LoadStoreInfo> loadStoreInfos;
		loadStoreInfos.emplace_back(LoadStoreInfo{VK_ATTACHMENT_LOAD_OP_DONT_CARE, VK_ATTACHMENT_STORE_OP_DONT_CARE});
		loadStoreInfos.emplace_back(LoadStoreInfo{VK_ATTACHMENT_LOAD_OP_CLEAR, VK_ATTACHMENT_STORE_OP_STORE});

		std::vector<SubpassInfo> subPassInfos;
		subPassInfos.emplace_back(SubpassInfo{{}, {}, {}, false, 0, VK_RESOLVE_MODE_NONE});

		depthPrePass.renderTarget = offScreenRT;
		depthPrePass.PrebuildPass(refDevice, loadStoreInfos, subPassInfos);

		PipelineState *depthOnlyPipelineState = depthPrePass.PushPipeline(refDevice, Opaque, std::string("depth_only"));
		depthOnlyPipelineState->set_depth_stencil_state(defaultDepthState);
		depthOnlyPipelineState->set_color_blend_state(depthOnlyColorState);
	}

	{
		// Linear Depth
		ShaderProgram *linearDepthProgram = ShaderProgram::Find(std::string("linear_depth"));
		linearDepthPass.pipelineLayout    = &refDevice.get_resource_cache().request_pipeline_layout(linearDepthProgram->GetShaderModules());
		PipelineState pipelineState;
		pipelineState.set_pipeline_layout(*linearDepthPass.pipelineLayout);
		refDevice.get_resource_cache().request_compute_pipeline(pipelineState);
#if 0 
		ShaderProgram * lightGridProgram = ShaderProgram::Find(std::string("light_grid"));
		PipelineLayout &lightGridLayout  = refDevice.get_resource_cache().request_pipeline_layout(lightGridProgram->GetShaderModules());
		PipelineState   lightGridState;
		lightGridState.set_pipeline_layout(lightGridLayout);
		refDevice.get_resource_cache().request_compute_pipeline(lightGridState);
#endif
	}

	{
		// Debug Depth
		std::vector<LoadStoreInfo> loadStoreInfos;
		loadStoreInfos.emplace_back(LoadStoreInfo{VK_ATTACHMENT_LOAD_OP_CLEAR, VK_ATTACHMENT_STORE_OP_STORE});
		loadStoreInfos.emplace_back(LoadStoreInfo{VK_ATTACHMENT_LOAD_OP_DONT_CARE, VK_ATTACHMENT_STORE_OP_DONT_CARE});
		std::vector<SubpassInfo> subPassInfos;
		subPassInfos.emplace_back(SubpassInfo{{}, {0}, {}, false, 0, VK_RESOLVE_MODE_NONE});

		debugDepthPass.renderTarget = offScreenRT;
		debugDepthPass.PrebuildPass(refDevice, loadStoreInfos, subPassInfos);

		PipelineState *debugDepthState = debugDepthPass.PushPipeline(refDevice, PostProcess, std::string("debug_depth"));
		debugDepthState->set_depth_stencil_state(postProcessDepthState);
		debugDepthState->set_color_blend_state(defaultColorState);
		VertexInputState vertexInputState;
		vertexInputState.bindings.emplace_back(VkVertexInputBindingDescription{0, sizeof(float) * 3, VK_VERTEX_INPUT_RATE_VERTEX});
		vertexInputState.attributes.emplace_back(VkVertexInputAttributeDescription{0, 0, VK_FORMAT_R32G32B32_SFLOAT, 0});
		debugDepthState->set_vertex_input_state(vertexInputState);
	}

	// opaque render
	{
		std::vector<LoadStoreInfo> loadStoreInfos;
		loadStoreInfos.emplace_back(LoadStoreInfo{VK_ATTACHMENT_LOAD_OP_CLEAR, VK_ATTACHMENT_STORE_OP_STORE});
		loadStoreInfos.emplace_back(LoadStoreInfo{VK_ATTACHMENT_LOAD_OP_LOAD, VK_ATTACHMENT_STORE_OP_DONT_CARE});

		std::vector<SubpassInfo> subPassInfos;
		subPassInfos.emplace_back(SubpassInfo{{}, {0}, {}, false, 0, VK_RESOLVE_MODE_NONE});

		opaquePass.renderTarget = offScreenRT;
		opaquePass.PrebuildPass(refDevice, loadStoreInfos, subPassInfos);

		//PipelineState*   opaqueRenderState = opaquePass.PushPipeline(refDevice, Opaque, std::string("pbr_base"));
		//opaqueRenderState->set_depth_stencil_state(defaultDepthState);
		//opaqueRenderState->set_color_blend_state(defaultColorState);
	}
}

void forward_plus::prepare_resources()
{
	Device &refDevice = *device.get();
	{
		// Load Shader
		struct RProgramSources
		{
			std::string programName;
			std::string vertexName;
			std::string fragmentName;
		};

		struct CProgramSources
		{
			std::string programName;
			std::string computeName;
		};

		ShaderVariant                emptyVariant{};
		std::vector<RProgramSources> shaderSourceFiles{
		    {"depth_only", "forward_plus/depth_only.vert", "forward_plus/depth_only.frag"},
		    {"debug_depth", "forward_plus/screen_base.vert", "forward_plus/debug_depth.frag"},
		};
		std::vector<CProgramSources> computeSourceFiles{
		    {"linear_depth", "forward_plus/linear_depth.comp"},
		    {"light_grid", "forward_plus/light_grid.comp"},
		};

		std::unordered_map<std::string, ShaderSource> shaderSources;
		for (int i = 0; i < shaderSourceFiles.size(); ++i)
		{
			if (shaderSources.find(shaderSourceFiles[i].vertexName) == shaderSources.end())
			{
				shaderSources.emplace(std::make_pair(shaderSourceFiles[i].vertexName, std::move(ShaderSource(shaderSourceFiles[i].vertexName))));
			}

			if (shaderSources.find(shaderSourceFiles[i].fragmentName) == shaderSources.end())
			{
				shaderSources.emplace(std::make_pair(shaderSourceFiles[i].fragmentName, std::move(ShaderSource(shaderSourceFiles[i].fragmentName))));
			}

			auto &vsSource = shaderSources[shaderSourceFiles[i].vertexName];
			auto &fsSource = shaderSources[shaderSourceFiles[i].fragmentName];
			auto &shaderVS = refDevice.get_resource_cache().request_shader_module(VK_SHADER_STAGE_VERTEX_BIT, vsSource, emptyVariant);
			auto &shaderFS = refDevice.get_resource_cache().request_shader_module(VK_SHADER_STAGE_FRAGMENT_BIT, fsSource, emptyVariant);

			std::initializer_list<ShaderModule *> initList{&shaderVS, &shaderFS};

			auto program = std::make_shared<ShaderProgram>(std::move(initList));
			ShaderProgram::AddShaderProgram(shaderSourceFiles[i].programName, std::move(program));
		}

		for (int i = 0; i < computeSourceFiles.size(); i++)
		{
			ShaderSource source(computeSourceFiles[i].computeName);
			auto &       shaderCS = refDevice.get_resource_cache().request_shader_module(VK_SHADER_STAGE_COMPUTE_BIT, source, emptyVariant);
			auto         program  = std::make_shared<ShaderProgram>(&shaderCS);
			ShaderProgram::AddShaderProgram(computeSourceFiles[i].programName, std::move(program));
			shaderSources.emplace(std::make_pair(computeSourceFiles[i].computeName, std::move(source)));
		}
	}

	{
		// Scene contain lighting/Texture(both image and sampler)/Material/Mesh/Camera
		load_scene("scenes/sponza/Sponza01.gltf");
	}
}

void forward_plus::prepare_camera()
{
	auto &camera_node = vkb::add_free_camera(*scene, "main_camera", get_render_context().get_surface_extent());

	camera = &camera_node.get_component<vkb::sg::Camera>();
	camera->set_far_plane(10000.0f);
}

void forward_plus::render(float delta_time)
{
	std::multimap<float, std::pair<sg::Node *, sg::SubMesh *>> opaqueNodes;
	std::multimap<float, std::pair<sg::Node *, sg::SubMesh *>> transparentNodes;
	get_sorted_nodes(opaqueNodes, transparentNodes);

	// reverse z
	std::vector<VkClearValue> clearValue{initializers::clear_color_value(1.0, 0.0, 0.0, 0.0), initializers::clear_depth_stencil_value(0.0, 0)};
	VkExtent2D                extent  = get_render_context().get_surface_extent();
	RenderContext &           context = get_render_context();

	CommandBuffer &commandBuffer = context.begin();
	commandBuffer.begin(VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT);
	set_viewport_and_scissor(commandBuffer, extent);

	// depth pre pass
	{
		commandBuffer.begin_render_pass(depthPrePass.GetRenderTarget(), depthPrePass.GetRenderPass(), depthPrePass.GetFrameBuffer(), clearValue);
		// update and bind buffer
		bind_pipeline_state(commandBuffer, depthPrePass.pipelines.at(Opaque));
		for (auto iter = opaqueNodes.begin(); iter != opaqueNodes.end(); iter++)
		{
			auto node    = iter->second.first;
			auto submesh = iter->second.second;
			update_global_uniform_buffers(commandBuffer, node);
			bind_descriptor(commandBuffer, depthPrePass.pipelines.at(Opaque), submesh);
			if (bind_vertex_input(commandBuffer, depthPrePass.pipelines.at(Opaque), submesh))
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

	const ImageView &depthView = depthPrePass.GetRenderTarget().get_views()[1];
	{
		// linear depth
		vkb::ImageMemoryBarrier barrier{};
		barrier.src_stage_mask  = VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT;
		barrier.dst_stage_mask  = VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT;
		barrier.dst_access_mask = VK_ACCESS_SHADER_WRITE_BIT;
		barrier.new_layout      = VK_IMAGE_LAYOUT_GENERAL;
		commandBuffer.image_memory_barrier(*linearDepthImageView, barrier);

		barrier.src_stage_mask  = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
		barrier.dst_stage_mask  = VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT;
		barrier.src_access_mask = 0;
		barrier.dst_access_mask = VK_ACCESS_SHADER_READ_BIT;
		barrier.old_layout      = VK_IMAGE_LAYOUT_UNDEFINED;
		barrier.new_layout      = VK_IMAGE_LAYOUT_DEPTH_STENCIL_READ_ONLY_OPTIMAL;
		commandBuffer.image_memory_barrier(depthView, barrier);

		commandBuffer.bind_pipeline_layout(*linearDepthPass.pipelineLayout);
		const auto &descLayout = linearDepthPass.pipelineLayout->get_descriptor_set_layout(0);
		commandBuffer.bind_input(depthView, 0, 0, 0);
		commandBuffer.bind_input(*linearDepthImageView, 0, 1, 0);

		VkExtent3D extent = linearDepthImage->get_extent();
		struct
		{
			float    nearPlane;
			float    farPlane;
			uint32_t width;
			uint32_t height;
		} uniforms{camera->get_near_plane(), camera->get_far_plane(), extent.width, extent.height};

		BufferAllocation allocation = get_render_context().get_active_frame().allocate_buffer(VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT, sizeof(uniforms));
		allocation.update(uniforms);
		commandBuffer.bind_buffer(allocation.get_buffer(), allocation.get_offset(), allocation.get_offset(), 0, 2, 0);
		commandBuffer.dispatch((uint32_t) glm::ceil(extent.width / 16.0f), (uint32_t) glm::ceil(extent.height / 16.0f), 1);
	}

	{
		//
		ImageMemoryBarrier barrier{};
		barrier.src_stage_mask  = VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT;
		barrier.dst_stage_mask  = VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT;
		barrier.src_access_mask = VK_ACCESS_SHADER_WRITE_BIT;
		barrier.dst_access_mask = VK_ACCESS_SHADER_READ_BIT;
		barrier.old_layout      = VK_IMAGE_LAYOUT_GENERAL;
		barrier.new_layout      = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
		commandBuffer.image_memory_barrier(*linearDepthImageView, barrier);

		barrier.dst_stage_mask  = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
		barrier.src_access_mask = VK_ACCESS_SHADER_READ_BIT;
		barrier.dst_access_mask = VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT;
		barrier.old_layout      = VK_IMAGE_LAYOUT_DEPTH_STENCIL_READ_ONLY_OPTIMAL;
		barrier.new_layout      = VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL;
		commandBuffer.image_memory_barrier(depthView, barrier);
	}

	{
		// show linear depth pass
		commandBuffer.begin_render_pass(debugDepthPass.GetRenderTarget(), debugDepthPass.GetRenderPass(), debugDepthPass.GetFrameBuffer(), clearValue);
		bind_pipeline_state(commandBuffer, debugDepthPass.pipelines[PostProcess]);
		bind_descriptor(commandBuffer, debugDepthPass.pipelines[PostProcess]);
		commandBuffer.bind_image(*linearDepthImageView, *linearClampSampler, 0, 0, 0);
		commandBuffer.set_vertex_input_state(debugDepthPass.pipelines[PostProcess].get_vertex_input_state());
		commandBuffer.bind_vertex_buffers(0, {*postProcessVB}, {0});
		commandBuffer.draw(3, 1, 0, 0);
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
	globalUniform.view_project    = vkb::vulkan_style_projection(camera->get_projection()) * camera->get_view();
	globalUniform.camera_position = camera->get_node()->get_component<vkb::sg::Transform>().get_translation();
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

void forward_plus::bind_descriptor(vkb::CommandBuffer &commandBuffer, vkb::PipelineState &pipeline, vkb::sg::SubMesh *submesh, bool bindMaterial)
{
	const PipelineLayout &layout = pipeline.get_pipeline_layout();
	commandBuffer.bind_pipeline_layout(const_cast<PipelineLayout &>(layout));
	auto &constantState = pipeline.get_specialization_constant_state().get_specialization_constant_state();
	for (auto iter = constantState.begin(); iter != constantState.end(); ++iter)
	{
		commandBuffer.set_specialization_constant(iter->first, iter->second);
	}

	if (bindMaterial && submesh)
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

bool forward_plus::bind_vertex_input(vkb::CommandBuffer &commandBuffer, vkb::PipelineState &pipeline, vkb::sg::SubMesh *submesh)
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
}

std::unique_ptr<vkb::Application> create_forward_plus()
{
	return std::make_unique<forward_plus>();
}
