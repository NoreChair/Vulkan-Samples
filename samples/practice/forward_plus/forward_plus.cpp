#include "forward_plus.h"
#include "Utils.h"
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

std::unordered_map<size_t, ShaderSource> forward_plus::ShaderProgram::shaderSourcePool = std::unordered_map<size_t, ShaderSource>();

const RenderTarget::CreateFunc forward_plus::swap_chain_create_func = [](core::Image &&swapchain_image) -> std::unique_ptr<RenderTarget> {
	VkFormat                 depth_format = get_suitable_depth_format(swapchain_image.get_device().get_gpu().get_handle());
	std::vector<core::Image> images;
	images.push_back(std::move(swapchain_image));
	return std::make_unique<RenderTarget>(std::move(images));
};

forward_plus::forward_plus()
{
	set_name(k_name);
	//debugDepth = true;
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
	prepare_buffer();
	prepare_light();
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
	return {"VK_LAYER_LUNARG_standard_validation"};
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
	int     tileCount = (int) (glm::ceil(extent2d.height / 16.0f) * glm::ceil(extent2d.width / 16.0f));

	lightBuffer     = std::make_shared<Buffer>(refDevice, sizeof(LightBuffer) * MAX_LIGHTS_COUNT, VK_BUFFER_USAGE_STORAGE_BUFFER_BIT, VmaMemoryUsage::VMA_MEMORY_USAGE_CPU_TO_GPU);
	lightGridBuffer = std::make_shared<Buffer>(refDevice, sizeof(uint32_t) * (MAX_LIGHTS_COUNT + 4) * tileCount, VK_BUFFER_USAGE_STORAGE_BUFFER_BIT, VmaMemoryUsage::VMA_MEMORY_USAGE_GPU_ONLY);
	lightMaskBuffer = std::make_shared<Buffer>(refDevice, sizeof(uint32_t) * tileCount * 4, VK_BUFFER_USAGE_STORAGE_BUFFER_BIT, VmaMemoryUsage::VMA_MEMORY_USAGE_GPU_ONLY);
	postProcessVB   = std::make_shared<Buffer>(refDevice, sizeof(float) * 9, VK_BUFFER_USAGE_VERTEX_BUFFER_BIT, VmaMemoryUsage::VMA_MEMORY_USAGE_CPU_TO_GPU);

	float fullScreenTriangle[] = {-1.0, -1.0, 0.0, -1.0, 3.0, 0.0, 3.0, -1.0, 0.0};
	postProcessVB->update(&fullScreenTriangle, sizeof(float) * 9);
}

void forward_plus::prepare_light()
{
	auto &    sceneAABB = scene->get_root_node().get_component<sg::AABB>();
	glm::vec3 posScale  = sceneAABB.get_max() - sceneAABB.get_min();
	glm::vec3 posBias   = sceneAABB.get_min();

	// todo: replace this with MT
	srand(12645);
	auto randUint = []() -> uint32_t {
		return rand();        // [0, RAND_MAX]
	};
	auto randFloat = [randUint]() -> float {
		return randUint() * (1.0f / RAND_MAX);        // convert [0, RAND_MAX] to [0, 1]
	};
	auto randVecUniform = [randFloat]() -> glm::vec3 {
		return glm::vec3(randFloat(), randFloat(), randFloat());
	};
	auto randGaussian = [randFloat]() -> float {
		// polar box-muller
		static bool  gaussianPair = true;
		static float y2;

		if (gaussianPair)
		{
			gaussianPair = false;

			float x1, x2, w;
			do
			{
				x1 = 2 * randFloat() - 1;
				x2 = 2 * randFloat() - 1;
				w  = x1 * x1 + x2 * x2;
			} while (w >= 1);

			w  = sqrt(-2 * log(w) / w);
			y2 = x2 * w;
			return x1 * w;
		}
		else
		{
			gaussianPair = true;
			return y2;
		}
	};
	auto randVecGaussian = [randGaussian]() -> glm::vec3 {
		return glm::normalize(glm::vec3(randGaussian(), randGaussian(), randGaussian()));
	};

	const float              pi = 3.14159265359f;
	std::vector<LightBuffer> lightData(MAX_LIGHTS_COUNT);
	std::vector<glm::vec3>   centers(MAX_LIGHTS_COUNT);
	std::vector<float>       radius(MAX_LIGHTS_COUNT);

	for (uint32_t n = 0; n < MAX_LIGHTS_COUNT; n++)
	{
		glm::vec3 pos         = randVecUniform() * posScale + posBias;
		float     lightRadius = randFloat() * 200.0f + 50.0f;

		glm::vec3 color      = randVecUniform();
		float     colorScale = randFloat() * .3f + .3f;
		color                = color * colorScale;

		uint32_t type = n < 32 * 2 ? 0 : 1;

		glm::vec3 coneDir   = randVecGaussian();
		float     coneInner = (randFloat() * .2f + .025f) * pi;
		float     coneOuter = coneInner + randFloat() * .1f * pi;

		lightData[n].position     = pos;
		lightData[n].color        = color;
		lightData[n].coneDir      = coneDir;
		lightData[n].coneAngles.x = 1.0f / (cos(coneInner) - cos(coneOuter));
		lightData[n].coneAngles.y = cos(coneOuter);
		lightData[n].coneAngles.z = cos(coneOuter * 0.5f);
		lightData[n].radius       = lightRadius;
		lightData[n].intensity    = type == 1 ? 5.0f : 1.0f;
		lightData[n].lightType    = type;
		lightData[n].padding      = 0;

		centers[n] = pos;
		radius[n]  = lightRadius;
	}

	lightBuffer->update(&lightData[0], MAX_LIGHTS_COUNT * sizeof(LightBuffer));
	debugDrawPass->add_bounding_sphere(std::move(centers), std::move(radius));
}

void forward_plus::prepare_pipelines()
{
	Device & refDevice    = *device.get();
	auto &   extent2d     = get_render_context().get_surface_extent();
	uint32_t windowWidth  = extent2d.width;
	uint32_t windowHeight = extent2d.height;

	VkExtent3D extent{windowWidth, windowHeight, 1};

	// Create Render Image
	Image depthImage(refDevice, extent, VK_FORMAT_D32_SFLOAT, VK_IMAGE_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT | VK_IMAGE_USAGE_SAMPLED_BIT | VK_IMAGE_USAGE_STORAGE_BIT, VmaMemoryUsage::VMA_MEMORY_USAGE_GPU_ONLY);
	Image colorImage(refDevice, extent, VK_FORMAT_R8G8B8A8_SRGB, VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_SAMPLED_BIT | VK_IMAGE_USAGE_TRANSFER_SRC_BIT, VmaMemoryUsage::VMA_MEMORY_USAGE_GPU_ONLY);

	linearDepthImage     = std::make_shared<Image>(refDevice, extent, VK_FORMAT_R32_SFLOAT, VK_IMAGE_USAGE_STORAGE_BIT | VK_IMAGE_USAGE_SAMPLED_BIT, VmaMemoryUsage::VMA_MEMORY_USAGE_GPU_ONLY);
	linearDepthImageView = std::make_shared<ImageView>(*linearDepthImage, VK_IMAGE_VIEW_TYPE_2D);

	std::vector<Image> offScreenImgs;
	offScreenImgs.emplace_back(std::move(colorImage));
	offScreenImgs.emplace_back(std::move(depthImage));
	offScreenRT = std::make_shared<RenderTarget>(std::move(offScreenImgs));

	{
		ShaderSource vs = ShaderProgram::FindShaderSource(std::string("forward_plus/depth_only.vert"));
		ShaderSource fs = ShaderProgram::FindShaderSource(std::string("forward_plus/depth_only.frag"));
		depthPrePass    = std::make_unique<depth_only_pass>(*render_context, std::move(vs), std::move(fs));
		depthPrePass->prepare(camera, offScreenRT.get());
	}

	{
		ShaderProgram *linearDepthProgram = ShaderProgram::Find(std::string("linear_depth"));
		linearDepthPass.pipelineLayout    = &refDevice.get_resource_cache().request_pipeline_layout(linearDepthProgram->GetShaderModules());
		PipelineState pipelineState;
		pipelineState.set_pipeline_layout(*linearDepthPass.pipelineLayout);
		refDevice.get_resource_cache().request_compute_pipeline(pipelineState);

		ShaderProgram *lightGridProgram = ShaderProgram::Find(std::string("light_grid"));
		lightGridPass.pipelineLayout    = &refDevice.get_resource_cache().request_pipeline_layout(lightGridProgram->GetShaderModules());
		PipelineState lightGridState;
		lightGridState.set_pipeline_layout(*lightGridPass.pipelineLayout);
		refDevice.get_resource_cache().request_compute_pipeline(lightGridState);
	}

	{
		ShaderSource vs = ShaderProgram::FindShaderSource(std::string("forward_plus/screen_base.vert"));
		ShaderSource fs = ShaderProgram::FindShaderSource(std::string("forward_plus/debug_depth.frag"));
		showDepthPass   = std::make_unique<show_depth_pass>(*render_context, std::move(vs), std::move(fs));
		showDepthPass->prepare(offScreenRT.get());
	}

	{
		ShaderSource vs = ShaderProgram::FindShaderSource(std::string("forward_plus/debug_draw.vert"));
		ShaderSource fs = ShaderProgram::FindShaderSource(std::string("forward_plus/debug_draw.frag"));
		debugDrawPass   = std::make_unique<debug_draw_pass>(*render_context, std::move(vs), std::move(fs));
		debugDrawPass->prepare(camera, offScreenRT.get());
	}

	{
		ShaderSource vs = ShaderProgram::FindShaderSource(std::string("forward_plus/pbr_plus.vert"));
		ShaderSource fs = ShaderProgram::FindShaderSource(std::string("forward_plus/pbr_plus.frag"));
		opaquePass      = std::make_unique<opaque_pass>(*render_context, std::move(vs), std::move(fs));
		opaquePass->prepare(camera, offScreenRT.get());
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

		std::vector<RProgramSources> shaderSourceFiles{
		    {"depth_only", "forward_plus/depth_only.vert", "forward_plus/depth_only.frag"},
		    {"pbr_plus", "forward_plus/pbr_plus.vert", "forward_plus/pbr_plus.frag"},
		    {"debug_depth", "forward_plus/screen_base.vert", "forward_plus/debug_depth.frag"},
		    {"debug_draw", "forward_plus/debug_draw.vert", "forward_plus/debug_draw.frag"},
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
			auto &shaderVS = refDevice.get_resource_cache().request_shader_module(VK_SHADER_STAGE_VERTEX_BIT, vsSource);
			auto &shaderFS = refDevice.get_resource_cache().request_shader_module(VK_SHADER_STAGE_FRAGMENT_BIT, fsSource);

			std::initializer_list<ShaderModule *> initList{&shaderVS, &shaderFS};

			auto program = std::make_shared<ShaderProgram>(std::move(initList));
			ShaderProgram::AddShaderProgram(shaderSourceFiles[i].programName, std::move(program));
		}

		for (int i = 0; i < computeSourceFiles.size(); i++)
		{
			ShaderSource source(computeSourceFiles[i].computeName);
			auto &       shaderCS = refDevice.get_resource_cache().request_shader_module(VK_SHADER_STAGE_COMPUTE_BIT, source);
			auto         program  = std::make_shared<ShaderProgram>(&shaderCS);
			ShaderProgram::AddShaderProgram(computeSourceFiles[i].programName, std::move(program));
			shaderSources.emplace(std::make_pair(computeSourceFiles[i].computeName, std::move(source)));
		}

		for (auto iter = shaderSources.begin(); iter != shaderSources.end(); iter++)
		{
			ShaderProgram::AddShaderSource(std::move(iter->second));
		}
	}

	{
		// Scene contain lighting/Texture(both image and sampler)/Material/Mesh/Camera
		load_scene("scenes/sponza/Sponza01.gltf");

		if (!scene->get_root_node().has_component<sg::AABB>())
		{
			auto  sceneAABB = std::make_unique<sg::AABB>();
			auto &meshs     = scene->get_components<sg::Mesh>();

			for (size_t i = 0; i < meshs.size(); i++)
			{
				sceneAABB->update(meshs[i]->get_bounds().get_min());
				sceneAABB->update(meshs[i]->get_bounds().get_max());
			}
			scene->add_component(std::move(sceneAABB), scene->get_root_node());
		}

		{
			GLTFLoader loader{*device};
			sphere_mesh = loader.read_simple_model_from_file("scenes/unit_sphere.gltf", 0);
			cube_mesh   = loader.read_simple_model_from_file("scenes/unit_cube.gltf", 0);
		}
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
	std::vector<VkClearValue> clearValue{initializers::clear_color_value(0.0, 0.0, 0.0, 0.0), initializers::clear_depth_stencil_value(0.0, 0)};
	VkExtent2D                extent  = get_render_context().get_surface_extent();
	RenderContext &           context = get_render_context();

	CommandBuffer &commandBuffer = context.begin();
	commandBuffer.begin(VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT);
	set_viewport_and_scissor(commandBuffer, extent);

	// depth pre pass
	{
		depthPrePass->draw(commandBuffer, opaqueNodes);
	}

	const ImageView &depthView = offScreenRT->get_views()[1];
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

		PipelineState defaultState;
		bind_pipeline_state(commandBuffer, defaultState);
		commandBuffer.bind_pipeline_layout(*linearDepthPass.pipelineLayout);
		commandBuffer.bind_input(depthView, 0, 0, 0);
		commandBuffer.bind_input(*linearDepthImageView, 0, 1, 0);

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
		vkb::ImageMemoryBarrier barrier{};
		barrier.src_stage_mask  = VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT;
		barrier.dst_stage_mask  = VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT;
		barrier.src_access_mask = VK_ACCESS_SHADER_WRITE_BIT;
		barrier.dst_access_mask = VK_ACCESS_SHADER_READ_BIT;
		barrier.new_layout      = VK_IMAGE_LAYOUT_GENERAL;
		commandBuffer.image_memory_barrier(*linearDepthImageView, barrier);

		BufferMemoryBarrier bufferBarrier{};
		bufferBarrier.src_stage_mask  = VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT;
		bufferBarrier.dst_stage_mask  = VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT;
		bufferBarrier.src_access_mask = VK_ACCESS_SHADER_READ_BIT;
		bufferBarrier.dst_access_mask = VK_ACCESS_SHADER_WRITE_BIT;

		commandBuffer.buffer_memory_barrier(*lightGridBuffer, 0, lightBuffer->get_size(), bufferBarrier);
		commandBuffer.buffer_memory_barrier(*lightMaskBuffer, 0, lightMaskBuffer->get_size(), bufferBarrier);

		VkExtent2D    dispatchCount = {(uint32_t) glm::ceil(extent.width / 16.0f), (uint32_t) glm::ceil(extent.height / 16.0f)};
		PipelineState defaultState;
		bind_pipeline_state(commandBuffer, defaultState);
		commandBuffer.bind_pipeline_layout(*lightGridPass.pipelineLayout);

		struct
		{
			glm::mat4  viewMatrix;
			glm::mat4  projMatrix;
			VkExtent2D viewport;
			uint32_t   tileCountX;
			uint32_t   lightBufferCount;
			float      invTileDim;
		} uniforms{
		    camera->get_view(),
		    vkb::vulkan_style_projection(camera->get_projection()),
		    extent,
		    dispatchCount.width,
		    MAX_LIGHTS_COUNT,
		    1.0f / 16.0f};
		BufferAllocation allocation = get_render_context().get_active_frame().allocate_buffer(VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT, sizeof(uniforms));
		allocation.update(uniforms);
		commandBuffer.bind_buffer(allocation.get_buffer(), allocation.get_offset(), allocation.get_offset(), 0, 0, 0);
		commandBuffer.bind_buffer(*lightBuffer, 0, lightBuffer->get_size(), 0, 1, 0);
		commandBuffer.bind_buffer(*lightGridBuffer, 0, lightGridBuffer->get_size(), 0, 2, 0);
		commandBuffer.bind_buffer(*lightMaskBuffer, 0, lightMaskBuffer->get_size(), 0, 3, 0);
		commandBuffer.bind_image(*linearDepthImageView, 0, 4, 0);
		commandBuffer.dispatch(dispatchCount.width, dispatchCount.height, 1);

		bufferBarrier.src_stage_mask  = VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT;
		bufferBarrier.dst_stage_mask  = VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT;
		bufferBarrier.src_access_mask = VK_ACCESS_SHADER_WRITE_BIT;
		bufferBarrier.dst_access_mask = VK_ACCESS_SHADER_READ_BIT;

		commandBuffer.buffer_memory_barrier(*lightGridBuffer, 0, lightBuffer->get_size(), bufferBarrier);
		commandBuffer.buffer_memory_barrier(*lightMaskBuffer, 0, lightMaskBuffer->get_size(), bufferBarrier);
	}

	{
		ImageMemoryBarrier barrier{};
		barrier.src_stage_mask  = VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT;
		barrier.dst_stage_mask  = VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT;
		barrier.src_access_mask = VK_ACCESS_SHADER_READ_BIT;
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
	//debugDepth = true;
	if (debugDepth)
	{
		// show linear depth pass
		showDepthPass->set_up(postProcessVB.get(), linearDepthImageView.get());
		showDepthPass->draw(commandBuffer);
	}
	else
	{
		opaquePass->set_up(lightGridBuffer.get(), lightBuffer.get());
		opaquePass->draw(commandBuffer, opaqueNodes);

		debugDrawPass->set_up(sphere_mesh.get(), cube_mesh.get());
		debugDrawPass->draw(commandBuffer);
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

bool forward_plus::bind_vertex_input(vkb::CommandBuffer &commandBuffer, vkb::PipelineLayout &pipelineLayout, vkb::sg::SubMesh *submesh)
{
	auto vertex_input_resources = pipelineLayout.get_resources(ShaderResourceType::Input, VK_SHADER_STAGE_VERTEX_BIT);

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
	auto &colorImageView = offScreenRT->get_views()[0];
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

void forward_plus::input_event(const vkb::InputEvent &input_event)
{
	VulkanSample::input_event(input_event);
}

std::unique_ptr<vkb::Application> create_forward_plus()
{
	return std::make_unique<forward_plus>();
}
