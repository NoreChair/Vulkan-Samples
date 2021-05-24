#include "forward_plus.h"
#include "ShaderProgram.h"
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

const RenderTarget::CreateFunc forward_plus::swap_chain_create_func = [](core::Image &&swapchain_image) -> std::unique_ptr<RenderTarget> {
	VkFormat                 depth_format = get_suitable_depth_format(swapchain_image.get_device().get_gpu().get_handle());
	std::vector<core::Image> images;
	images.push_back(std::move(swapchain_image));
	return std::make_unique<RenderTarget>(std::move(images));
};

forward_plus::forward_plus()
{
	set_name(k_name);
	auto &config = get_configuration();

	config.insert<vkb::BoolSetting>(0, debugDepth, false);
	config.insert<vkb::BoolSetting>(0, drawAABB, false);
	config.insert<vkb::BoolSetting>(0, drawLight, false);
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

	prepare_shaders();
	prepare_pipelines();
	prepare_buffer();
	prepare_scene();
	prepare_light();

	//stats->request_stats({ vkb::StatIndex::cpu_cycles,vkb::StatIndex::gpu_cycles });
	gui = std::make_unique<vkb::Gui>(*this, platform.get_window(), stats.get());

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
	//return {"VK_LAYER_LUNARG_standard_validation"};
	return {};
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
	//lightMaskBuffer = std::make_shared<Buffer>(refDevice, sizeof(uint32_t) * tileCount * 4, VK_BUFFER_USAGE_STORAGE_BUFFER_BIT, VmaMemoryUsage::VMA_MEMORY_USAGE_GPU_ONLY);
	postProcessVB = std::make_shared<Buffer>(refDevice, sizeof(float) * 9, VK_BUFFER_USAGE_VERTEX_BUFFER_BIT, VmaMemoryUsage::VMA_MEMORY_USAGE_CPU_TO_GPU);

	float fullScreenTriangle[] = {-1.0, -1.0, 0.0, -1.0, 3.0, 0.0, 3.0, -1.0, 0.0};
	postProcessVB->update(&fullScreenTriangle, sizeof(float) * 9);
}

void forward_plus::prepare_light()
{
	auto &    sceneAABB = scene->get_root_node().get_component<sg::AABB>();
	glm::vec3 posScale  = sceneAABB.get_max() - sceneAABB.get_min();
	glm::vec3 posBias   = sceneAABB.get_min();
	posBias.y += 1200;
	posScale.y -= 1400;

	// todo: replace this with MT
	srand(57495);
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
	std::vector<glm::vec3>   colors(MAX_LIGHTS_COUNT);

	for (uint32_t n = 0; n < MAX_LIGHTS_COUNT; n++)
	{
		glm::vec3 pos         = randVecUniform() * posScale + posBias;
		float     lightRadius = randFloat() * 400.0f + 200.0f;

		glm::vec3 color      = randVecUniform();
		float     colorScale = randFloat() * .3f + .3f;
		color                = color * colorScale;

		uint32_t type = n < MAX_LIGHTS_COUNT / 2 ? 0 : 1;

		glm::vec3 coneDir      = randVecGaussian();
		float     coneInner    = (randFloat() * .2f + .025f) * pi;
		float     coneOuter    = coneInner + randFloat() * .1f * pi;
		float     halfOuterCos = cos(coneOuter * 0.5f);

		lightData[n].position     = pos;
		lightData[n].color        = color;
		lightData[n].coneDir      = coneDir;
		lightData[n].coneAngles.x = 1.0f / (cos(coneInner) - cos(coneOuter));
		lightData[n].coneAngles.y = cos(coneOuter);
		lightData[n].coneAngles.z = 1.0f / (halfOuterCos * halfOuterCos);
		lightData[n].radius       = lightRadius;
		lightData[n].intensity    = randFloat() * 5.0f;
		lightData[n].lightType    = type;
		lightData[n].padding      = 0;

		centers[n] = pos;
		radius[n]  = lightRadius;
		colors[n]  = color;
	}

	lightBuffer->update(&lightData[0], MAX_LIGHTS_COUNT * sizeof(LightBuffer));
	debugDrawPass->add_bounding_sphere(std::move(centers), std::move(radius), std::move(colors));
}

void forward_plus::prepare_pipelines()
{
	Device & refDevice    = *device.get();
	auto &   extent2d     = get_render_context().get_surface_extent();
	uint32_t windowWidth  = extent2d.width;
	uint32_t windowHeight = extent2d.height;

	VkExtent3D extent{windowWidth, windowHeight, 1};
	VkExtent3D shadowExtent{2048, 2048, 1};
	// Create Render Image
	Image depthImage(refDevice, extent, VK_FORMAT_D32_SFLOAT, VK_IMAGE_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT | VK_IMAGE_USAGE_SAMPLED_BIT | VK_IMAGE_USAGE_STORAGE_BIT, VmaMemoryUsage::VMA_MEMORY_USAGE_GPU_ONLY);
	Image colorImage(refDevice, extent, VK_FORMAT_R8G8B8A8_SRGB, VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_SAMPLED_BIT | VK_IMAGE_USAGE_TRANSFER_SRC_BIT, VmaMemoryUsage::VMA_MEMORY_USAGE_GPU_ONLY);
	Image shadowImage(refDevice, shadowExtent, VK_FORMAT_D32_SFLOAT, VK_IMAGE_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT | VK_IMAGE_USAGE_SAMPLED_BIT | VK_IMAGE_USAGE_STORAGE_BIT, VmaMemoryUsage::VMA_MEMORY_USAGE_GPU_ONLY);

	linearDepthImage      = std::make_shared<Image>(refDevice, extent, VK_FORMAT_R32_SFLOAT, VK_IMAGE_USAGE_STORAGE_BIT | VK_IMAGE_USAGE_SAMPLED_BIT, VmaMemoryUsage::VMA_MEMORY_USAGE_GPU_ONLY);
	linearDepthImageView  = std::make_shared<ImageView>(*linearDepthImage, VK_IMAGE_VIEW_TYPE_2D);
	screenShadowImage     = std::make_shared<Image>(refDevice, extent, VK_FORMAT_R8_UNORM, VK_IMAGE_USAGE_STORAGE_BIT | VK_IMAGE_USAGE_SAMPLED_BIT, VmaMemoryUsage::VMA_MEMORY_USAGE_GPU_ONLY);
	screenShadowImageView = std::make_shared<ImageView>(*screenShadowImage, VK_IMAGE_VIEW_TYPE_2D);

	std::vector<Image> offScreenImgs;
	offScreenImgs.emplace_back(std::move(colorImage));
	offScreenImgs.emplace_back(std::move(depthImage));
	offScreenRT = std::make_shared<RenderTarget>(std::move(offScreenImgs));

	std::vector<Image> shadowImgs;
	shadowImgs.emplace_back(std::move(shadowImage));
	shadowMapRT = std::make_shared<RenderTarget>(std::move(shadowImgs));

	{
		ShaderSource vs = ShaderProgram::FindShaderSource(std::string("forward_plus/depth_only.vert"));
		ShaderSource fs = ShaderProgram::FindShaderSource(std::string("forward_plus/depth_only.frag"));
		depthPrePass    = std::make_unique<depth_only_pass>(*render_context, std::move(vs), std::move(fs));
		depthPrePass->prepare();

		vs         = ShaderProgram::FindShaderSource(std::string("forward_plus/depth_only.vert"));
		fs         = ShaderProgram::FindShaderSource(std::string("forward_plus/depth_only.frag"));
		shadowPass = std::make_unique<depth_only_pass>(*render_context, std::move(vs), std::move(fs));
		shadowPass->prepare();
		shadowPass->set_as_shadow_pipeline();

		auto *computeProgram = ShaderProgram::Find(std::string("screen_shadow"));
		screenShadowPass     = std::make_unique<screen_shadow_pass>(device.get(), render_context.get());
		screenShadowPass->prepare(computeProgram->GetShaderModules());

		computeProgram  = ShaderProgram::Find(std::string("linear_depth"));
		linearDepthPass = std::make_unique<linear_depth_pass>(device.get(), render_context.get());
		linearDepthPass->prepare(computeProgram->GetShaderModules());

		computeProgram = ShaderProgram::Find(std::string("light_grid"));
		lightGridPass  = std::make_unique<light_grid_pass>(device.get(), render_context.get());
		lightGridPass->prepare(computeProgram->GetShaderModules());

		vs            = ShaderProgram::FindShaderSource(std::string("forward_plus/screen_base.vert"));
		fs            = ShaderProgram::FindShaderSource(std::string("forward_plus/screen_base.frag"));
		showDepthPass = std::make_unique<show_depth_pass>(*render_context, std::move(vs), std::move(fs));
		showDepthPass->prepare();

		vs            = ShaderProgram::FindShaderSource(std::string("forward_plus/debug_draw.vert"));
		fs            = ShaderProgram::FindShaderSource(std::string("forward_plus/debug_draw.frag"));
		debugDrawPass = std::make_unique<debug_draw_pass>(*render_context, std::move(vs), std::move(fs));
		debugDrawPass->prepare();

		vs         = ShaderProgram::FindShaderSource(std::string("forward_plus/pbr_plus.vert"));
		fs         = ShaderProgram::FindShaderSource(std::string("forward_plus/pbr_plus.frag"));
		opaquePass = std::make_unique<opaque_pass>(*render_context, std::move(vs), std::move(fs), extent2d);
		opaquePass->prepare();
	}
}

void forward_plus::prepare_shaders()
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
		    {"screen_base", "forward_plus/screen_base.vert", "forward_plus/screen_base.frag"},
		    {"debug_draw", "forward_plus/debug_draw.vert", "forward_plus/debug_draw.frag"},
		};
		std::vector<CProgramSources> computeSourceFiles{
		    {"linear_depth", "forward_plus/linear_depth.comp"},
		    {"light_grid", "forward_plus/light_grid.comp"},
		    {"screen_shadow", "forward_plus/screen_shadow.comp"}};

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
}

void forward_plus::prepare_scene()
{
	// Scene contain lighting/Texture(both image and sampler)/Material/Mesh/Camera
	load_scene("scenes/sponza/Sponza01.gltf");

	auto                   sceneAABB = std::make_unique<sg::AABB>();
	auto &                 meshs     = scene->get_components<sg::Mesh>();
	std::vector<glm::vec3> center;
	std::vector<glm::vec3> extent;
	for (size_t i = 0; i < meshs.size(); i++)
	{
		const auto &bounds = meshs[i]->get_bounds();
		for (auto &node : meshs[i]->get_nodes())
		{
			auto     node_transform = node->get_transform().get_world_matrix();
			sg::AABB world_bounds{bounds.get_min(), bounds.get_max()};
			world_bounds.transform(node_transform);
			center.push_back(world_bounds.get_center());
			extent.push_back(world_bounds.get_scale() * 0.5f);
			sceneAABB->update(world_bounds.get_min());
			sceneAABB->update(world_bounds.get_max());
		}
	}

	center.push_back(sceneAABB->get_center());
	extent.push_back(sceneAABB->get_scale() * 0.5f);
	debugDrawPass->add_bounding_box(std::move(center), std::move(extent));

	{
		GLTFLoader loader{*device};
		sphere_mesh = loader.read_simple_model_from_file("scenes/unit_sphere.gltf", 0);
		cube_mesh   = loader.read_simple_model_from_file("scenes/unit_cube.gltf", 0);
	}

	auto &camera_node = vkb::add_free_camera(*scene, "main_camera", get_render_context().get_surface_extent());

	camera = &camera_node.get_component<vkb::sg::Camera>();
	camera->set_far_plane(10000.0f);
	camera->set_near_plane(0.01f);

	auto &lights       = scene->get_components<sg::Light>();
	auto  direct_light = std::find_if(lights.begin(), lights.end(), [](sg::Light *iter) -> bool { return iter->get_light_type() == sg::LightType::Directional; });
	DEBUG_ASSERT(direct_light != lights.end(), "Direction light missing");

	// hack sun direction
	sg::LightProperties sunLight;
	sunLight.direction = glm::normalize(glm::vec3(-1.0, -1.0, 0.0));

	auto light_node          = (*direct_light)->get_node();
	auto direct_light_camera = std::make_unique<shadow_camera>("light_camera");
	direct_light_camera->set_node(*light_node);
	light_node->set_component(*direct_light_camera);

	light_camera              = direct_light_camera.get();
	glm::vec2 shadowMapExtent = glm::vec2(shadowMapRT->get_extent().width, shadowMapRT->get_extent().height);
	(*direct_light)->set_properties(sunLight);
	light_camera->set_up(sunLight.direction, sceneAABB->get_center(), glm::vec3(2500, 2500, 2500), shadowMapExtent, 16);
	scene->add_component(std::move(direct_light_camera));

	if (!scene->get_root_node().has_component<sg::AABB>())
	{
		scene->add_component(std::move(sceneAABB), scene->get_root_node());
	}
}

void forward_plus::render(float delta_time)
{
	std::multimap<float, std::pair<sg::Node *, sg::SubMesh *>> opaqueNodes;
	std::multimap<float, std::pair<sg::Node *, sg::SubMesh *>> transparentNodes;
	std::multimap<float, std::pair<sg::Node *, sg::SubMesh *>> shadowNodes;

	sg::Transform &mainCameraRTS  = camera->get_node()->get_transform();
	glm::vec3      cameraForward  = glm::mat3(mainCameraRTS.get_world_matrix()) * glm::vec3(0.0, 0.0, 1.0);
	sg::Transform &lightCameraRTS = light_camera->get_node()->get_transform();
	glm::vec3      lightForward   = glm::mat3(lightCameraRTS.get_world_matrix()) * glm::vec3(0.0, 0.0, 1.0);

	get_sorted_nodes(cameraForward, mainCameraRTS.get_world_translation(), &opaqueNodes, &transparentNodes);
	get_sorted_nodes(lightForward, light_camera->get_shadow_center(), &shadowNodes);

	// reverse z
	std::vector<VkClearValue>
	    zeroClearValue{initializers::clear_color_value(0.0, 0.0, 0.0, 0.0), initializers::clear_depth_stencil_value(0.0, 0)};
	std::vector<VkClearValue>
	    oneClearValue{initializers::clear_depth_stencil_value(1.0, 0)};
	VkExtent2D     extent  = get_render_context().get_surface_extent();
	RenderContext &context = get_render_context();

	CommandBuffer &commandBuffer = context.begin();
	commandBuffer.begin(VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT);

	{
		auto &     shadowImage = shadowMapRT->get_views()[0].get_image();
		VkExtent2D shadowExtent{shadowImage.get_extent().width, shadowImage.get_extent().height};
		set_viewport_and_scissor(commandBuffer, shadowExtent);

		std::vector<LoadStoreInfo> loadStoreInfos;
		loadStoreInfos.emplace_back(LoadStoreInfo{VK_ATTACHMENT_LOAD_OP_CLEAR, VK_ATTACHMENT_STORE_OP_STORE});

		std::vector<SubpassInfo> subPassInfos;
		subPassInfos.emplace_back(SubpassInfo{{}, {}, {}, false, 0, VK_RESOLVE_MODE_NONE});

		auto &render_pass  = device->get_resource_cache().request_render_pass(shadowMapRT->get_attachments(), loadStoreInfos, subPassInfos);
		auto &frame_buffer = device->get_resource_cache().request_framebuffer(*shadowMapRT, render_pass);
		commandBuffer.begin_render_pass(*shadowMapRT, render_pass, frame_buffer, oneClearValue);

		shadowPass->set_up(light_camera, &shadowNodes);
		shadowPass->draw(commandBuffer);

		commandBuffer.end_render_pass();
	}

	set_viewport_and_scissor(commandBuffer, extent);

	// depth pre pass
	{
		std::vector<LoadStoreInfo> loadStoreInfos;
		loadStoreInfos.emplace_back(LoadStoreInfo{VK_ATTACHMENT_LOAD_OP_DONT_CARE, VK_ATTACHMENT_STORE_OP_DONT_CARE});
		loadStoreInfos.emplace_back(LoadStoreInfo{VK_ATTACHMENT_LOAD_OP_CLEAR, VK_ATTACHMENT_STORE_OP_STORE});

		std::vector<SubpassInfo> subPassInfos;
		subPassInfos.emplace_back(SubpassInfo{{}, {}, {}, false, 0, VK_RESOLVE_MODE_NONE});

		auto &render_pass  = device->get_resource_cache().request_render_pass(offScreenRT->get_attachments(), loadStoreInfos, subPassInfos);
		auto &frame_buffer = device->get_resource_cache().request_framebuffer(*offScreenRT, render_pass);
		commandBuffer.begin_render_pass(*offScreenRT, render_pass, frame_buffer, zeroClearValue);

		depthPrePass->set_up(camera, &opaqueNodes);
		depthPrePass->draw(commandBuffer);

		commandBuffer.end_render_pass();
	}

	ImageView &depthView     = const_cast<ImageView &>(offScreenRT->get_views()[1]);
	ImageView &shadowMapView = const_cast<ImageView &>(shadowMapRT->get_views()[0]);
	{
		vkb::ImageMemoryBarrier barrier{};
		barrier.src_stage_mask  = VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT;
		barrier.dst_stage_mask  = VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT;
		barrier.dst_access_mask = VK_ACCESS_SHADER_WRITE_BIT;
		barrier.new_layout      = VK_IMAGE_LAYOUT_GENERAL;
		commandBuffer.image_memory_barrier(*linearDepthImageView, barrier);
		commandBuffer.image_memory_barrier(*screenShadowImageView, barrier);

		barrier.src_stage_mask  = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
		barrier.dst_stage_mask  = VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT;
		barrier.src_access_mask = 0;
		barrier.dst_access_mask = VK_ACCESS_SHADER_READ_BIT;
		barrier.old_layout      = VK_IMAGE_LAYOUT_UNDEFINED;
		barrier.new_layout      = VK_IMAGE_LAYOUT_DEPTH_STENCIL_READ_ONLY_OPTIMAL;
		commandBuffer.image_memory_barrier(depthView, barrier);
		commandBuffer.image_memory_barrier(shadowMapView, barrier);

		// linear depth
		linearDepthPass->set_up(&depthView, linearDepthImageView.get(), camera);
		linearDepthPass->dispatch(commandBuffer);

		// screen shadow
		screenShadowPass->set_up(&depthView, &shadowMapView, screenShadowImageView.get());
		screenShadowPass->set_camera(camera, light_camera);
		screenShadowPass->dispatch(commandBuffer);
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
		//commandBuffer.buffer_memory_barrier(*lightMaskBuffer, 0, lightMaskBuffer->get_size(), bufferBarrier);

		lightGridPass->set_up(lightBuffer.get(), lightGridBuffer.get(), linearDepthImageView.get(), camera);
		lightGridPass->dispatch(commandBuffer);

		bufferBarrier.src_stage_mask  = VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT;
		bufferBarrier.dst_stage_mask  = VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT;
		bufferBarrier.src_access_mask = VK_ACCESS_SHADER_WRITE_BIT;
		bufferBarrier.dst_access_mask = VK_ACCESS_SHADER_READ_BIT;
		commandBuffer.buffer_memory_barrier(*lightGridBuffer, 0, lightBuffer->get_size(), bufferBarrier);
		//commandBuffer.buffer_memory_barrier(*lightMaskBuffer, 0, lightMaskBuffer->get_size(), bufferBarrier);
	}

	{
		// Before render main scene
		ImageMemoryBarrier barrier{};
		barrier.src_stage_mask  = VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT;
		barrier.dst_stage_mask  = VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT;
		barrier.src_access_mask = VK_ACCESS_SHADER_READ_BIT;
		barrier.dst_access_mask = VK_ACCESS_SHADER_READ_BIT;
		barrier.old_layout      = VK_IMAGE_LAYOUT_GENERAL;
		barrier.new_layout      = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
		commandBuffer.image_memory_barrier(*linearDepthImageView, barrier);
		barrier.src_access_mask = VK_ACCESS_SHADER_WRITE_BIT;
		commandBuffer.image_memory_barrier(*screenShadowImageView, barrier);

		barrier.dst_stage_mask  = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
		barrier.src_access_mask = VK_ACCESS_SHADER_READ_BIT;
		barrier.dst_access_mask = VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT;
		barrier.old_layout      = VK_IMAGE_LAYOUT_DEPTH_STENCIL_READ_ONLY_OPTIMAL;
		barrier.new_layout      = VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL;
		commandBuffer.image_memory_barrier(depthView, barrier);
	}

	if (debugDepth)
	{
		// show linear depth pass
		std::vector<LoadStoreInfo> loadStoreInfos;
		loadStoreInfos.emplace_back(LoadStoreInfo{VK_ATTACHMENT_LOAD_OP_CLEAR, VK_ATTACHMENT_STORE_OP_STORE});
		loadStoreInfos.emplace_back(LoadStoreInfo{VK_ATTACHMENT_LOAD_OP_DONT_CARE, VK_ATTACHMENT_STORE_OP_DONT_CARE});

		std::vector<SubpassInfo> subPassInfos;
		subPassInfos.emplace_back(SubpassInfo{{}, {0}, {}, false, 0, VK_RESOLVE_MODE_NONE});

		auto &render_pass  = device->get_resource_cache().request_render_pass(offScreenRT->get_attachments(), loadStoreInfos, subPassInfos);
		auto &frame_buffer = device->get_resource_cache().request_framebuffer(*offScreenRT, render_pass);

		commandBuffer.begin_render_pass(*offScreenRT, render_pass, frame_buffer, zeroClearValue);
		//showDepthPass->set_up(postProcessVB.get(), &shadowMapView);
		//showDepthPass->set_up(postProcessVB.get(), linearDepthImageView.get());
		showDepthPass->set_up(postProcessVB.get(), screenShadowImageView.get());
		showDepthPass->draw(commandBuffer);

		gui->draw(commandBuffer);

		commandBuffer.end_render_pass();
	}
	else
	{
		std::vector<LoadStoreInfo> loadStoreInfos;
		loadStoreInfos.emplace_back(LoadStoreInfo{VK_ATTACHMENT_LOAD_OP_CLEAR, VK_ATTACHMENT_STORE_OP_STORE});
		loadStoreInfos.emplace_back(LoadStoreInfo{VK_ATTACHMENT_LOAD_OP_LOAD, VK_ATTACHMENT_STORE_OP_DONT_CARE});

		std::vector<SubpassInfo> subPassInfos;
		subPassInfos.emplace_back(SubpassInfo{{}, {0}, {}, false, 0, VK_RESOLVE_MODE_NONE});

		auto &renderPass  = device->get_resource_cache().request_render_pass(offScreenRT->get_attachments(), loadStoreInfos, subPassInfos);
		auto &frameBuffer = device->get_resource_cache().request_framebuffer(*offScreenRT, renderPass);

		commandBuffer.begin_render_pass(*offScreenRT, renderPass, frameBuffer, zeroClearValue);

		opaquePass->screenShadow = screenShadowImageView.get();
		opaquePass->set_up(lightGridBuffer.get(), lightBuffer.get(), camera, &opaqueNodes);
		opaquePass->draw(commandBuffer);
		

		debugDrawPass->draw_sphere = drawLight;
		debugDrawPass->draw_box    = drawAABB;
		debugDrawPass->set_up(sphere_mesh.get(), cube_mesh.get(), camera);
		debugDrawPass->draw(commandBuffer);

		gui->draw(commandBuffer);

		commandBuffer.end_render_pass();
	}

	blit_and_present(commandBuffer);
	commandBuffer.end();
	context.submit(commandBuffer);
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
	update_gui(delta_time);
	render(delta_time);
}

void forward_plus::draw_gui()
{
	gui->show_options_window(
	    /* body = */ [this]() {
		    ImGui::Checkbox("Show Depth", &debugDepth);
		    ImGui::Checkbox("Draw AABB", &drawAABB);
		    ImGui::Checkbox("Draw Light Outline", &drawLight);
	    },
	    /* lines = */ 3);
}

void forward_plus::get_sorted_nodes(glm::vec3 direction, glm::vec3 position, std::multimap<float, std::pair<sg::Node *, sg::SubMesh *>> *opaque_nodes, std::multimap<float, std::pair<sg::Node *, sg::SubMesh *>> *transparent_nodes)
{
	auto meshes = scene->get_components<sg::Mesh>();
	for (auto &mesh : meshes)
	{
		for (auto &node : mesh->get_nodes())
		{
			auto node_transform = node->get_transform().get_world_matrix();

			const sg::AABB &mesh_bounds = mesh->get_bounds();

			sg::AABB world_bounds{mesh_bounds.get_min(), mesh_bounds.get_max()};
			world_bounds.transform(node_transform);

			glm::vec3 delta    = world_bounds.get_center() - position;
			float     distance = glm::dot(direction, delta);

			for (auto &sub_mesh : mesh->get_submeshes())
			{
				if (sub_mesh->get_material()->alpha_mode == sg::AlphaMode::Blend && transparent_nodes != nullptr)
				{
					transparent_nodes->emplace(distance, std::make_pair(node, sub_mesh));
				}
				else
				{
					opaque_nodes->emplace(distance, std::make_pair(node, sub_mesh));
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
