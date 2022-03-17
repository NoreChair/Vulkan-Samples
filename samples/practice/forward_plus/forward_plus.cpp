#include "forward_plus.h"

#include "GraphicContext.h"
#include "ShaderProgram.h"
#include "SphericalHarmonic.h"
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
using namespace GraphicContext;

#ifdef SH_TEST
float s_shWeight[28];

void GenTestSH(float *pSh)
{
	glm::vec4 colors[6] = {
	    glm::vec4(1.0f, 1.0f, 1.0f, 1.0f),        // white
	    glm::vec4(0.5f, 1.0f, 1.0f, 1.0f),        // azure
	    glm::vec4(1.0f, 1.0f, 0.0f, 1.0f),        // yellow
	    glm::vec4(1.0f, 0.0f, 0.0f, 1.0f),        // red
	    glm::vec4(0.0f, 1.0f, 0.0f, 1.0f),        // green
	    glm::vec4(0.0f, 0.0f, 1.0f, 1.0f)         // blue
	};

	int width, height;
	width = height = 8;
	// +-x, +-y, +-z
	float *pImageData = new float[width * height * 4 * 6];
	float *iter       = pImageData;

	int size = width * height;
	for (int j = 0; j < 6; j++)
	{
		glm::vec4 color = colors[j];
		for (int i = 0; i < size; i++)
		{
			iter = pImageData + ((j * size + i) * 4);
			memcpy(iter, &color, sizeof(float) * 4);
		}
	}

	float rgbSH[27];
	ProjectFromCubeMap(pImageData, width, height, 4, rgbSH);

	delete[] pImageData;

	// reorder to RGB<(L1,L0), L2(0~3)>, L2<rgb(4),0.0>
	iter = pSh;
	for (int i = 0; i < 3; i++)
	{
		int offset = i * 9;
		*(iter++)  = rgbSH[offset + 1];
		*(iter++)  = rgbSH[offset + 2];
		*(iter++)  = rgbSH[offset + 3];
		*(iter++)  = rgbSH[offset + 0];
		*(iter++)  = rgbSH[offset + 4];
		*(iter++)  = rgbSH[offset + 5];
		*(iter++)  = rgbSH[offset + 6];
		*(iter++)  = rgbSH[offset + 7];
	}
	*(iter++) = rgbSH[8];
	*(iter++) = rgbSH[17];
	*(iter++) = rgbSH[26];
	*iter     = 0.0f;
}

void DrawSHShpere(RenderContext &context, CommandBuffer &commandBuffer, sg::Camera *camera, sg::SubMesh *pSphere, glm::vec4 *pSh)
{
	DepthStencilState  defaultDepthState;
	RasterizationState defaultRasterState;

	commandBuffer.set_depth_stencil_state(defaultDepthState);
	commandBuffer.set_rasterization_state(defaultRasterState);

	auto            program = ShaderProgram::Find(std::string("sh_sphere"));
	PipelineLayout &layout  = context.get_device().get_resource_cache().request_pipeline_layout(program->GetShaderModules());
	commandBuffer.bind_pipeline_layout(layout);

	struct
	{
		glm::mat4 model;
		glm::mat4 viewProj;
		glm::vec4 rSH0;
		glm::vec4 rSH1;
		glm::vec4 gSH0;
		glm::vec4 gSH1;
		glm::vec4 bSH0;
		glm::vec4 bSH1;
		glm::vec4 rgbSH2;
	} ubo;

	ubo.model    = glm::translate(glm::scale(glm::vec3(50.0)), glm::vec3(0.0, 8.0, 0.0));
	ubo.viewProj = vkb::vulkan_style_projection(camera->get_projection()) * camera->get_view();
	ubo.rSH0     = pSh[0];
	ubo.rSH1     = pSh[1];
	ubo.gSH0     = pSh[2];
	ubo.gSH1     = pSh[3];
	ubo.bSH0     = pSh[4];
	ubo.bSH1     = pSh[5];
	ubo.rgbSH2   = pSh[6];

	auto allocation = context.get_active_frame().allocate_buffer(VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT, sizeof(ubo));
	allocation.update(ubo);
	commandBuffer.bind_buffer(allocation.get_buffer(), allocation.get_offset(), allocation.get_size(), 0, 0, 0);

	auto             vertex_input_resources = layout.get_resources(ShaderResourceType::Input, VK_SHADER_STAGE_VERTEX_BIT);
	VertexInputState vertex_input_state;

	for (auto &input_resource : vertex_input_resources)
	{
		sg::VertexAttribute attribute;

		if (!pSphere->get_attribute(input_resource.name, attribute))
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

	for (auto &input_resource : vertex_input_resources)
	{
		const auto &buffer_iter = pSphere->vertex_buffers.find(input_resource.name);

		if (buffer_iter != pSphere->vertex_buffers.end())
		{
			std::vector<std::reference_wrapper<const core::Buffer>> buffers;
			buffers.emplace_back(std::ref(buffer_iter->second));

			commandBuffer.bind_vertex_buffers(input_resource.location, std::move(buffers), {0});
		}
	}

	if (pSphere->vertex_indices != 0)
	{
		commandBuffer.bind_index_buffer(*pSphere->index_buffer, pSphere->index_offset, pSphere->index_type);
		commandBuffer.draw_indexed(pSphere->vertex_indices, 1, 0, 0, 0);
	}
	else
	{
		commandBuffer.draw(pSphere->vertices_count, 1, 0, 0);
	}
}
#endif

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
{}

bool forward_plus::prepare(Platform &platform)
{
	if (!VulkanSample::prepare(platform))
	{
		return false;
	}

	auto &extent2d = get_render_context().get_surface_extent();
	GraphicContext::Init(*device.get(), extent2d.width, extent2d.height);

	prepare_shaders();
	prepare_pipelines();
	prepare_scene();
	prepare_light();

	//stats->request_stats({ vkb::StatIndex::cpu_cycles,vkb::StatIndex::gpu_cycles });
	gui = std::make_unique<vkb::Gui>(*this, platform.get_window(), stats.get());
#ifdef GenTestSH
	GenTestSH(s_shWeight);
#endif
	return true;
}

void forward_plus::prepare_render_context()
{
	// so we can just copy offscreen image to swap chain image
	auto &properties = render_context->get_swapchain().get_properties();
	//properties.image_usage |= VK_IMAGE_USAGE_STORAGE_BIT;
	properties.image_usage |= VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT;
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
}

void forward_plus::resize(const uint32_t width, const uint32_t height)
{
	VulkanSample::resize(width, height);
}

void forward_plus::prepare_light()
{
	auto &    sceneAABB = scene->get_root_node().get_component<sg::AABB>();
	glm::vec3 posScale  = sceneAABB.get_max() - sceneAABB.get_min();
	glm::vec3 posBias   = sceneAABB.get_min();

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
	opaquePass = std::make_unique<opaque_pass>(*render_context, std::move(vs), std::move(fs), get_render_context().get_surface_extent());
	opaquePass->prepare();
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
		    {"sky", "forward_plus/sky.vert", "forward_plus/sky.frag"},
		    {"sh_sphere", "forward_plus/test/sh_sphere.vert", "forward_plus/test/sh_sphere.frag"},
		    {"extract_luma_fs", "forward_plus/screen_base.vert", "forward_plus/hdr/ExtractLuma.frag"},
		    {"tone_mapping_fs", "forward_plus/screen_base.vert", "forward_plus/hdr/ToneMapping.frag"}};

		std::vector<CProgramSources> computeSourceFiles{
		    {"linear_depth", "forward_plus/linear_depth.comp"},
		    {"light_grid", "forward_plus/light_grid.comp"},
		    {"screen_shadow", "forward_plus/screen_shadow.comp"},
		    {"adjust_exposure", "forward_plus/hdr/AdjustExposure.comp"},
		    {"extract_luma", "forward_plus/hdr/ExtractLuma.comp"},
		    {"gen_histogram", "forward_plus/hdr/GenerateHistogram.comp"},
		    {"tone_mapping", "forward_plus/hdr/ToneMapping.comp"}};

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
	sunLight.direction       = glm::normalize(glm::vec3(0.2, -1.0, 0.2));
	opaquePass->sunDirection = glm::vec4(sunLight.direction, 0.0);

	auto light_node          = (*direct_light)->get_node();
	auto direct_light_camera = std::make_unique<shadow_camera>("light_camera");
	direct_light_camera->set_node(*light_node);
	light_node->set_component(*direct_light_camera);

	light_camera              = direct_light_camera.get();
	glm::vec2 shadowMapExtent = glm::vec2(shadowImage->get_extent().width, shadowImage->get_extent().height);
	(*direct_light)->set_properties(sunLight);
	light_camera->set_up(sunLight.direction, sceneAABB->get_center(), glm::vec3(2500, 2500, 2500), shadowMapExtent, 16);
	scene->add_component(std::move(direct_light_camera));

	if (!scene->get_root_node().has_component<sg::AABB>())
	{
		scene->add_component(std::move(sceneAABB), scene->get_root_node());
	}
}

void forward_plus::process_shadow(vkb::CommandBuffer &commandBuffer)
{
	vkb::ImageMemoryBarrier barrier{};
	barrier.src_stage_mask  = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
	barrier.dst_stage_mask  = VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT;
	barrier.src_access_mask = VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT;
	barrier.dst_access_mask = VK_ACCESS_SHADER_READ_BIT;
	barrier.old_layout      = VK_IMAGE_LAYOUT_DEPTH_ATTACHMENT_OPTIMAL;
	barrier.new_layout      = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
	commandBuffer.image_memory_barrier(*sceneDepthImageView, barrier);
	commandBuffer.image_memory_barrier(*shadowImageView, barrier);

    // linear depth
	linearDepthPass->dispatch(commandBuffer, sceneDepthImageView.get(), linearDepthImageView.get(), camera);

	// screen shadow
	screenShadowPass->set_up(sceneDepthImageView.get(), shadowImageView.get(), screenShadowImageView.get());
	screenShadowPass->set_camera(camera, light_camera);
	screenShadowPass->dispatch(commandBuffer);
}

void forward_plus::process_light_buffer(vkb::CommandBuffer &commandBuffer)
{
	vkb::ImageMemoryBarrier barrier{};
	barrier.src_stage_mask  = VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT;
	barrier.dst_stage_mask  = VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT;
	barrier.src_access_mask = VK_ACCESS_SHADER_WRITE_BIT;
	barrier.dst_access_mask = VK_ACCESS_SHADER_READ_BIT;
	barrier.new_layout      = VK_IMAGE_LAYOUT_GENERAL;
	commandBuffer.image_memory_barrier(*linearDepthImageView, barrier);

	BufferMemoryBarrier bufferBarrier{};
	bufferBarrier.src_stage_mask  = VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT;
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

void forward_plus::process_HDR(vkb::CommandBuffer &commandBuffer)
{
	auto &swapchainView = const_cast<ImageView &>(render_context->get_active_frame().get_render_target().get_views()[0]);
#if 0
    {
        // Prepare to process tone mapping
        ImageMemoryBarrier barrier{};
        barrier.src_stage_mask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
        barrier.dst_stage_mask = VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT;
        barrier.src_access_mask = VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT;
        barrier.dst_access_mask = VK_ACCESS_SHADER_READ_BIT;
        barrier.old_layout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
        barrier.new_layout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
        commandBuffer.image_memory_barrier(*hdrColorImageView, barrier);

        barrier.src_stage_mask = VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT;
        barrier.dst_stage_mask = VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT;
        barrier.src_access_mask = 0;
        barrier.dst_access_mask = VK_ACCESS_SHADER_WRITE_BIT;
		barrier.old_layout      = VK_IMAGE_LAYOUT_GENERAL;
		barrier.new_layout      = VK_IMAGE_LAYOUT_GENERAL;
		commandBuffer.image_memory_barrier(*sdrColorImageView, barrier);
    }

    // HDR process and auto exposure
    {
        VkExtent3D imageExtent = hdrColorImage->get_extent();
        uint32_t groupCountX = (imageExtent.width + 15) / 16;
        uint32_t groupCountY = (imageExtent.height + 15) / 16;

        struct {
            glm::vec4 extent;
            float targetLuma;
            float adaptationRate;
            float minExposure;
            float maxExposure;
            uint32_t pixelCount;
        }toneMapUniforms;
        toneMapUniforms.extent = glm::vec4((float)imageExtent.width, (float)imageExtent.height, 1.0f / (float)imageExtent.width, 1.0f / (float)imageExtent.height);
        toneMapUniforms.targetLuma = 1.0f;
        toneMapUniforms.adaptationRate = 0.05f;
        toneMapUniforms.minExposure = -8.0f;
        toneMapUniforms.maxExposure = 8.0f;
        toneMapUniforms.pixelCount = imageExtent.width * imageExtent.height;

        auto allocation = context.get_active_frame().allocate_buffer(VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT, sizeof(toneMapUniforms));
        allocation.update(toneMapUniforms);

        std::vector<ShaderModule*> shaderMoudles = ShaderProgram::Find(std::string("extract_luma"))->GetShaderModules();
        vkb::PipelineLayout &extractLumaPipeline = device->get_resource_cache().request_pipeline_layout(shaderMoudles);
        commandBuffer.bind_pipeline_layout(extractLumaPipeline);
        commandBuffer.bind_buffer(allocation.get_buffer(), allocation.get_offset(), allocation.get_size(), 0, 0, 0);
        commandBuffer.bind_buffer(*exposureBuffer, 0, exposureBuffer->get_size(), 0, 1, 0);
        commandBuffer.bind_image(*hdrColorImageView, *linearClampSampler, 0, 2, 0);
        commandBuffer.bind_image(*lumaResultImageView, 0, 3, 0);
        commandBuffer.dispatch(groupCountX, groupCountY, 0);

        shaderMoudles = ShaderProgram::Find(std::string("tone_mapping"))->GetShaderModules();
        vkb::PipelineLayout &toneMapPipeline = device->get_resource_cache().request_pipeline_layout(shaderMoudles);
        commandBuffer.bind_pipeline_layout(toneMapPipeline);
        commandBuffer.bind_buffer(allocation.get_buffer(), allocation.get_offset(), allocation.get_size(), 0, 0, 0);
        commandBuffer.bind_buffer(*exposureBuffer, 0, exposureBuffer->get_size(), 0, 1, 0);
        commandBuffer.bind_image(*hdrColorImageView, *linearClampSampler, 0, 2, 0);
        commandBuffer.bind_image(*sdrColorImageView, 0, 3, 0);
        commandBuffer.dispatch(groupCountX, groupCountY, 0);

        ImageMemoryBarrier imageBarrier{};
        imageBarrier.src_stage_mask = VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT;
        imageBarrier.dst_stage_mask = VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT;
        imageBarrier.src_access_mask = VK_ACCESS_SHADER_WRITE_BIT;
        imageBarrier.dst_access_mask = VK_ACCESS_SHADER_READ_BIT;
        imageBarrier.old_layout = VK_IMAGE_LAYOUT_GENERAL;
        imageBarrier.new_layout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
        commandBuffer.image_memory_barrier(*lumaResultImageView, imageBarrier);

        shaderMoudles = ShaderProgram::Find(std::string("gen_histogram"))->GetShaderModules();
        vkb::PipelineLayout &histogramPipeline = device->get_resource_cache().request_pipeline_layout(shaderMoudles);
        commandBuffer.bind_pipeline_layout(histogramPipeline);
        commandBuffer.bind_buffer(allocation.get_buffer(), allocation.get_offset(), allocation.get_size(), 0, 0, 0);
        commandBuffer.bind_buffer(*lumaHistogram, 0, lumaHistogram->get_size(), 0, 1, 0);
        commandBuffer.bind_image(*lumaResultImageView, *linearClampSampler, 0, 2, 0);
        commandBuffer.dispatch(groupCountX, 0, 0);

        BufferMemoryBarrier bufferBarrier{};
        bufferBarrier.src_stage_mask = VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT;
        bufferBarrier.dst_stage_mask = VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT;
        bufferBarrier.src_access_mask = VK_ACCESS_SHADER_WRITE_BIT;
        bufferBarrier.dst_access_mask = VK_ACCESS_SHADER_READ_BIT;
        commandBuffer.buffer_memory_barrier(*lumaHistogram, 0, lumaHistogram->get_size(), bufferBarrier);
        bufferBarrier.src_stage_mask = VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT;
        bufferBarrier.dst_stage_mask = VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT;
        bufferBarrier.src_access_mask = VK_ACCESS_SHADER_READ_BIT;
        bufferBarrier.dst_access_mask = VK_ACCESS_SHADER_WRITE_BIT;
        commandBuffer.buffer_memory_barrier(*exposureBuffer, 0, exposureBuffer->get_size(), bufferBarrier);

        shaderMoudles = ShaderProgram::Find(std::string("adjust_exposure"))->GetShaderModules();
        vkb::PipelineLayout &autoExposurePipeline = device->get_resource_cache().request_pipeline_layout(shaderMoudles);
        commandBuffer.bind_pipeline_layout(autoExposurePipeline);
        commandBuffer.bind_buffer(allocation.get_buffer(), allocation.get_offset(), allocation.get_size(), 0, 0, 0);
        commandBuffer.bind_buffer(*exposureBuffer, 0, exposureBuffer->get_size(), 0, 1, 0);
        commandBuffer.bind_buffer(*lumaHistogram, 0, lumaHistogram->get_size(), 0, 2, 0);
        commandBuffer.dispatch(1, 0, 0);
    }

    {
		// Prepare to draw gui

		ImageMemoryBarrier barrier{};
		barrier.src_stage_mask  = VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT;
		barrier.dst_stage_mask  = VK_PIPELINE_STAGE_TRANSFER_BIT;
		barrier.src_access_mask = VK_ACCESS_SHADER_WRITE_BIT;
		barrier.dst_access_mask = VK_ACCESS_TRANSFER_READ_BIT;
		barrier.old_layout      = VK_IMAGE_LAYOUT_GENERAL;
		barrier.new_layout      = VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL;
		commandBuffer.image_memory_barrier(*sdrColorImageView, barrier);

		barrier.src_stage_mask  = VK_PIPELINE_STAGE_BOTTOM_OF_PIPE_BIT;
		barrier.dst_stage_mask  = VK_PIPELINE_STAGE_TRANSFER_BIT;
		barrier.src_access_mask = 0;
		barrier.dst_access_mask = VK_ACCESS_TRANSFER_WRITE_BIT;
		barrier.old_layout      = VK_IMAGE_LAYOUT_UNDEFINED;
		barrier.new_layout      = VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL;
		commandBuffer.image_memory_barrier(swapchainView, barrier);

		VkExtent3D  colorImageExtent = sdrColorImageView->get_image().get_extent();
		VkExtent3D  swapChainExtent  = swapchainView.get_image().get_extent();
		VkImageBlit blit{};
		blit.srcOffsets[0]             = VkOffset3D{0, 0, 0};
		blit.srcOffsets[1]             = VkOffset3D{(int32_t) colorImageExtent.width, (int32_t) colorImageExtent.height, 0};
		blit.dstOffsets[0]             = VkOffset3D{0, 0, 0};
		blit.dstOffsets[1]             = VkOffset3D{(int32_t) swapChainExtent.width, (int32_t) swapChainExtent.height, 0};
		blit.srcSubresource.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
		blit.srcSubresource.layerCount = 1;
		blit.dstSubresource.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
		blit.dstSubresource.layerCount = 1;

		commandBuffer.blit_image(sdrColorImageView->get_image(), swapchainView.get_image(), {blit});

        barrier.src_stage_mask = VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT;
        barrier.dst_stage_mask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
        barrier.src_access_mask = VK_ACCESS_SHADER_WRITE_BIT;
        barrier.dst_access_mask = VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT;
        barrier.old_layout = VK_IMAGE_LAYOUT_GENERAL;
        barrier.new_layout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
        commandBuffer.image_memory_barrier(swapchainView, barrier);
    }
#else

	// Prepare to process tone mapping
	ImageMemoryBarrier barrier{};
	barrier.src_stage_mask  = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
	barrier.dst_stage_mask  = VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT;
	barrier.src_access_mask = VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT;
	barrier.dst_access_mask = VK_ACCESS_SHADER_READ_BIT;
	barrier.old_layout      = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
	barrier.new_layout      = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
	commandBuffer.image_memory_barrier(*hdrColorImageView, barrier);

	full_screen_draw(commandBuffer, swapchainView, [this](CommandBuffer &cb, RenderContext &context) {
		struct
		{
			float exposure;
		} toneMapUniforms;

		toneMapUniforms.exposure = exposureValue;

		auto allocation = context.get_active_frame().allocate_buffer(VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT, sizeof(toneMapUniforms));
		allocation.update(toneMapUniforms);

		Device &device         = context.get_device();
		auto &  modules        = ShaderProgram::Find(std::string("tone_mapping_fs"))->GetShaderModules();
		auto &  pipelineLayout = device.get_resource_cache().request_pipeline_layout(modules);
		cb.bind_pipeline_layout(pipelineLayout);
		cb.bind_buffer(allocation.get_buffer(), allocation.get_offset(), allocation.get_size(), 0, 0, 0);
		cb.bind_image(*hdrColorImageView, *linearClampSampler, 0, 1, 0);
	});

	// auto exposure
#	if 1
	{
		VkExtent3D imageExtent = hdrColorImage->get_extent();
		uint32_t   groupCountX = (imageExtent.width + 15) / 16;
		uint32_t   groupCountY = (imageExtent.height + 15) / 16;

		struct
		{
			glm::vec4 extent;
			float     targetLuma;
			float     adaptationRate;
			float     minExposure;
			float     maxExposure;
			uint32_t  pixelCount;
		} toneMapUniforms;
		toneMapUniforms.extent         = glm::vec4((float) imageExtent.width, (float) imageExtent.height, 1.0f / (float) imageExtent.width, 1.0f / (float) imageExtent.height);
		toneMapUniforms.targetLuma     = 1.0f;
		toneMapUniforms.adaptationRate = 0.05f;
		toneMapUniforms.minExposure    = -8.0f;
		toneMapUniforms.maxExposure    = 8.0f;
		toneMapUniforms.pixelCount     = imageExtent.width * imageExtent.height;

		auto allocation = render_context->get_active_frame().allocate_buffer(VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT, sizeof(toneMapUniforms));
		allocation.update(toneMapUniforms);

		//std::vector<ShaderModule *> shaderMoudles       = ShaderProgram::Find(std::string("extract_luma"))->GetShaderModules();
		//vkb::PipelineLayout &       extractLumaPipeline = device->get_resource_cache().request_pipeline_layout(shaderMoudles);
		//commandBuffer.bind_pipeline_layout(extractLumaPipeline);
		//commandBuffer.bind_buffer(allocation.get_buffer(), allocation.get_offset(), allocation.get_size(), 0, 0, 0);
		//commandBuffer.bind_buffer(*exposureBuffer, 0, exposureBuffer->get_size(), 0, 1, 0);
		//commandBuffer.bind_image(*hdrColorImageView, *linearClampSampler, 0, 2, 0);
		//commandBuffer.bind_image(*lumaResultImageView, 0, 3, 0);
		//commandBuffer.dispatch(groupCountX, groupCountY, 0);

		full_screen_draw(commandBuffer, *lumaResultImageView, [this, &allocation](CommandBuffer &cb, RenderContext &context) {
			Device &device         = context.get_device();
			auto &  modules        = ShaderProgram::Find(std::string("extract_luma_fs"))->GetShaderModules();
			auto &  pipelineLayout = device.get_resource_cache().request_pipeline_layout(modules);
			cb.bind_pipeline_layout(pipelineLayout);
			cb.bind_buffer(allocation.get_buffer(), allocation.get_offset(), allocation.get_size(), 0, 0, 0);
			cb.bind_buffer(*exposureBuffer, 0, exposureBuffer->get_size(), 0, 1, 0);
			cb.bind_image(*hdrColorImageView, *linearClampSampler, 0, 2, 0);
		});

		ImageMemoryBarrier imageBarrier{};
		imageBarrier.src_stage_mask  = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
		imageBarrier.dst_stage_mask  = VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT;
		imageBarrier.src_access_mask = VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT;
		imageBarrier.dst_access_mask = VK_ACCESS_SHADER_READ_BIT;
		imageBarrier.old_layout      = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
		imageBarrier.new_layout      = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
		commandBuffer.image_memory_barrier(*lumaResultImageView, imageBarrier);

        BufferMemoryBarrier bufferBarrier{};
		bufferBarrier.src_stage_mask  = VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT;
		bufferBarrier.dst_stage_mask  = VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT;
		bufferBarrier.src_access_mask = 0;
		bufferBarrier.dst_access_mask = VK_ACCESS_SHADER_WRITE_BIT;
		commandBuffer.buffer_memory_barrier(*lumaHistogram, 0, lumaHistogram->get_size(), bufferBarrier);

		auto shaderMoudles       = ShaderProgram::Find(std::string("gen_histogram"))->GetShaderModules();
		vkb::PipelineLayout &histogramPipeline = device->get_resource_cache().request_pipeline_layout(shaderMoudles);
		commandBuffer.bind_pipeline_layout(histogramPipeline);
		commandBuffer.bind_buffer(allocation.get_buffer(), allocation.get_offset(), allocation.get_size(), 0, 0, 0);
		commandBuffer.bind_buffer(*lumaHistogram, 0, lumaHistogram->get_size(), 0, 1, 0);
		commandBuffer.bind_image(*lumaResultImageView, *linearClampSampler, 0, 2, 0);
		commandBuffer.dispatch(groupCountX, 0, 0);

		bufferBarrier.src_stage_mask  = VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT;
		bufferBarrier.dst_stage_mask  = VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT;
		bufferBarrier.src_access_mask = VK_ACCESS_SHADER_WRITE_BIT;
		bufferBarrier.dst_access_mask = VK_ACCESS_SHADER_READ_BIT;
		commandBuffer.buffer_memory_barrier(*lumaHistogram, 0, lumaHistogram->get_size(), bufferBarrier);
		bufferBarrier.src_stage_mask  = VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT;
		bufferBarrier.dst_stage_mask  = VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT;
		bufferBarrier.src_access_mask = VK_ACCESS_SHADER_READ_BIT;
		bufferBarrier.dst_access_mask = VK_ACCESS_SHADER_WRITE_BIT;
		commandBuffer.buffer_memory_barrier(*exposureBuffer, 0, exposureBuffer->get_size(), bufferBarrier);

		shaderMoudles                             = ShaderProgram::Find(std::string("adjust_exposure"))->GetShaderModules();
		vkb::PipelineLayout &autoExposurePipeline = device->get_resource_cache().request_pipeline_layout(shaderMoudles);
		commandBuffer.bind_pipeline_layout(autoExposurePipeline);
		commandBuffer.bind_buffer(allocation.get_buffer(), allocation.get_offset(), allocation.get_size(), 0, 0, 0);
		commandBuffer.bind_buffer(*exposureBuffer, 0, exposureBuffer->get_size(), 0, 1, 0);
		commandBuffer.bind_buffer(*lumaHistogram, 0, lumaHistogram->get_size(), 0, 2, 0);
		commandBuffer.dispatch(1, 0, 0);
	}
#	endif
#endif
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
	std::vector<VkClearValue> colorClearValue{initializers::clear_color_value(0.0, 0.0, 0.0, 0.0), initializers::clear_depth_stencil_value(0.0, 0)};
	std::vector<VkClearValue> oneClearValue{initializers::clear_depth_stencil_value(1.0, 0)};
	std::vector<VkClearValue> zeroClearValue{initializers::clear_depth_stencil_value(0.0, 0)};

	VkExtent2D     extent  = get_render_context().get_surface_extent();
	RenderContext &context = get_render_context();

	CommandBuffer &commandBuffer = context.begin();
	commandBuffer.begin(VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT);

	{
		VkExtent2D shadowExtent{shadowImage->get_extent().width, shadowImage->get_extent().height};
		set_viewport_and_scissor(commandBuffer, shadowExtent);

		std::vector<LoadStoreInfo> loadStoreInfos;
		loadStoreInfos.emplace_back(LoadStoreInfo{VK_ATTACHMENT_LOAD_OP_CLEAR, VK_ATTACHMENT_STORE_OP_STORE});

		std::vector<SubpassInfo> subPassInfos;
		subPassInfos.emplace_back(SubpassInfo{{}, {}, {}, false, 0, VK_RESOLVE_MODE_NONE});

		std::vector<Attachment> attachments;
		attachments.emplace_back(Attachment{shadowImageView->get_format(), shadowImage->get_sample_count(), shadowImage->get_usage()});

		std::vector<ImageView *> imageViews;
		imageViews.push_back(shadowImageView.get());

		auto &render_pass  = device->get_resource_cache().request_render_pass(attachments, loadStoreInfos, subPassInfos);
		auto &frame_buffer = device->get_resource_cache().request_framebuffer(imageViews, render_pass);
		commandBuffer.begin_render_pass(shadowExtent, render_pass, frame_buffer, oneClearValue);

		shadowPass->draw(commandBuffer, light_camera, &shadowNodes);

		commandBuffer.end_render_pass();
	}

	set_viewport_and_scissor(commandBuffer, extent);

	// depth pre pass
	{
		std::vector<LoadStoreInfo> loadStoreInfos;
		loadStoreInfos.emplace_back(LoadStoreInfo{VK_ATTACHMENT_LOAD_OP_CLEAR, VK_ATTACHMENT_STORE_OP_STORE});

		std::vector<SubpassInfo> subPassInfos;
		subPassInfos.emplace_back(SubpassInfo{{}, {}, {}, false, 0, VK_RESOLVE_MODE_NONE});

		std::vector<Attachment> attachments;
		attachments.emplace_back(Attachment{sceneDepthImageView->get_format(), sceneDepthImage->get_sample_count(), sceneDepthImage->get_usage()});

		std::vector<ImageView *> imageViews;
		imageViews.push_back(sceneDepthImageView.get());

		auto &render_pass  = device->get_resource_cache().request_render_pass(attachments, loadStoreInfos, subPassInfos);
		auto &frame_buffer = device->get_resource_cache().request_framebuffer(imageViews, render_pass);
		commandBuffer.begin_render_pass(extent, render_pass, frame_buffer, zeroClearValue);

		depthPrePass->draw(commandBuffer, camera, &opaqueNodes);

		commandBuffer.end_render_pass();
	}

	process_shadow(commandBuffer);
	process_light_buffer(commandBuffer);

	{
		// render main scene
		ImageMemoryBarrier barrier{};
		barrier.src_stage_mask  = VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT;
		barrier.dst_stage_mask  = VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT;
		barrier.src_access_mask = VK_ACCESS_SHADER_WRITE_BIT;
		barrier.dst_access_mask = VK_ACCESS_SHADER_READ_BIT;
		barrier.old_layout      = VK_IMAGE_LAYOUT_GENERAL;
		barrier.new_layout      = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
		commandBuffer.image_memory_barrier(*screenShadowImageView, barrier);

		barrier.src_stage_mask  = VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT;
		barrier.dst_stage_mask  = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
		barrier.src_access_mask = VK_ACCESS_SHADER_READ_BIT;
		barrier.dst_access_mask = VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT;
		barrier.old_layout      = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
		barrier.new_layout      = VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL;
		commandBuffer.image_memory_barrier(*sceneDepthImageView, barrier);

		std::vector<LoadStoreInfo> loadStoreInfos;
		loadStoreInfos.emplace_back(LoadStoreInfo{VK_ATTACHMENT_LOAD_OP_CLEAR, VK_ATTACHMENT_STORE_OP_STORE});
		loadStoreInfos.emplace_back(LoadStoreInfo{VK_ATTACHMENT_LOAD_OP_CLEAR, VK_ATTACHMENT_STORE_OP_DONT_CARE});

		std::vector<SubpassInfo> subPassInfos;
		subPassInfos.emplace_back(SubpassInfo{{}, {0}, {}, false, 0, VK_RESOLVE_MODE_NONE});

		std::vector<Attachment> attachments;
		attachments.emplace_back(Attachment{hdrColorImageView->get_format(), hdrColorImage->get_sample_count(), hdrColorImage->get_usage()});
		attachments.emplace_back(Attachment{sceneDepthImageView->get_format(), sceneDepthImage->get_sample_count(), sceneDepthImage->get_usage()});

		std::vector<ImageView *> imageViews;
		imageViews.push_back(hdrColorImageView.get());
		imageViews.push_back(sceneDepthImageView.get());

		auto &renderPass  = device->get_resource_cache().request_render_pass(attachments, loadStoreInfos, subPassInfos);
		auto &frameBuffer = device->get_resource_cache().request_framebuffer(imageViews, renderPass);

		commandBuffer.begin_render_pass(extent, renderPass, frameBuffer, colorClearValue);

		opaquePass->screenShadow = screenShadowImageView.get();
		opaquePass->draw(commandBuffer, camera, &opaqueNodes);
#ifdef GenTestSH
		DrawSHShpere(*render_context, commandBuffer, camera, sphere_mesh.get(), (glm::vec4 *) s_shWeight);
#endif
		opaquePass->draw_sky(commandBuffer, sphere_mesh.get());

		commandBuffer.end_render_pass();
	}

	process_HDR(commandBuffer);

	{
		// gui
		std::vector<LoadStoreInfo> loadStoreInfos;
		loadStoreInfos.emplace_back(LoadStoreInfo{VK_ATTACHMENT_LOAD_OP_LOAD, VK_ATTACHMENT_STORE_OP_STORE});

		std::vector<SubpassInfo> subPassInfos;
		subPassInfos.emplace_back(SubpassInfo{{}, {0}, {}, false, 0, VK_RESOLVE_MODE_NONE});

		auto &renderTarget = render_context->get_active_frame().get_render_target();
		auto &renderPass   = device->get_resource_cache().request_render_pass(renderTarget.get_attachments(), loadStoreInfos, subPassInfos);
		auto &frameBuffer  = device->get_resource_cache().request_framebuffer(renderTarget, renderPass);

		commandBuffer.begin_render_pass(renderTarget, renderPass, frameBuffer, colorClearValue);
		gui->draw(commandBuffer);
		commandBuffer.end_render_pass();
	}

	{
		auto &swapchainView = render_context->get_active_frame().get_render_target().get_views()[0];
		// Presentation
		ImageMemoryBarrier barrier{};
		barrier.src_stage_mask  = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
		barrier.dst_stage_mask  = VK_PIPELINE_STAGE_TRANSFER_BIT;
		barrier.src_access_mask = VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT;
		barrier.dst_access_mask = VK_ACCESS_TRANSFER_READ_BIT;
		barrier.old_layout      = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
		barrier.new_layout      = VK_IMAGE_LAYOUT_PRESENT_SRC_KHR;
		commandBuffer.image_memory_barrier(swapchainView, barrier);
	}

	commandBuffer.end();
	context.submit(commandBuffer);
}

void forward_plus::update(float delta_time)
{
	update_scene(delta_time);
	update_gui(delta_time);
	render(delta_time);
}

void forward_plus::draw_gui()
{
	VulkanSample::draw_gui();

	bool windowVisible = ImGui::Begin("Options", nullptr, ImGuiWindowFlags_MenuBar);
	if (!windowVisible)
	{
		ImGui::End();
		return;
	}

	ImGui::Text("Draw Options:");
	ImGui::Checkbox("Show Depth", &debugDepth);
	ImGui::Checkbox("Draw AABB", &drawAABB);
	ImGui::Checkbox("Draw Light Outline", &drawLight);
	ImGui::DragFloat("Exposure", &exposureValue, 0.1, 0.0, 10.0);
	ImGui::End();
}

void forward_plus::finish()
{
	VulkanSample::finish();
	GraphicContext::Release();
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

void forward_plus::full_screen_draw(vkb::CommandBuffer &command_buffer, vkb::core::ImageView &target, std::function<void(vkb::CommandBuffer &, vkb::RenderContext &)> body)
{
	std::vector<LoadStoreInfo> loadStoreInfos;
	loadStoreInfos.emplace_back(LoadStoreInfo{VK_ATTACHMENT_LOAD_OP_DONT_CARE, VK_ATTACHMENT_STORE_OP_STORE});

	std::vector<SubpassInfo> subPassInfos;
	subPassInfos.emplace_back(SubpassInfo{{}, {0}, {}, false, 0, VK_RESOLVE_MODE_NONE});

	std::vector<Attachment> attachments;
	attachments.emplace_back(Attachment{target.get_format(), target.get_image().get_sample_count(), target.get_image().get_usage()});

	std::vector<ImageView *> imageViews;
	imageViews.push_back(&target);

	auto &     image = target.get_image();
	VkExtent2D extent{image.get_extent().width, image.get_extent().height};

	auto &renderPass  = device->get_resource_cache().request_render_pass(attachments, loadStoreInfos, subPassInfos);
	auto &frameBuffer = device->get_resource_cache().request_framebuffer(imageViews, renderPass);

    VkViewport viewport{0.0, 0.0, (float)extent.width, (float)extent.height, 0.0, 1.0};
	command_buffer.set_viewport(0, {viewport});
	command_buffer.set_scissor(0, {VkRect2D{VkOffset2D{0, 0}, extent}});
	std::vector<VkClearValue> colorClearValue{initializers::clear_color_value(0.0, 0.0, 0.0, 0.0)};
	command_buffer.begin_render_pass(extent, renderPass, frameBuffer, colorClearValue);

	PipelineState     default_pipeline_state;
	DepthStencilState postProcessDepthState;        // depth test/write off
	postProcessDepthState.depth_test_enable  = false;
	postProcessDepthState.depth_write_enable = false;
	postProcessDepthState.depth_compare_op   = VK_COMPARE_OP_ALWAYS;

	ColorBlendState           defaultColorState;
	ColorBlendAttachmentState defaultAttaState;
	defaultColorState.attachments.push_back(defaultAttaState);

	VertexInputState vertexInputState;
	vertexInputState.bindings.emplace_back(VkVertexInputBindingDescription{0, sizeof(float) * 3, VK_VERTEX_INPUT_RATE_VERTEX});
	vertexInputState.attributes.emplace_back(VkVertexInputAttributeDescription{0, 0, VK_FORMAT_R32G32B32_SFLOAT, 0});

	command_buffer.set_color_blend_state(defaultColorState);
	command_buffer.set_depth_stencil_state(postProcessDepthState);
	command_buffer.set_rasterization_state(default_pipeline_state.get_rasterization_state());
	command_buffer.set_viewport_state(default_pipeline_state.get_viewport_state());
	command_buffer.set_multisample_state(default_pipeline_state.get_multisample_state());

	body(command_buffer, *render_context);

	command_buffer.set_vertex_input_state(vertexInputState);
	command_buffer.bind_vertex_buffers(0, {*postProcessVB}, {0});
	command_buffer.draw(3, 1, 0, 0);
	command_buffer.end_render_pass();
}

void forward_plus::input_event(const vkb::InputEvent &input_event)
{
	VulkanSample::input_event(input_event);
}

std::unique_ptr<vkb::Application> create_forward_plus()
{
	return std::make_unique<forward_plus>();
}
