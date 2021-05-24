#include "screen_shadow_pass.h"
#include "common/vk_initializers.h"
#include "scene_graph/components/perspective_camera.h"

screen_shadow_pass::screen_shadow_pass(vkb::Device *d, vkb::RenderContext *ctx) :
    compute_pass(d),
    context(ctx)
{
}

void screen_shadow_pass::prepare(std::vector<vkb::ShaderModule *> &shader_module)
{
	compute_pass::prepare(shader_module);

	VkSamplerCreateInfo samplerInfo{};
	samplerInfo.magFilter    = VK_FILTER_LINEAR;
	samplerInfo.minFilter    = VK_FILTER_LINEAR;
	samplerInfo.mipmapMode   = VK_SAMPLER_MIPMAP_MODE_LINEAR;
	samplerInfo.addressModeU = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
	samplerInfo.addressModeV = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
	samplerInfo.addressModeW = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
	samplerInfo.minLod       = 0;
	samplerInfo.maxLod       = VK_LOD_CLAMP_NONE;
	linearClampSampler       = std::make_unique<vkb::core::Sampler>(*device, samplerInfo);

	samplerInfo.compareEnable = VK_TRUE;
	samplerInfo.compareOp     = VK_COMPARE_OP_LESS;
	shadowSampler             = std::make_unique<vkb::core::Sampler>(*device, samplerInfo);
}

void screen_shadow_pass::set_up(vkb::core::ImageView *depth, vkb::core::ImageView *shadowMap, vkb::core::ImageView *target)
{
	uavs[0] = depth;
	uavs[1] = shadowMap;
	uavs[2] = target;
}

void screen_shadow_pass::set_camera(vkb::sg::Camera *camera, shadow_camera *light_camera)
{
	main_camera        = camera;
	this->light_camera = light_camera;
}

void screen_shadow_pass::dispatch(vkb::CommandBuffer &command_buffer)
{
	struct
	{
		glm::vec4 extent;
		glm::vec4 shadowExtent;
		glm::mat4 invMainMatrix;
		glm::mat4 shadowMatrix;
	} uniforms;

	VkExtent3D imageSize       = uavs[2]->get_image().get_extent();
	VkExtent3D shadowImageSize = uavs[1]->get_image().get_extent();
	uniforms.extent            = glm::vec4(imageSize.width, imageSize.height, 1.0f / imageSize.width, 1.0f / imageSize.height);
	uniforms.shadowExtent      = glm::vec4(shadowImageSize.width, shadowImageSize.height, 1.0f / shadowImageSize.width, 1.0f / shadowImageSize.height);

	auto mainProj = main_camera->get_projection();
	mainProj[1][1] *= -1.0;
	auto lightProj = light_camera->get_projection();
	lightProj[1][1] *= -1.0;

	uniforms.invMainMatrix = glm::inverse(mainProj * main_camera->get_view());
	uniforms.shadowMatrix  = lightProj * light_camera->get_view();

	auto allocation = context->get_active_frame().allocate_buffer(VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT, sizeof(uniforms));
	allocation.update(uniforms);

	bind_pipeline(command_buffer);

	command_buffer.bind_buffer(allocation.get_buffer(), allocation.get_offset(), allocation.get_size(), 0, 0, 0);
	command_buffer.bind_image(*uavs[0], *linearClampSampler, 0, 1, 0);
	command_buffer.bind_image(*uavs[1], *shadowSampler, 0, 2, 0);
	command_buffer.bind_image(*uavs[2], 0, 3, 0);

	command_buffer.dispatch((uint32_t) glm::ceil(imageSize.width / 16.0f), (uint32_t) glm::ceil(imageSize.height / 16.0f), 1);
}
