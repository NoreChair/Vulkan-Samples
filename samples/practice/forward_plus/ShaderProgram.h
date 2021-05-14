#pragma once

#include "Utils.h"
#include "common/vk_common.h"
#include "core/shader_module.h"
#include <initializer_list>
#include <unordered_map>

class ShaderProgram
{
  public:
	ShaderProgram(vkb::ShaderModule *shader);

	ShaderProgram(std::initializer_list<vkb::ShaderModule *> &&params);

	std::vector<vkb::ShaderModule *> &GetShaderModules();

	bool IsGraphicProgram();

	bool IsComputeProgram();

	static size_t AddShaderProgram(std::string &name, std::shared_ptr<ShaderProgram> &&program);

	static void AddShaderProgram(size_t uid, std::shared_ptr<ShaderProgram> &&program);

	static void RemoveShaderProgram(size_t uid);

	static ShaderProgram *Find(std::string &name);

	static ShaderProgram *Find(size_t uid);

	static vkb::ShaderSource &FindShaderSource(size_t uid);

	static vkb::ShaderSource &FindShaderSource(std::string &name);

	static size_t AddShaderSource(vkb::ShaderSource &&shaderSource);

  private:
	static std::unordered_map<size_t, std::shared_ptr<ShaderProgram>> shaderProgramPool;
	static std::unordered_map<size_t, vkb::ShaderSource>              shaderSourcePool;
	VkShaderStageFlagBits                                             programStageFlag;
	std::vector<vkb::ShaderModule *>                                  shaderModules;
};