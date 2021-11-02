#include "ShaderProgram.h"

std::unordered_map<size_t, std::shared_ptr<ShaderProgram>> ShaderProgram::shaderProgramPool = std::unordered_map<size_t, std::shared_ptr<ShaderProgram>>();

std::unordered_map<size_t, vkb::ShaderSource> ShaderProgram::shaderSourcePool = std::unordered_map<size_t, vkb::ShaderSource>();

ShaderProgram::ShaderProgram(vkb::ShaderModule *shader)
{
	shaderModules.push_back(shader);
	programStageFlag = shader->get_stage();
}

ShaderProgram::ShaderProgram(std::initializer_list<vkb::ShaderModule *> &&params)
{
	programStageFlag = (VkShaderStageFlagBits) 0;
	for (auto ptr = params.begin(); ptr != params.end(); ++ptr)
	{
		shaderModules.push_back(*ptr);
		programStageFlag = (VkShaderStageFlagBits)((int) programStageFlag | (int) (*ptr)->get_stage());
	}
}

std::vector<vkb::ShaderModule *> &ShaderProgram::GetShaderModules()
{
	return shaderModules;
}

bool ShaderProgram::IsGraphicProgram()
{
	return (programStageFlag & VK_SHADER_STAGE_ALL_GRAPHICS) != 0;
}

bool ShaderProgram::IsComputeProgram()
{
	return (programStageFlag & VK_SHADER_STAGE_COMPUTE_BIT) != 0;
}

size_t ShaderProgram::AddShaderProgram(std::string name, std::shared_ptr<ShaderProgram> &&program)
{
	size_t uid = std::hash<std::string>{}(name);
	AddShaderProgram(uid, std::move(program));
	return uid;
}

void ShaderProgram::AddShaderProgram(size_t uid, std::shared_ptr<ShaderProgram> &&program)
{
	DEBUG_ASSERT(shaderProgramPool.find(uid) == shaderProgramPool.end(), "Shader program uid collision");
	shaderProgramPool.emplace(uid, std::move(program));
}

void ShaderProgram::RemoveShaderProgram(size_t uid)
{
	auto iter = shaderProgramPool.find(uid);
	if (iter != shaderProgramPool.end())
	{
		shaderProgramPool.erase(iter);
	}
}

ShaderProgram *ShaderProgram::Find(std::string name)
{
	size_t uid  = std::hash<std::string>{}(name);
	auto   iter = shaderProgramPool.find(uid);
	if (iter != shaderProgramPool.end())
	{
		return iter->second.get();
	}
	return nullptr;
}

ShaderProgram *ShaderProgram::Find(size_t uid)
{
	auto iter = shaderProgramPool.find(uid);
	if (iter != shaderProgramPool.end())
	{
		return iter->second.get();
	}
	return nullptr;
}

vkb::ShaderSource &ShaderProgram::FindShaderSource(size_t uid)
{
	auto iter = shaderSourcePool.find(uid);
	if (iter != shaderSourcePool.end())
	{
		return iter->second;
	}
	throw std::runtime_error("Can't find shader with UID : " + uid);
}

vkb::ShaderSource &ShaderProgram::FindShaderSource(std::string name)
{
	size_t uid  = std::hash<std::string>{}(name);
	auto   iter = shaderSourcePool.find(uid);
	if (iter != shaderSourcePool.end())
	{
		return iter->second;
	}
	throw std::runtime_error("Can't find shader with name : " + name);
}

size_t ShaderProgram::AddShaderSource(vkb::ShaderSource &&shaderSource)
{
	size_t uid = std::hash<std::string>{}(shaderSource.get_filename());
	DEBUG_ASSERT(shaderSourcePool.find(uid) == shaderSourcePool.end(), "Shader source uid collision");
	shaderSourcePool.emplace(uid, std::move(shaderSource));
	return uid;
}