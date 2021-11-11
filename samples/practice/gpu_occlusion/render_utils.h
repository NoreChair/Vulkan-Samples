#pragma once

#include "core/device.h"
#include "core/command_buffer.h"
#include "scene_graph/scene.h"
#include "scene_graph/components/image.h"
#include "scene_graph/components/mesh.h"
#include "scene_graph/components/sub_mesh.h"

#include <map>
namespace RenderUtils{
    typedef std::multimap<float, std::pair<vkb::sg::Node *, vkb::sg::SubMesh *>> SortedMeshes;

    glm::mat4 VulkanStyleProjection(const glm::mat4 &proj);

    void GetSortedNodes(vkb::sg::Scene* scene, glm::vec3 direction, glm::vec3 position, SortedMeshes* opaque_nodes, SortedMeshes* transparent_nodes = nullptr, SortedMeshes* cull_off_nodes = nullptr);
    void GetSortedNodes(const std::vector<vkb::sg::Mesh*>& meshes, glm::vec3 direction, glm::vec3 position, SortedMeshes* opaque_nodes, SortedMeshes* transparent_nodes = nullptr, SortedMeshes* cull_off_nodes = nullptr);

    void UploadImageToGPU(vkb::CommandBuffer &command_buffer, vkb::core::Buffer &staging_buffer, vkb::sg::Image &image);
    void UploadImage(vkb::Device& device, std::vector<std::unique_ptr<vkb::sg::Image>>& imags, bool clearHostMemory = true);

    void BindPipelineState(vkb::CommandBuffer &comman_buffer, vkb::PipelineState &pipeline);
    bool BindVertexInput(vkb::CommandBuffer &command_buffer, vkb::PipelineLayout &pipeline_layout, vkb::sg::SubMesh *submesh);

    void ExtractPlanes(glm::mat4 view_project, glm::vec4* planes);
    bool FrustumAABB(const glm::vec4 planes[6], glm::vec3 min, glm::vec3 max);
}