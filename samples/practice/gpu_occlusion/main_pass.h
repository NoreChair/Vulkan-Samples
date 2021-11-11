#pragma once

#include "render_utils.h"

#include "core/pipeline.h"
#include "rendering/render_context.h"
#include "scene_graph/node.h"
#include "scene_graph/components/camera.h"
#include "scene_graph/components/sub_mesh.h"
#include "scene_graph/components/material.h"
#include "scene_graph/components/aabb.h"

namespace MainPass {
    void Init(vkb::Device& device);
    void BeginPass(vkb::RenderContext& context, vkb::CommandBuffer& commandBuffer);
    void DrawOpaque(vkb::RenderContext& context, vkb::CommandBuffer& commandBuffer, vkb::sg::Camera* camera, RenderUtils::SortedMeshes *submeshs, vkb::sg::Scene* scene);
    void DrawAlphaTest(vkb::RenderContext& context, vkb::CommandBuffer& commandBuffer, vkb::sg::Camera* camera, RenderUtils::SortedMeshes *submeshs, vkb::sg::Scene* scene);
    void DrawDebugAABB(vkb::RenderContext& context, vkb::CommandBuffer& commandBuffer, vkb::sg::Camera* camera, vkb::sg::SubMesh* debugMesh, const std::vector<std::reference_wrapper<const vkb::sg::AABB>>& aabbs);
    void DrawSky(vkb::RenderContext& context, vkb::CommandBuffer& commandBuffer, vkb::sg::Camera* camera, vkb::sg::SubMesh* skyMesh, vkb::sg::Material* material);
    void EndPass(vkb::CommandBuffer& commandBuffer);
}