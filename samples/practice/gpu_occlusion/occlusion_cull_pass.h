#pragma once
#include "render_utils.h"

#include "core/pipeline.h"
#include "rendering/render_context.h"
#include "scene_graph/node.h"
#include "scene_graph/components/camera.h"
#include "scene_graph/components/sub_mesh.h"
#include "scene_graph/components/aabb.h"

namespace OcclusionCullPass {
    void Init(vkb::Device& device);
    void BeginPass(vkb::RenderContext& context, vkb::CommandBuffer& commandBuffer);
    void DrawModel(vkb::RenderContext& context, vkb::CommandBuffer& commandBuffer, vkb::sg::Camera* camera, RenderUtils::SortedMeshes *subMeshes, bool query);
    void DrawProxy(vkb::RenderContext& context, vkb::CommandBuffer& commandBuffer, vkb::sg::Camera* camera, vkb::sg::SubMesh* proxy, RenderUtils::SortedMeshes *subMeshes, bool query);
    int EndPass(vkb::CommandBuffer& commandBuffer);

}
