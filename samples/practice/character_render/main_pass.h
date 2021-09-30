#pragma once

#include "render_utils.h"

#include "core/pipeline.h"
#include "rendering/render_context.h"
#include "scene_graph/node.h"
#include "scene_graph/components/camera.h"
#include "scene_graph/components/sub_mesh.h"

namespace MainPass {
    void Init(vkb::Device& device);
    void Draw(vkb::RenderContext& context, vkb::CommandBuffer& commandBuffer, vkb::sg::Camera* camera, RenderUtils::SortedMeshes *submeshs, vkb::sg::Scene* scene);
}