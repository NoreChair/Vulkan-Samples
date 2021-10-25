#pragma once

#include "render_utils.h"
#include "shadow_camera.h"

#include "core/pipeline.h"
#include "rendering/render_context.h"
#include "scene_graph/node.h"
#include "scene_graph/components/sub_mesh.h"

namespace ShadowPass {
    void Init(vkb::Device& device);
    void Draw(vkb::RenderContext& context, vkb::CommandBuffer& commandBuffer, ShadowCamera* camera, RenderUtils::SortedMeshes *submeshs);
}