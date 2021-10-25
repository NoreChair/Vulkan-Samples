#pragma once

#include "render_utils.h"
#include "shadow_camera.h"

#include "core/pipeline.h"
#include "rendering/render_context.h"
#include "scene_graph/node.h"
#include "scene_graph/components/camera.h"
#include "scene_graph/components/sub_mesh.h"

namespace MainPass {
    void Init(vkb::Device& device);
    void Draw(vkb::RenderContext& context, vkb::CommandBuffer& commandBuffer, vkb::sg::Camera* camera, ShadowCamera* shadowCamera, RenderUtils::SortedMeshes *submeshs, vkb::sg::Scene* scene);
    void DrawSky(vkb::RenderContext& context, vkb::CommandBuffer& commandBuffer, vkb::sg::Camera* camera, vkb::sg::SubMesh* skyMesh);
}