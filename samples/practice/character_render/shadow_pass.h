#pragma once

#include "core/pipeline.h"
#include "rendering/render_context.h"
#include "scene_graph/node.h"
#include "scene_graph/components/camera.h"
#include "scene_graph/components/sub_mesh.h"

namespace ShadowPass {
    extern std::unique_ptr<vkb::GraphicsPipeline> g_shadowPipeline;

    void Init();
    void Draw(vkb::RenderContext& context, vkb::sg::Camera* camera, std::multimap<float, std::pair<vkb::sg::Node *, vkb::sg::SubMesh *>> *submeshs);
}