#include "render_utils.h"
#include "scene_graph/node.h"
#include "scene_graph/components/aabb.h"
#include "scene_graph/components/mesh.h"
#include "scene_graph/components/material.h"

namespace RenderUtils {
    glm::mat4 VulkanStyleProjection(const glm::mat4 &proj) {
        // Flip Y in clipspace. X = -1, Y = -1 is topLeft in Vulkan.
        glm::mat4 mat = proj;
        mat[1][1] *= -1;

        return mat;
    }

    void UploadImageToGPU(vkb::CommandBuffer &command_buffer, vkb::core::Buffer &staging_buffer, vkb::sg::Image &image) {
        // Clean up the image data, as they are copied in the staging buffer
        image.clear_data();

        {
            vkb::ImageMemoryBarrier memory_barrier{};
            memory_barrier.old_layout = VK_IMAGE_LAYOUT_UNDEFINED;
            memory_barrier.new_layout = VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL;
            memory_barrier.src_access_mask = 0;
            memory_barrier.dst_access_mask = VK_ACCESS_TRANSFER_WRITE_BIT;
            memory_barrier.src_stage_mask = VK_PIPELINE_STAGE_HOST_BIT;
            memory_barrier.dst_stage_mask = VK_PIPELINE_STAGE_TRANSFER_BIT;

            command_buffer.image_memory_barrier(image.get_vk_image_view(), memory_barrier);
        }

        // Create a buffer image copy for every mip level
        auto &mipmaps = image.get_mipmaps();

        std::vector<VkBufferImageCopy> buffer_copy_regions(mipmaps.size());

        for (size_t i = 0; i < mipmaps.size(); ++i) {
            auto &mipmap = mipmaps[i];
            auto &copy_region = buffer_copy_regions[i];

            copy_region.bufferOffset = mipmap.offset;
            copy_region.imageSubresource = image.get_vk_image_view().get_subresource_layers();
            // Update miplevel
            copy_region.imageSubresource.mipLevel = mipmap.level;
            copy_region.imageExtent = mipmap.extent;
        }

        command_buffer.copy_buffer_to_image(staging_buffer, image.get_vk_image(), buffer_copy_regions);

        {
            vkb::ImageMemoryBarrier memory_barrier{};
            memory_barrier.old_layout = VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL;
            memory_barrier.new_layout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
            memory_barrier.src_access_mask = VK_ACCESS_TRANSFER_WRITE_BIT;
            memory_barrier.dst_access_mask = VK_ACCESS_SHADER_READ_BIT;
            memory_barrier.src_stage_mask = VK_PIPELINE_STAGE_TRANSFER_BIT;
            memory_barrier.dst_stage_mask = VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT;

            command_buffer.image_memory_barrier(image.get_vk_image_view(), memory_barrier);
        }
    }

    void GetSortedNodes(vkb::sg::Scene* scene, glm::vec3 direction, glm::vec3 position, SortedMeshes* opaque_nodes, SortedMeshes* transparent_nodes, SortedMeshes* cull_off_nodes) {
        const auto meshes = scene->get_components<vkb::sg::Mesh>();
        for (auto &mesh : meshes) {
            for (auto &node : mesh->get_nodes()) {
                auto node_transform = node->get_transform().get_world_matrix();

                const vkb::sg::AABB &mesh_bounds = mesh->get_bounds();

                vkb::sg::AABB world_bounds{mesh_bounds.get_min(), mesh_bounds.get_max()};
                world_bounds.transform(node_transform);

                glm::vec3 delta = world_bounds.get_center() - position;
                float     distance = glm::dot(direction, delta);

                for (auto &sub_mesh : mesh->get_submeshes()) {
                    if (sub_mesh->get_material()->alpha_mode == vkb::sg::AlphaMode::Blend && transparent_nodes != nullptr) {
                        transparent_nodes->emplace(distance, std::make_pair(node, sub_mesh));
                    } else if(sub_mesh->get_material()->alpha_mode == vkb::sg::AlphaMode::Mask && cull_off_nodes != nullptr){
                        cull_off_nodes->emplace(distance, std::make_pair(node, sub_mesh));
                    } else {
                        opaque_nodes->emplace(distance, std::make_pair(node, sub_mesh));
                    }
                }
            }
        }
    }

    void GetSortedNodes(const std::vector<vkb::sg::Mesh*>& meshes, glm::vec3 direction, glm::vec3 position, SortedMeshes * opaque_nodes, SortedMeshes * transparent_nodes, SortedMeshes * cull_off_nodes) {
        for (auto &mesh : meshes) {
            for (auto &node : mesh->get_nodes()) {
                auto node_transform = node->get_transform().get_world_matrix();

                const vkb::sg::AABB &mesh_bounds = mesh->get_bounds();

                vkb::sg::AABB world_bounds{mesh_bounds.get_min(), mesh_bounds.get_max()};
                world_bounds.transform(node_transform);

                glm::vec3 delta = world_bounds.get_center() - position;
                float     distance = glm::dot(direction, delta);

                for (auto &sub_mesh : mesh->get_submeshes()) {
                    if (sub_mesh->get_material()->alpha_mode == vkb::sg::AlphaMode::Blend && transparent_nodes != nullptr) {
                        transparent_nodes->emplace(distance, std::make_pair(node, sub_mesh));
                    } else if (sub_mesh->get_material()->alpha_mode == vkb::sg::AlphaMode::Mask && cull_off_nodes != nullptr) {
                        cull_off_nodes->emplace(distance, std::make_pair(node, sub_mesh));
                    } else {
                        opaque_nodes->emplace(distance, std::make_pair(node, sub_mesh));
                    }
                }
            }
        }
    }

    void UploadImage(vkb::Device& device, std::vector<std::unique_ptr<vkb::sg::Image>>& imags, bool clearHostMemory) {
        // Upload images to GPU
        std::vector<vkb::core::Buffer> transient_buffers;

        auto &command_buffer = device.request_command_buffer();

        command_buffer.begin(VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT);

        for (size_t image_index = 0; image_index < imags.size(); image_index++) {
            auto &image = imags.at(image_index);

            vkb::core::Buffer stage_buffer{device,
                                      image->get_data().size(),
                                      VK_BUFFER_USAGE_TRANSFER_SRC_BIT,
                                      VMA_MEMORY_USAGE_CPU_ONLY};

            stage_buffer.update(image->get_data());

            UploadImageToGPU(command_buffer, stage_buffer, *image);

            transient_buffers.push_back(std::move(stage_buffer));
        }

        command_buffer.end();

        auto &queue = device.get_queue_by_flags(VK_QUEUE_GRAPHICS_BIT, 0);

        queue.submit(command_buffer, device.request_fence());

        device.get_fence_pool().wait();
        device.get_fence_pool().reset();
        device.get_command_pool().reset_pool();
        device.wait_idle();

        transient_buffers.clear();

        if (clearHostMemory) {
            for (size_t image_index = 0; image_index < imags.size(); image_index++) {
                auto &image = imags.at(image_index);
                image->clear_data();
            }
        }
    }

    void BindPipelineState(vkb::CommandBuffer &command_buffer, vkb::PipelineState &pipeline) {
        command_buffer.set_color_blend_state(pipeline.get_color_blend_state());
        command_buffer.set_depth_stencil_state(pipeline.get_depth_stencil_state());
        command_buffer.set_input_assembly_state(pipeline.get_input_assembly_state());
        command_buffer.set_rasterization_state(pipeline.get_rasterization_state());
        command_buffer.set_viewport_state(pipeline.get_viewport_state());
        command_buffer.set_multisample_state(pipeline.get_multisample_state());
        command_buffer.set_vertex_input_state(pipeline.get_vertex_input_state());
    }

    bool BindVertexInput(vkb::CommandBuffer &command_buffer, vkb::PipelineLayout &pipeline_layout, vkb::sg::SubMesh *submesh) {
        auto vertex_input_resources = pipeline_layout.get_resources(vkb::ShaderResourceType::Input, VK_SHADER_STAGE_VERTEX_BIT);

        vkb::VertexInputState vertex_input_state;

        for (auto &input_resource : vertex_input_resources) {
            vkb::sg::VertexAttribute attribute;

            if (!submesh->get_attribute(input_resource.name, attribute)) {
                continue;
            }

            VkVertexInputAttributeDescription vertex_attribute{};
            vertex_attribute.binding = input_resource.location;
            vertex_attribute.format = attribute.format;
            vertex_attribute.location = input_resource.location;
            vertex_attribute.offset = attribute.offset;

            vertex_input_state.attributes.push_back(vertex_attribute);

            VkVertexInputBindingDescription vertex_binding{};
            vertex_binding.binding = input_resource.location;
            vertex_binding.stride = attribute.stride;

            vertex_input_state.bindings.push_back(vertex_binding);
        }

        command_buffer.set_vertex_input_state(vertex_input_state);

        // Find submesh vertex buffers matching the shader input attribute names
        for (auto &input_resource : vertex_input_resources) {
            const auto &buffer_iter = submesh->vertex_buffers.find(input_resource.name);

            if (buffer_iter != submesh->vertex_buffers.end()) {
                std::vector<std::reference_wrapper<const vkb::core::Buffer>> buffers;
                buffers.emplace_back(std::ref(buffer_iter->second));

                // Bind vertex buffers only for the attribute locations defined
                command_buffer.bind_vertex_buffers(input_resource.location, std::move(buffers), {0});
            }
        }

        // Draw submesh indexed if indices exists
        if (submesh->vertex_indices != 0) {
            // Bind index buffer of submesh
            command_buffer.bind_index_buffer(*submesh->index_buffer, submesh->index_offset, submesh->index_type);
            return true;
        }
        return false;
    }

    void ExtractPlanes(glm::mat4 view_project, glm::vec4* planes) {
        planes[0] = view_project[3] + view_project[0];
        planes[1] = view_project[3] - view_project[0];
        planes[2] = view_project[3] + view_project[1];
        planes[3] = view_project[3] - view_project[1];
        planes[4] = view_project[3]; // DX/Vulkan like only, ogl not
        planes[5] = view_project[3] - view_project[2];
        for (int n = 0; n < 6; n++) {
            // normalize
            planes[n] *= 1.0 / glm::sqrt(glm::dot(glm::vec3(planes[n].xyz), glm::vec3(planes[n].xyz)));
        }
    }

    bool FrustumAABB(const glm::vec4 planes[6], glm::vec3 min, glm::vec3 max) {
        glm::vec4 center = glm::vec4((min + max) * 0.5f, 1.0f);
        glm::vec3 height = (max - min) * 0.5f;
        for (uint32_t i = 0; i < 6; i++) {
            float e = glm::dot(glm::vec3(planes[i].xyz), height * glm::sign(glm::vec3(planes[i].xyz)));
            if (glm::dot(planes[i], center) + e < 0.0f) {
                return false;
            }
        }
        return true;
    }
}