#ifndef VOXBLOX_ROS_RADIO_NUCLEAR_MAPPER_VIS_H_
#define VOXBLOX_ROS_RADIO_NUCLEAR_MAPPER_VIS_H_

#include <memory>

#include <voxblox/utils/color_maps.h>
#include <voxblox_msgs/Mesh.h>

#include "voxblox_ros/conversions.h"
#include "voxblox_ros/mesh_vis.h"

namespace voxblox {
  Color getColorForVoxelPointer(const IntensityVoxel* voxel, const std::shared_ptr<ColorMap>& color_map){
    Color c;
    if (voxel != nullptr && voxel->weight > 0.0) {
      float intensity = voxel->intensity;
      //printf("Intensity: %f", intensity);
//          std::cout << "Intensity: " << intensity << std::endl;
      c = color_map->colorLookup(intensity);

    } else {
      c = color_map->colorLookup(0.0);
    }
    return c;
  }

  inline void recolorVoxbloxMeshMsgByRadioNuclearIntensity(
      const Layer<IntensityVoxel>& intensity_layer,
      const std::shared_ptr<ColorMap>& color_map,
      voxblox_msgs::Mesh* mesh_msg,
      Mesh* mesh,
      std::vector<Point>* mesh_points_) {
    CHECK_NOTNULL(mesh_msg);
    CHECK(color_map);

/*  size_t vps = intensity_layer.voxels_per_side();
  size_t nps = vps * vps * vps;

  BlockIndexList blocks;
  intensity_layer.getAllAllocatedBlocks(&blocks);
 for (const BlockIndex& index : blocks){
    const Block<IntensityVoxel>& block = intensity_layer.getBlockByIndex(index);
    for (size_t linear_index = 0; linear_index < nps; ++linear_index) {
      Point coord = block.computeCoordinatesFromLinearIndex(linear_index);
      // std::cout << "Block: x: " << coord.x() << "; y: " << coord.y() << "; z: " << coord.z() << std::endl;
      const IntensityVoxel* voxel = intensity_layer.getVoxelPtrByCoordinates(coord);
      if (voxel != nullptr) {
        //std::cout << "Voxel intensity: " << voxel->intensity << std::endl;
        std::cout << "Block: x: " << coord.x() << "; y: " << coord.y() << "; z: " << coord.z() << std::endl;
      }
    }
  }*/

    const size_t num_vertices_before = mesh->vertices.size();
    VertexIndex idx = 0;

    // Go over all the blocks in the mesh message.
    for (voxblox_msgs::MeshBlock& mesh_block : mesh_msg->mesh_blocks) {
      // Look up verticies in the thermal layer.
      for (size_t vert_idx = 0u; vert_idx < mesh_block.x.size(); ++vert_idx) {

        // only needed if color information was originally missing
/*      mesh_block.r.resize(mesh_block.x.size());
      mesh_block.g.resize(mesh_block.x.size());
      mesh_block.b.resize(mesh_block.x.size());

      const IntensityVoxel* voxel = intensity_layer.getVoxelPtrByCoordinates(
          Point(mesh_block.x[vert_idx], mesh_block.y[vert_idx],
                mesh_block.z[vert_idx]));*/

        constexpr float point_conv_factor = 2.0f / std::numeric_limits<uint16_t>::max();
        const float mesh_x =
            (static_cast<float>(mesh_block.x[vert_idx]) * point_conv_factor +
             static_cast<float>(mesh_block.index[0])) * mesh_msg->block_edge_length;
        const float mesh_y =
            (static_cast<float>(mesh_block.y[vert_idx]) * point_conv_factor +
             static_cast<float>(mesh_block.index[1])) * mesh_msg->block_edge_length;
        const float mesh_z =
            (static_cast<float>(mesh_block.z[vert_idx]) * point_conv_factor +
             static_cast<float>(mesh_block.index[2])) * mesh_msg->block_edge_length;
        Point p = Point(mesh_x, mesh_y, mesh_z);

        mesh_points_->push_back(p);

        const IntensityVoxel* voxel = intensity_layer.getVoxelPtrByCoordinates(p);

        //std::cout << "Voxel: x: " << mesh_x << std::endl;
//        std::cout << "Voxel: X: " << mesh_x << "; y: " << mesh_y << "; z: " << mesh_z << std::endl;

//        if (voxel == nullptr) {
//          //printf("voxel is null pointer!");
//        } else if (voxel->weight <= 0.0) {
////          std::cout << "weight too small" << std::endl;
//        }

        Color c = getColorForVoxelPointer(voxel, color_map);

        //Update mesh message color
        mesh_block.r[vert_idx] = c.r;
        mesh_block.g[vert_idx] = c.g;
        mesh_block.b[vert_idx] = c.b;

        //Add to mesh
        mesh->vertices.push_back(p);
        mesh->colors.push_back(c);
        mesh->normals.push_back(p);
        mesh->indices.push_back(num_vertices_before + idx++);
      }
    }
  }

}  // namespace voxblox

#endif  // VOXBLOX_ROS_RADIO_NUCLEAR_MAPPER_VIS_H_