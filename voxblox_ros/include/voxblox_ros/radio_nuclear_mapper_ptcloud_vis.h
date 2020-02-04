/// Plagiarism Notice:
/// The code in this file comes from file pfcloud_vis.h
/// (in particular from functions visualizeIntensityVoxels and createIntensityPointcloudFromIntensityLayer)
/// and has been adapted for the special purpose of radiation mapping by Robert Hülsmann.
/// New/edited lines of code are marked with the comment "RH" (except simple renaming "intensity" to "radiation" etc.).

#ifndef VOXBLOX_ROS_RADIO_NUCLEAR_MAPPER_PTCLOUD_VIS_H_
#define VOXBLOX_ROS_RADIO_NUCLEAR_MAPPER_PTCLOUD_VIS_H_

#include "voxblox_ros/ptcloud_vis.h"

namespace voxblox {

  inline bool visualizeRadiationVoxels(const RadiationVoxel& voxel,
                                       const Point& /*coord*/,
                                       double* intensity) {
    constexpr float kMinWeight = 1e-3;

    CHECK_NOTNULL(intensity);
    if (voxel.distance > kMinWeight) {
      *intensity = voxel.intensity;
      return true;
    }
    return false;
  }

  inline void createRadiationPointcloudFromRadiationLayer(
      const Layer<RadiationVoxel>& layer,
      pcl::PointCloud<pcl::PointXYZI>* pointcloud) {
    CHECK_NOTNULL(pointcloud);
    createColorPointcloudFromLayer<RadiationVoxel>(
        layer, &visualizeRadiationVoxels, pointcloud);
  }
}  // namespace voxblox

#endif  // VOXBLOX_ROS_RADIO_NUCLEAR_MAPPER_PTCLOUD_VIS_H_