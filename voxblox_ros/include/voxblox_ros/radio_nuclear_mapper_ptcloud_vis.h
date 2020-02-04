/// Plagiarism Notice:
/// The code in this file comes from file pfcloud_vis.h
/// (in particular from functions visualizeIntensityVoxels and createIntensityPointcloudFromIntensityLayer)
/// and has been adapted for the special purpose of radiation mapping by Robert Hülsmann.
/// New/edited lines of code are marked with the comment "RH"

#ifndef VOXBLOX_ROS_RADIO_NUCLEAR_MAPPER_PTCLOUD_VIS_H_ /// RH
#define VOXBLOX_ROS_RADIO_NUCLEAR_MAPPER_PTCLOUD_VIS_H_ /// RH

#include "voxblox_ros/ptcloud_vis.h" /// RH

namespace voxblox {

  inline bool visualizeRadiationVoxels(const RadiationVoxel& voxel, /// RH
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

  inline void createRadiationPointcloudFromRadiationLayer( /// RH
      const Layer<RadiationVoxel>& layer, /// RH
      pcl::PointCloud<pcl::PointXYZI>* pointcloud) {
    CHECK_NOTNULL(pointcloud);
    createColorPointcloudFromLayer<RadiationVoxel>( /// RH
        layer, &visualizeRadiationVoxels, pointcloud); /// RH
  }
}  // namespace voxblox

#endif  // VOXBLOX_ROS_RADIO_NUCLEAR_MAPPER_PTCLOUD_VIS_H_ /// RH