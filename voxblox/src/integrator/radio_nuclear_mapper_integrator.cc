/// Plagiarism Notice:
/// The code in this file comes from file intensity_integrator.cc
/// and has been adapted for the special purpose of radiation mapping by Robert Hülsmann.
/// New/edited variables or methods of class RadioNuclearMapperIntegrator are marked with comment "RH" IN HEADER FILE.
/// IN ADDITION, new/edited lines of code in adopted methods are marked in the same way IN THIS FILE
/// (except simple renaming "intensity" to "radiation" etc.).

#include "voxblox/integrator/radio_nuclear_mapper_integrator.h"

namespace voxblox {
  RadioNuclearMapperIntegrator::RadioNuclearMapperIntegrator(const Layer<TsdfVoxel>& tsdf_layer,
                                                             Layer<RadiationVoxel>* radiation_layer)
      : max_distance_(5.0),
        radiation_prop_voxel_radius_(2),
        tsdf_layer_(tsdf_layer),
        radiation_layer_(radiation_layer){}

  void RadioNuclearMapperIntegrator::addRadiationSensorValueBearingVectors(const Point& origin,
                                                                           const Pointcloud& bearing_vectors,
                                                                           const float radiation_sensor_value){
    timing::Timer intensity_timer("intensity/integrate");

    const FloatingPoint voxel_size = tsdf_layer_.voxel_size();

    for (size_t i = 0; i < bearing_vectors.size(); ++i) {
      Point surface_intersection = Point::Zero();
      FloatingPoint distance = 0.0; /// RH
      // Cast ray from the origin in the direction of the bearing vector until
      // finding an intersection with a surface.
      bool success = getSurfaceDistanceAlongRay<TsdfVoxel>(
          tsdf_layer_, origin, bearing_vectors[i], max_distance_,
          &surface_intersection, distance);  /// RH: distance added

      if (!success) {
        continue;
      }

      // Now look up the matching voxels in the intensity layer and mark them.
      // Let's just start with 1.
      Block<RadiationVoxel>::Ptr block_ptr =
          radiation_layer_->allocateBlockPtrByCoordinates(surface_intersection);
      RadiationVoxel& voxel =
          block_ptr->getVoxelByCoordinates(surface_intersection);

      /// Update intensity and confidence if needed (RH)
      updateRadiationVoxel(voxel, radiation_sensor_value, distance);

      // Now check the surrounding voxels along the bearing vector. If they have
      // never been observed, then fill in their value. Otherwise don't.
      Point close_voxel = surface_intersection;
      for (int voxel_offset = -radiation_prop_voxel_radius_;
           voxel_offset <= radiation_prop_voxel_radius_; voxel_offset++) {
        close_voxel =
            surface_intersection + bearing_vectors[i] * voxel_offset * voxel_size;
        Block<RadiationVoxel>::Ptr new_block_ptr =
            radiation_layer_->allocateBlockPtrByCoordinates(close_voxel);
        RadiationVoxel& new_voxel = block_ptr->getVoxelByCoordinates(close_voxel);

        /// Update intensity and weight if needed (RH)
        updateRadiationVoxel(new_voxel, radiation_sensor_value, distance);
      }
    }
  }

  void RadioNuclearMapperIntegrator::updateRadiationVoxel(voxblox::RadiationVoxel& voxel,
                                                              const float in_intensity, const float in_distance){
    /// If distance is lower (so confidence is higher) than the stored one, update distance and intensity
    /// If distance is equal to the stored one, update intensity, if new one is higher (pessimistic assumption)
    if (in_distance < voxel.distance) {
      voxel.distance = in_distance;
      voxel.intensity = in_intensity;
    } else if (in_distance == voxel.distance){
      if (in_intensity > voxel.intensity) {
        voxel.intensity = in_intensity;
      }
    }
  }
}  // namespace voxblox
