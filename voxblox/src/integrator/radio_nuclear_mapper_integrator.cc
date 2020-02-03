#include "voxblox/integrator/radio_nuclear_mapper_integrator.h"

/// The code in class RadioNuclearMapperIntegrator comes from class IntensityIntegrator
/// and has been adapted for the special purpose by me.
/// New variables or methods are marked with the comment "RH" (Robert Hülsmann) IN HEADER FILE.
/// IN ADDITION, new lines of code in adopted methods are marked in the same way IN THIS FILE.

namespace voxblox {
  RadioNuclearMapperIntegrator::RadioNuclearMapperIntegrator(const Layer<TsdfVoxel>& tsdf_layer,
                                                             Layer<IntensityVoxel>* intensity_layer)
      : max_distance_(5.0),
        intensity_prop_voxel_radius_(2),
        tsdf_layer_(tsdf_layer),
        intensity_layer_(intensity_layer){}

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
          &surface_intersection, &distance);  /// (RH: distance added)

      if (!success) {
        continue;
      }

      // Now look up the matching voxels in the intensity layer and mark them.
      // Let's just start with 1.
      Block<IntensityVoxel>::Ptr block_ptr =
          intensity_layer_->allocateBlockPtrByCoordinates(surface_intersection);
      IntensityVoxel& voxel =
          block_ptr->getVoxelByCoordinates(surface_intersection);

      /// Update intensity and confidence if needed (RH)
      updateIntensityVoxel(voxel, radiation_sensor_value, distance);

      // Now check the surrounding voxels along the bearing vector. If they have
      // never been observed, then fill in their value. Otherwise don't.
      Point close_voxel = surface_intersection;
      for (int voxel_offset = -intensity_prop_voxel_radius_;
           voxel_offset <= intensity_prop_voxel_radius_; voxel_offset++) {
        close_voxel =
            surface_intersection + bearing_vectors[i] * voxel_offset * voxel_size;
        Block<IntensityVoxel>::Ptr new_block_ptr =
            intensity_layer_->allocateBlockPtrByCoordinates(close_voxel);
        IntensityVoxel& new_voxel = block_ptr->getVoxelByCoordinates(close_voxel);

        /// Update intensity and weight if needed (RH)
        updateIntensityVoxel(new_voxel, radiation_sensor_value, distance);
      }
    }
  }

  void RadioNuclearMapperIntegrator::updateIntensityVoxel(voxblox::IntensityVoxel& voxel,
                                                              const float in_intensity, const float in_distance){
    /// Using weight property of a voxel as distance value
    /// If distance is lower (so confidence is higher) than the stored one, update weight and intensity
    /// If distance is equal to the stored one, update intensity, if new one is higher (pessimistic assumption)
    if (in_distance < voxel.weight) {
      voxel.weight = in_distance;
      voxel.intensity = in_intensity;
    } else if (in_distance == voxel.weight){
      if (in_intensity > voxel.intensity) {
        voxel.intensity = in_intensity;
      }
    }
  }
}  // namespace voxblox
