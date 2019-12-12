#include "voxblox/integrator/radio_nuclear_mapper_integrator.h"

namespace voxblox {
  RadioNuclearMapperIntegrator::RadioNuclearMapperIntegrator(const Layer<TsdfVoxel>& tsdf_layer,
                                                             Layer<IntensityVoxel>* intensity_layer)
      : max_distance_(5.0),
        intensity_prop_voxel_radius_(2),
        tsdf_layer_(tsdf_layer),
        intensity_layer_(intensity_layer),
        allowed_distance_functions_({"increasing", "decreasing", "constant"}),
        dist_func_('z'){}

  void RadioNuclearMapperIntegrator::setDistanceFunction(const std::string distance_function) {
    if (distance_function == "increasing"){
      dist_func_ = 'i';
    }else if (distance_function == "decreasing"){
      dist_func_ = 'd';
    }else if (distance_function == "constant"){
      dist_func_ = 'c';
    }else{ // zero
      dist_func_ = 'z';
    }
    printf("dist_func_ = %c\n", dist_func_);
  }

  void RadioNuclearMapperIntegrator::addIntensityBearingVectors(const Point& origin, const Pointcloud& bearing_vectors,
                                                                const float intensity) {
    timing::Timer intensity_timer("intensity/integrate");
    const FloatingPoint voxel_size = tsdf_layer_.voxel_size();

    for (size_t i = 0; i < bearing_vectors.size(); ++i) {
      Point surface_intersection = Point::Zero();
      FloatingPoint distance = 0.0;
      // Cast ray from the origin in the direction of the bearing vector until
      // finding an intersection with a surface.
      bool success = getSurfaceDistanceAlongRay<TsdfVoxel>(
          tsdf_layer_, origin, bearing_vectors[i], max_distance_,
          &surface_intersection, &distance);  // <== distance added

      if (!success) {
        continue;
      }

      // Now look up the matching voxels in the intensity layer and mark them.
      // Let's just start with 1.
      Block<IntensityVoxel>::Ptr block_ptr =
          intensity_layer_->allocateBlockPtrByCoordinates(surface_intersection);
      IntensityVoxel& voxel =
          block_ptr->getVoxelByCoordinates(surface_intersection);

      // Get temporal intensity and weight to check if an update is necessary and perform it if necessary
      calcTmpIntensityAndWeight(intensity, distance, tmp_intensity, tmp_weight);
      printf("Intensity = %f // Weight = %f\n", tmp_intensity, tmp_weight);

      // Update intensity and weight if needed
      updateIntensityAndWeight(voxel, tmp_intensity, tmp_weight);

      // Now check the surrounding voxels along the bearing vector. If they have
      // never been observed, then fill in their value. Otherwise don't.
      Point close_voxel = surface_intersection;
      for (int voxel_offset = -intensity_prop_voxel_radius_;
           voxel_offset <= intensity_prop_voxel_radius_; voxel_offset++) {
        close_voxel =
            surface_intersection + bearing_vectors[i] * voxel_offset * voxel_size;  // TODO: Was passiert hier?
        Block<IntensityVoxel>::Ptr new_block_ptr =
            intensity_layer_->allocateBlockPtrByCoordinates(close_voxel);
        IntensityVoxel& new_voxel = block_ptr->getVoxelByCoordinates(close_voxel);

        // Update intensity and weight if needed
        updateIntensityAndWeight(new_voxel, tmp_intensity, tmp_weight);
      }
    }
  }

  void RadioNuclearMapperIntegrator::calcTmpIntensityAndWeight(const float in_intensity, const float in_distance,
                                                               float& tmp_intensity, float& tmp_weight) {
    // Using weight property of a voxel as a confidence value
    // define a temporal weight as the inverse of the distance
    if (in_distance < 1e-6){
      tmp_weight = std::numeric_limits<float>::infinity();
    } else {
      tmp_weight = 1 / in_distance;
    }

    if (dist_func_ == 'i') {  // increasing
      tmp_intensity =  in_intensity * in_distance * in_distance;
    } else if (dist_func_ == 'd') {  // decreasing
      tmp_intensity =  in_intensity / (in_distance * in_distance);
    } else if (dist_func_ == 'c') {  // constant
      tmp_intensity =  in_intensity;
    } else {  // zero
      tmp_intensity =  0.0;
    }
  }

  void RadioNuclearMapperIntegrator::updateIntensityAndWeight(voxblox::IntensityVoxel& voxel,
                                                              const float in_intensity, const float in_weight){
    // If the temporal weight (or confidence) is higher than the stored one, update weight and check the intensity
    if (in_weight >= voxel.weight){
      voxel.weight = in_weight;
      //      if (tmp_intensity > voxel.intensity){
      voxel.intensity = in_intensity;
      //      }
    }
  }
}  // namespace voxblox
