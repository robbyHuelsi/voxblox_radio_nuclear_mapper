#include "voxblox/integrator/radio_nuclear_mapper_integrator.h"

namespace voxblox {
  RadioNuclearMapperIntegrator::RadioNuclearMapperIntegrator(const Layer<TsdfVoxel>& tsdf_layer,
                                                             Layer<IntensityVoxel>* intensity_layer)
      : // radiation_sensor_min_(0.0),  // TODO:Remove
        // radiation_sensor_max_(1.0),  // TODO:Remove
//        use_logarithm_(false),
        max_distance_(5.0),
        intensity_prop_voxel_radius_(2),
        tsdf_layer_(tsdf_layer),
        intensity_layer_(intensity_layer)
//        allowed_distance_functions_({"increasing", "decreasing", "constant"}),
//        dist_func_('z')
        {}

//  void RadioNuclearMapperIntegrator::setDistanceFunction(const std::string distance_function) {
//    if (distance_function == "increasing"){
//      dist_func_ = 'i';
//    }else if (distance_function == "decreasing"){
//      dist_func_ = 'd';
//    }else if (distance_function == "constant"){
//      dist_func_ = 'c';
//    }else{ // zero
//      dist_func_ = 'z';
//    }
//    printf("dist_func_ = %c\n", dist_func_);
//  }

  void RadioNuclearMapperIntegrator::addRadiationSensorValueBearingVectors(const Point& origin,
                                                                           const Pointcloud& bearing_vectors,
                                                                           const float radiation_sensor_value){
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

      // Get temporal intensity and confidence to check if an update is necessary and perform it if necessary
//      float intensity, confidence;
//      calcIntensityAndConfidence(radiation_sensor_value, distance, intensity, confidence);
//      float confidence;
//      calcConfidence(distance,confidence);
//      printf("Intensity = %f // Confidence = %f\n", intensity, confidence);  // TODO: Remove

      // Update intensity and confidence if needed
//      updateIntensityAndWeight(voxel, intensity, confidence);
      updateIntensityVoxel(voxel, radiation_sensor_value, distance);

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
//        updateIntensityAndWeight(new_voxel, intensity, confidence);
        updateIntensityVoxel(new_voxel, radiation_sensor_value, distance);
      }
    }
  }

//  void RadioNuclearMapperIntegrator::calcConfidence(const float distance, float& confidence) {
//    // Define confidence as the inverse of the distance
//    if (distance < 1e-6){
//      confidence = std::numeric_limits<float>::infinity();
//    } else {
//      confidence = 1 / distance;
//    }
//  }

//  void RadioNuclearMapperIntegrator::calcIntensityAndConfidence(const float radiation_sensor_value,
//                                                                const float distance,
//                                                                float& intensity, float& confidence) {
//    intensity = radiation_sensor_value;
//
//    // Define confidence as the inverse of the distance
//    if (distance < 1e-6){
//      confidence = std::numeric_limits<float>::infinity();
//    } else {
//      confidence = 1 / distance;
//    }
//
//    // Apply the desired function
//    if (dist_func_ == 'i') {  // increasing
////      intensity =  intensity * distance * distance;
//      intensity = intensity * pow(distance + 1, 2);
//    } else if (dist_func_ == 'd') {  // decreasing
////      intensity =  intensity / (distance * distance);
//      intensity = intensity / pow(distance + 1, 2);
//    } else if (dist_func_ == 'c') {  // constant
//      intensity = intensity;
//    } else {  // zero
//      intensity =  0.0;
//    }
//
//    // Use logarithmic mapping if needed
//    if(use_logarithm_){
//      intensity = log(intensity);
//      intensity = intensity < 0.0 ? 0.0 : intensity;  // TODO: Comment
//    }
//
////    // Normalize between 0.0 and 1.0  // TODO: Remove
////    intensity = radiation_sensor_min_ + intensity / (radiation_sensor_max_ - radiation_sensor_min_);
//
//  }

  void RadioNuclearMapperIntegrator::updateIntensityVoxel(voxblox::IntensityVoxel& voxel,
                                                              const float in_intensity, const float in_distance){
    // Using weight property of a voxel as distance value
    // If the temporal confidence (1/d) is higher than the stored one, update weight and intensity
//    printf("Voxel: Intensity = %f; Distance = %f\n", in_intensity, in_distance);
    if (in_distance < voxel.weight) {
      voxel.weight = in_distance;
      voxel.intensity = in_intensity;
    } else if (in_distance == voxel.weight){
      if (in_intensity > voxel.intensity) {
//        printf("Same distance, but current intensity is higher.\n");
        voxel.intensity = in_intensity;
//      printf("Voxel updated: Intensity = %f; Distance = %f\n", in_intensity, in_distance);
      }
    }
  }
}  // namespace voxblox
