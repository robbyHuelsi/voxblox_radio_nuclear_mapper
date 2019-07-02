#include "voxblox/integrator/intensity_integrator.h"

#include "voxblox/utils/distance_utils.h"

namespace voxblox {

IntensityIntegrator::IntensityIntegrator(const Layer<TsdfVoxel>& tsdf_layer,
                                         Layer<IntensityVoxel>* intensity_layer)
    : max_distance_(5.0),
      max_weight_(100.0),
      intensity_prop_voxel_radius_(2),
      tsdf_layer_(tsdf_layer),
      intensity_layer_(intensity_layer) {}

void IntensityIntegrator::addIntensityBearingVectors(
    const Point& origin, const Pointcloud& bearing_vectors,
    const std::vector<float>& intensities) {

  timing::Timer intensity_timer("intensity/integrate");

  CHECK_EQ(bearing_vectors.size(), intensities.size())
      << "Intensity and bearing vector size does not match!";
  const FloatingPoint voxel_size = tsdf_layer_.voxel_size();

  // Get middle point
  const unsigned int middle_int = (int)round(bearing_vectors.size()/2);
  Point middle_point = Point::Zero();
  const DistanceUtilsResult m = getSurfaceDistanceAlongRay<TsdfVoxel>(
      tsdf_layer_, origin,
      bearing_vectors[middle_int],
      max_distance_, &middle_point);

  if(!m.closer_than_max_dist){
    printf("Middle point too far away (>%f m). \n", m.distance);
    return;
  }

  //float distance = m.distance / max_distance_;
  float assumed_intensity = pow(m.distance, 2) * intensities[middle_int];

  printf("middle_int: %d; middle_point: (%f, %f, %f), assumed_intensity: %f\n",
      middle_int, middle_point[0], middle_point[1], middle_point[2], assumed_intensity);

  for (size_t i = 0; i < bearing_vectors.size(); ++i) {
    Point surface_intersection = Point::Zero();
    // Cast ray from the origin in the direction of the bearing vector until
    // finding an intersection with a surface.
    DistanceUtilsResult d = getSurfaceDistanceAlongRay<TsdfVoxel>(
//    bool success = getSurfaceDistanceAlongRay<TsdfVoxel>(
        tsdf_layer_, origin, bearing_vectors[i], max_distance_,
        &surface_intersection);

    if(!d.closer_than_max_dist){
      printf("Too far away (>%f m). \n", d.distance);
      continue;
    }

    if(d.distance < 0.1){
      printf("Too close (%f m). \n", d.distance);
      continue;
    }

    float intensity;
    if(assumed_intensity > 0.0) {
      float inter_point_sq_distance;
      if (i == middle_int) {
        inter_point_sq_distance = 0.0;
        intensity = assumed_intensity;
      } else {
        inter_point_sq_distance = (pow(surface_intersection.x() - middle_point.x(), 2) +
                                   pow(surface_intersection.y() - middle_point.y(), 2) +
                                   pow(surface_intersection.z() - middle_point.z(), 2));
        intensity = assumed_intensity / inter_point_sq_distance; //todo assumed_intensity == 0 ? 0 :
      }

      //float new_weight = (1.0 - distance);
      printf("d_1: %f; d^2_2: %f; intensity_img: %f, intensity_res: %f\n", d.distance, inter_point_sq_distance,
             intensities[i], intensity);
    }else{
      intensity = 0.0;
      printf("intensity: 0\n");
    }

    // Now look up the matching voxels in the intensity layer and mark them.
    // Let's just start with 1.
    Block<IntensityVoxel>::Ptr block_ptr =
        intensity_layer_->allocateBlockPtrByCoordinates(surface_intersection);
    IntensityVoxel& voxel =
        block_ptr->getVoxelByCoordinates(surface_intersection);
    // Version 0
//    voxel.intensity =
//        (voxel.weight * voxel.intensity + intensity) / (voxel.weight + 1);
//    voxel.weight += 1.0;
//    if (voxel.weight > max_weight_) {
//      voxel.weight = max_weight_;
//    }
    // Version 2
    if (intensity > voxel.intensity) {
      voxel.intensity = intensity;
    }
    // Version 3
//    voxel.intensity =
//        (voxel.weight * voxel.intensity + intensity * new_weight) / (voxel.weight + new_weight);
//    voxel.weight += new_weight;
//    if (voxel.weight > max_weight_) {
//      voxel.weight = max_weight_;
//    }

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
      if (new_voxel.weight < 1e-6) { //todo: nochmal checken, was das für Auswirkungen hat
        // Version 0
//        new_voxel.intensity =
//            (new_voxel.weight * new_voxel.intensity + intensity) / (new_voxel.weight + 1);
//
//        new_voxel.weight = max_weight_;
        // Version 1
//        new_voxel.intensity = intensities[i];
//        new_voxel.weight += 1.0;
        // Version 2
        if (intensity > new_voxel.intensity) {
          new_voxel.intensity = intensity;
        }
        // Version 3
//        new_voxel.intensity =
//            (new_voxel.weight * new_voxel.intensity + intensity * new_weight) / (new_voxel.weight + new_weight);
//        new_voxel.weight += new_weight;
//        if (new_voxel.weight > max_weight_) {
//          new_voxel.weight = max_weight_;
//        }
      }
    }
  }
}

}  // namespace voxblox
