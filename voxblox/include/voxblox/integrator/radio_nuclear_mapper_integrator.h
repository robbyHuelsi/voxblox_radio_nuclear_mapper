#ifndef VOXBLOX_INTEGRATOR_RADIO_NUCLEAR_MAPPER_INTEGRATOR_H_
#define VOXBLOX_INTEGRATOR_RADIO_NUCLEAR_MAPPER_INTEGRATOR_H_

#include <glog/logging.h>
#include <Eigen/Core>
#include <algorithm>
#include <queue>
#include <utility>
#include <vector>

#include "voxblox/core/layer.h"
#include "voxblox/core/voxel.h"
#include "voxblox/core/common.h"
#include "voxblox/integrator/integrator_utils.h"
#include "voxblox/utils/timing.h"

#include "voxblox/utils/radio_nuclear_mapper_distance_utils.h"

namespace voxblox {

/**
 * Integrates intensities from a set of bearing vectors (i.e., an intensity
 * image, such as a thermal image) by projecting them onto the TSDF surface
 * and coloring near the surface crossing.
 */
class RadioNuclearMapperIntegrator {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  RadioNuclearMapperIntegrator(const Layer<TsdfVoxel>& tsdf_layer,
                               Layer<IntensityVoxel>* intensity_layer);

  /// Set the max distance for projecting into the TSDF layer.
  void setMaxDistance(const FloatingPoint max_distance) {
    max_distance_ = max_distance;
  }
  FloatingPoint getMaxDistance() const { return max_distance_; }

  std::vector<std::string> getAllowedDistanceFunctions() const {return allowed_distance_functions_;}


  void setDistanceFunction(const std::string distance_function);

  /**
   * Integrates intensities into the intensity layer by projecting normalized
   * bearing vectors (in the WORLD coordinate frame) from the origin (also in
   * the world coordinate frame) into the TSDF layer, and then setting the
   * intensities near the surface boundaries.
   */
  void addIntensityBearingVectors(const Point& origin,
                                  const Pointcloud& bearing_vectors,
                                  //const std::vector<float>& intensities, // TODO: Remove
                                  const float intensity);

  /**  // TODO
   *
   * @param dist_func
   * @param in_intensity
   * @param in_distance
   * @param tmp_intensity
   * @param tmp_weight
   */
  void calcTmpIntensityAndWeight(const float in_intensity, const float in_distance,
                                 float& tmp_intensity, float& tmp_weight);

 private:
  FloatingPoint max_distance_;
//  float max_weight;  // TODO: Remove
  float tmp_weight;  // <= added
  float tmp_intensity;  // <== added
  /// Number of voxels to propagate from the surface along the bearing vector.
  int intensity_prop_voxel_radius_;

  const Layer<TsdfVoxel>& tsdf_layer_;
  Layer<IntensityVoxel>* intensity_layer_;

  std::vector<std::string> allowed_distance_functions_;
  char dist_func_;

  /**  // TODO
   *
   * @param voxel
   * @param in_intensity
   * @param in_weight
   */
  void updateIntensityAndWeight(IntensityVoxel& voxel,
                                const float in_intensity, const float in_weight);
};

}  // namespace voxblox

#endif  // VOXBLOX_INTEGRATOR_RADIO_NUCLEAR_MAPPER_INTEGRATOR_H_
