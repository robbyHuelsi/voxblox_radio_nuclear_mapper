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

//  void setRadiationSensorMinValue(const float min_val) {  // TODO:Remove
//    radiation_sensor_min_ = min_val;
//  }
//  void setRadiationSensorMaxValue(const float max_val) {
//    radiation_sensor_max_ = max_val;
//  }

  void setUseLogarithm(const bool use_logarithm) {
    use_logarithm_ = use_logarithm;
  }

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
  void addRadiationSensorValueBearingVectors(const Point& origin,
                                             const Pointcloud& bearing_vectors,
                                             const float radiation_sensor_value);

  /**  // TODO
   *
   * @param dist_func
   * @param radiation_sensor_value
   * @param distance
   * @param intensity
   * @param confidence
   */
  void calcIntensityAndConfidence(const float radiation_sensor_value, const float distance,
                                  float& intensity, float& confidence);

 private:
//  float radiation_sensor_min_;  // TODO:Remove
//  float radiation_sensor_max_;  // TODO:Remove
  bool use_logarithm_;
  FloatingPoint max_distance_;
//  float max_weight;  // TODO: Remove

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
