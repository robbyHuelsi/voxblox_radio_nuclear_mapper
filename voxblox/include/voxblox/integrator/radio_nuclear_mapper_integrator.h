/// Plagiarism Notice:
/// The code in this file comes from file intensity_integrator.h
/// and has been adapted for the special purpose of radiation mapping by Robert Hülsmann.
/// New/edited variables or methods of class RadioNuclearMapperIntegrator are marked with the comment "RH".

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

  // Set the max distance for projecting into the TSDF layer.
  void setMaxDistance(const FloatingPoint max_distance) {
    max_distance_ = max_distance;
  }
  FloatingPoint getMaxDistance() const { return max_distance_; }

  /**
   * Integrates intensities into the intensity layer by projecting normalized
   * bearing vectors (in the WORLD coordinate frame) from the origin (also in
   * the world coordinate frame) into the TSDF layer, and then setting the
   * intensities near the surface boundaries.
   */
  void addRadiationSensorValueBearingVectors(const Point& origin,
                                             const Pointcloud& bearing_vectors,
                                             const float radiation_sensor_value);

 private:
  bool use_logarithm_; /// RH
  FloatingPoint max_distance_;

  // Number of voxels to propagate from the surface along the bearing vector.
  int intensity_prop_voxel_radius_;

  const Layer<TsdfVoxel>& tsdf_layer_;
  Layer<IntensityVoxel>* intensity_layer_;

  void updateIntensityVoxel(voxblox::IntensityVoxel& voxel, const float in_intensity, const float in_distance);  /// RH
};

}  // namespace voxblox

#endif  // VOXBLOX_INTEGRATOR_RADIO_NUCLEAR_MAPPER_INTEGRATOR_H_
