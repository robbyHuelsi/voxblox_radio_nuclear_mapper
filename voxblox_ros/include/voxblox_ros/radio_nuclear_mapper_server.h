#ifndef VOXBLOX_ROS_RADIO_NUCLEAR_MAPPER_SERVER_H_
#define VOXBLOX_ROS_RADIO_NUCLEAR_MAPPER_SERVER_H_

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <memory>

#include <voxblox/core/voxel.h>
#include <voxblox/integrator/intensity_integrator.h>  // TODO
#include <voxblox/utils/color_maps.h>

#include "voxblox_ros/radio_nuclear_mapper_vis.h"
#include "voxblox_ros/intensity_server.h"
#include "voxblox_ros/tsdf_server.h"

namespace voxblox {

class RadioNuclearMapperServer : public IntensityServer {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  RadioNuclearMapperServer(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);
  virtual ~RadioNuclearMapperServer() {}

  virtual void updateMesh();
//  virtual void publishPointclouds();

  void intensityImageCallback(const sensor_msgs::ImageConstPtr& image);

 protected:
  /// Subscriber for intensity images.
  ros::Subscriber intensity_image_sub_;

  // Publish markers for visualization.
  ros::Publisher intensity_pointcloud_pub_;
  ros::Publisher intensity_mesh_pub_;

  /// Parameters of the incoming UNDISTORTED intensity images.
  double focal_length_px_;

  /** How much to subsample the image by (not proper downsampling, just
   * subsampling).
   */
  int subsample_factor_;

  // Intensity layer, integrator, and color maps, all related to storing
  // and visualizing intensity data.
  std::shared_ptr<Layer<IntensityVoxel>> intensity_layer_;
  std::unique_ptr<IntensityIntegrator> intensity_integrator_;

  // Visualization tools.
  std::shared_ptr<ColorMap> color_map_;
};

}  // namespace voxblox

#endif  // VOXBLOX_ROS_RADIO_NUCLEAR_MAPPER_SERVER_H_
