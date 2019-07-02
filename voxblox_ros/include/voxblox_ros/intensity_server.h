#ifndef VOXBLOX_ROS_INTENSITY_SERVER_H_
#define VOXBLOX_ROS_INTENSITY_SERVER_H_

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <memory>

#include <voxblox/core/voxel.h>
#include <voxblox/integrator/intensity_integrator.h>
#include <voxblox/utils/color_maps.h>

#include "voxblox_ros/intensity_vis.h"
#include "voxblox_ros/tsdf_server.h"

// radiological nuclear mapper
#include <abc_msgs_fkie/MeasurementRaw.h>

namespace voxblox {

class IntensityServer : public TsdfServer {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  IntensityServer(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);
  virtual ~IntensityServer() {}

  virtual void updateMesh();
  virtual void publishPointclouds();

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

  // radiological nuclear mapper
  std::string radiation_sensor_frame_id_;
  float radiation_msg_val_min_;
  float radiation_msg_val_max_;
  bool radiation_msg_use_log_;
  int radiation_image_height_;
  int radiation_image_width_;
  float radiation_image_max_dist_;
  float radiation_image_dispersion_;
  unsigned int radiation_msg_step_;
  ros::Subscriber radiation_sensor_sub_;
  float distance(int x, int y);
  float squared_distance(int x, int y);
  void radiationSensorCallback(const abc_msgs_fkie::MeasurementRawConstPtr& msg);
//  void radiationSensorCallback(abc_msgs_fkie::MeasurementRawConstPtr msg);
  std_msgs::Header intensity_test_image_header_;
  ros::Publisher intensity_test_image_publisher_;
};

}  // namespace voxblox

#endif  // VOXBLOX_ROS_INTENSITY_SERVER_H_
