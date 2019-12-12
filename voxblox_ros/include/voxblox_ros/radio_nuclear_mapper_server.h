#ifndef VOXBLOX_ROS_RADIO_NUCLEAR_MAPPER_SERVER_H_
#define VOXBLOX_ROS_RADIO_NUCLEAR_MAPPER_SERVER_H_

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <memory>

#include <voxblox/core/voxel.h>
#include <voxblox/integrator/radio_nuclear_mapper_integrator.h>  // TODO
#include <voxblox/utils/color_maps.h>

#include "voxblox_ros/radio_nuclear_mapper_vis.h"
#include "voxblox_ros/intensity_server.h"
#include "voxblox_ros/tsdf_server.h"

// radiological nuclear mapper
#include <abc_msgs_fkie/MeasurementRaw.h>

namespace voxblox {
  class RadioNuclearMapperServer : public IntensityServer {
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW

      RadioNuclearMapperServer(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);
      virtual ~RadioNuclearMapperServer() {}

      void getServerConfigFromRosParam(const ros::NodeHandle& nh_private);

      virtual void updateMesh();
      //  virtual void publishPointclouds();

      void intensityImageCallback(const sensor_msgs::ImageConstPtr& image);

    protected:
      /// Subscriber for Radiation message.
      ros::Subscriber radiation_sensor_sub_;

      /// Publish markers for visualization.
      ros::Publisher intensity_pointcloud_pub_;
      ros::Publisher intensity_mesh_pub_;

      /// Intensity layer, integrator, and color maps, all related to storing
      /// and visualizing intensity data.
      std::shared_ptr<Layer<IntensityVoxel>> intensity_layer_;
      std::unique_ptr<RadioNuclearMapperIntegrator> rnm_integrator_;

      /// Visualization tools
      std::shared_ptr<ColorMap> color_map_;

      /// Parameters for radiological nuclear mapper
      std::string radiation_sensor_topic_;
      std::string radiation_sensor_frame_id_;
      float radiation_max_distance_;
      std::string radiation_distance_function_;
      float radiation_msg_val_min_;
      float radiation_msg_val_max_;
      bool radiation_msg_use_log_;
      int radiation_ang_res_y_;
      int radiation_ang_res_z_;

      unsigned int radiation_msg_step_;

//      float distance(int x, int y);  // TODO: Remove
//      float squared_distance(int x, int y);
      void radiationSensorCallback(const abc_msgs_fkie::MeasurementRawConstPtr& msg);
      //  void radiationSensorCallback(abc_msgs_fkie::MeasurementRawConstPtr msg);

  };

}  // namespace voxblox

#endif  // VOXBLOX_ROS_RADIO_NUCLEAR_MAPPER_SERVER_H_
