#ifndef VOXBLOX_ROS_RADIO_NUCLEAR_MAPPER_SERVER_H_
#define VOXBLOX_ROS_RADIO_NUCLEAR_MAPPER_SERVER_H_

#include "voxblox_ros/tsdf_server.h"
#include "voxblox_ros/radio_nuclear_mapper_vis.h"
#include <voxblox/integrator/radio_nuclear_mapper_integrator.h>  // TODO
#include <abc_msgs_fkie/MeasurementRaw.h>
#include <std_msgs/String.h>

namespace voxblox {

  class RadioNuclearMapperServer : public TsdfServer {
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW

      RadioNuclearMapperServer(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);
      virtual ~RadioNuclearMapperServer() {}

      void getServerConfigFromRosParam(const ros::NodeHandle& nh_private);

      virtual bool setColorMapScheme(const std::string color_map_scheme_name, std::shared_ptr<ColorMap>& color_map);
      virtual bool getRadiationDistanceFunctionByName(const std::string distance_function_name, char& dist_func); //TODO: Change char to function pointer
      virtual void setCMExtrValByMostExtrPossible(const float radiation_msg_val_min, const float radiation_msg_val_max,
                                                  const char dist_func, const bool use_logarithm,
                                                  const float radiation_max_distance,
                                                  const std::shared_ptr<ColorMap>& color_map);

      /// Incremental update.
      virtual void updateMesh();
      /// Batch update.
      virtual bool generateMesh();
      virtual bool generateMesh(const std::string& distance_function, const bool use_logarithm);
      /// Publishes all available pointclouds.
      virtual void publishPointclouds();

    protected:
      /// Intensity layer, integrator, and color maps, all related to storing
      /// and visualizing intensity data.
      std::shared_ptr<Layer<IntensityVoxel>> radiation_layer_;
      std::unique_ptr<RadioNuclearMapperIntegrator> radiation_integrator_;
      Mesh radiation_mesh_; //TODO: Replace with tsdf mesh or remove completely

      /// Parameters for radiological nuclear mapper
      std::string radiation_sensor_topic_;
      std::string radiation_sensor_frame_id_;
      float radiation_max_distance_;
      char radiation_distance_function_;
      float radiation_msg_val_min_;
      float radiation_msg_val_max_;
      bool radiation_msg_use_log_;
      int radiation_ang_res_y_; //TODO: use euqidistant points
      int radiation_ang_res_z_;

      /// Parameters for mesh saving
      std::string save_mesh_trigger_topic_; //TODO: Use tsfd_server procedure for saving?

      /// Visualization tools
      std::shared_ptr<ColorMap> color_map_; //TODO: Use tsfd_server variable?

      /// Publish markers for visualization.
      ros::Publisher radiation_pointcloud_pub_; //TODO: Use tsfd_server publisher?
      ros::Publisher radiation_mesh_pub_; //TODO: Use tsfd_server publisher?

      /// Subscribe radiation intensity
      ros::Subscriber radiation_sensor_sub_;

      /// Subscribe save mesh trigger
      ros::Subscriber save_mesh_trigger_sub_; //TODO: Use tsfd_server procedure for saving?

      void radiationSensorCallback(const abc_msgs_fkie::MeasurementRawConstPtr& msg);
      void saveMeshTriggerCallback(const std_msgs::StringConstPtr& msg); //TODO: Use tsfd_server procedure for saving?
  };

}  // namespace voxblox

#endif  // VOXBLOX_ROS_RADIO_NUCLEAR_MAPPER_SERVER_H_
