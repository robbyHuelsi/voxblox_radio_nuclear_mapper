#ifndef VOXBLOX_ROS_RADIO_NUCLEAR_MAPPER_SERVER_H_
#define VOXBLOX_ROS_RADIO_NUCLEAR_MAPPER_SERVER_H_

#include "voxblox_ros/tsdf_server.h"
#include "voxblox/integrator/radio_nuclear_mapper_integrator.h"  // TODO
#include <abc_msgs_fkie/MeasurementRaw.h>
#include <std_msgs/String.h>

namespace voxblox {

  class RadioNuclearMapperServer;
  /// Type definition for radiation distance function
  typedef void (RadioNuclearMapperServer::*RDFType)(float&, const float&);

  class RadioNuclearMapperServer : public TsdfServer {
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
      RadioNuclearMapperServer(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);
      virtual ~RadioNuclearMapperServer() {}

      /// handling with parameters
      void getServerConfigFromRosParam(const ros::NodeHandle& nh_private);

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
      std::string radiation_distance_function_name_;
      RDFType radiation_distance_function_;
      float radiation_msg_val_min_;
      float radiation_msg_val_max_;
      bool radiation_msg_use_log_;
      int radiation_bearing_vector_num_; //TODO: use euqidistant points
      Pointcloud bearing_vectors_;
      size_t rad_sen_callback_counter_;

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

      /// handling with parameters
      bool setColorMapScheme(const std::string color_map_scheme_name, std::shared_ptr<ColorMap>& color_map);
      bool getRadiationDistanceFunctionByName(const std::string distance_function_name,
                                              RDFType& rad_dist_func);
      void setCMExtrValByMostExtrPossible(const float radiation_msg_val_min,
                                          const float radiation_msg_val_max,
                                          RDFType& rad_dist_func,
                                          const bool use_logarithm,
                                          const float radiation_max_distance,
                                          const std::shared_ptr<ColorMap>& color_map);

      void generateBearingVectors(const int n, Pointcloud& bearing_vectors);

      void calcIntensity(const float sensor_value, const float distance,
                         RDFType& rad_dist_func, const bool use_logarithm, float& intensity);
      Color getColorForVoxelPointer(const IntensityVoxel* voxel, const std::shared_ptr<ColorMap>& color_map,
                                    RDFType& rad_dist_func, const bool use_logarithm);
      void recolorVoxbloxMeshMsgByRadiationIntensity(const Layer<IntensityVoxel>& intensity_layer,
                                                     const std::shared_ptr<ColorMap>& color_map,
                                                     RDFType& rad_dist_func, const bool use_logarithm,
                                                     voxblox_msgs::Mesh* mesh_msg);

      void radiationSensorCallback(const abc_msgs_fkie::MeasurementRawConstPtr& msg);
      void saveMeshTriggerCallback(const std_msgs::StringConstPtr& msg); //TODO: Use tsfd_server procedure for saving?

      void rad_dist_func_increasing(float& intensity, const float& distance);
      void rad_dist_func_decreasing(float& intensity, const float& distance);
      void rad_dist_func_constant(float& intensity, const float& distance);
      void rad_dist_func_zero(float& intensity, const float& distance);
  };
}  // namespace voxblox

#endif  // VOXBLOX_ROS_RADIO_NUCLEAR_MAPPER_SERVER_H_
