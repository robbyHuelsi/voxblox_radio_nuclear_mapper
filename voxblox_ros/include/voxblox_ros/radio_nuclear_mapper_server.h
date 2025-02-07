/// Plagiarism Notice:
/// The code in this file comes from file intensity_server.h
/// and has been adapted for the special purpose of radiation mapping by Robert Hülsmann.
/// New/edited lines of code are marked with the comment "RH" (except simple renaming "intensity" to "radiation" etc.).

#ifndef VOXBLOX_ROS_RADIO_NUCLEAR_MAPPER_SERVER_H_
#define VOXBLOX_ROS_RADIO_NUCLEAR_MAPPER_SERVER_H_

#include "voxblox/core/common.h"
#include "voxblox/integrator/radio_nuclear_mapper_integrator.h" /// RH
#include "voxblox/utils/radio_nuclear_mapper_color_maps.h" /// RH
#include "voxblox_ros/tsdf_server.h"
#include "voxblox_ros/radio_nuclear_mapper_ptcloud_vis.h" /// RH
#include <abc_msgs_fkie/MeasurementRaw.h>
#include <std_msgs/String.h>

#include <jsoncpp/json/json.h>
#include <jsoncpp/json/reader.h>
#include <jsoncpp/json/writer.h>
#include <jsoncpp/json/value.h>

namespace voxblox {

  class RadioNuclearMapperServer;
  /// Type definition for radiation distance function (RH)
  typedef float (*RDFType)(const float);

  class RadioNuclearMapperServer : public TsdfServer {
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW

      RadioNuclearMapperServer(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);
      ~RadioNuclearMapperServer() override = default;

      /// handling with parameters (RH)
      void getServerConfigFromRosParam(const ros::NodeHandle& nh_private);

      /// Incremental update.
      void updateMesh() override;

      /// Batch update (whole block: RH)
      bool generateMesh() override;
      bool generateMesh(const std::string& time_stemp_str);
      bool generateMesh(const std::string& distance_function_name, bool use_logarithm,
                        const std::string& color_map_scheme_name, const std::string& adjust_extr_val,
                        const std::string& time_stemp_str);

      /// Publishes all available pointclouds.
      void publishPointclouds() override;

    protected:
      /// Intensity layer, integrator, and color maps, all related to storing
      /// and visualizing intensity data.
      std::shared_ptr<Layer<RadiationVoxel>> radiation_layer_;
      std::unique_ptr<RadioNuclearMapperIntegrator> radiation_integrator_;

      /// Parameters for radiological nuclear mapper (whole block: RH)
      std::string radiation_sensor_topic_;
      std::string radiation_sensor_frame_id_;
      float radiation_max_distance_;
      std::string radiation_distance_function_name_;
      RDFType radiation_distance_function_;
      bool radiation_use_logarithm_;
      float radiation_msg_val_min_;
      float radiation_msg_val_max_;
      int radiation_bearing_vector_num_;
      Pointcloud bearing_vectors_;
      size_t radiation_sensor_callback_counter_;
      std::string radiation_color_map_scheme_name_;
      std::shared_ptr<ColorMap> radiation_color_map_;
      ros::Time last_time_stamp_;

      /// Parameters for mesh saving (RH)
      std::string save_mesh_trigger_topic_;

      // Publish markers for visualization.
      ros::Publisher radiation_pointcloud_pub_;
      ros::Publisher radiation_mesh_pub_;

      /// Subscribe radiation intensity (RH)
      ros::Subscriber radiation_sensor_sub_;

      /// Subscribe save mesh trigger (RH)
      ros::Subscriber save_mesh_trigger_sub_;

      /// handling with parameters (whole block: RH)
      static bool setColorMapScheme(const std::string& color_map_scheme_name, std::shared_ptr<ColorMap>& color_map);
      bool getRadiationDistanceFunctionByName(const std::string& distance_function_name,
                                              RDFType& rad_dist_func);
      std::tuple<float, float> setCMExtrValByMostExtrPossible(float radiation_msg_val_min,
                                                              float radiation_msg_val_max,
                                                              const RDFType& rad_dist_func,
                                                              bool use_logarithm,
                                                              float radiation_max_distance,
                                                              std::shared_ptr<ColorMap>& color_map);
      std::tuple<float, float> setCMExtrValByExtrValOfVoxelsAtMeshPositions(const Mesh& mesh,
                                                                            const Layer<RadiationVoxel>& radiation_layer,
                                                                            const RDFType& rad_dist_func, bool use_logarithm,
                                                                            const std::string& ident_str,
                                                                            std::shared_ptr<ColorMap>& color_map);

      static void generateBearingVectors(int n, Pointcloud& bearing_vectors); /// RH

      /// whole block: RH
      static float calcIntensity(float sensor_value, float distance, const RDFType& rad_dist_func, bool use_logarithm);
      static Color getColorForVoxelPointer(const RadiationVoxel* voxel, const std::shared_ptr<ColorMap>& color_map,
                                           const RDFType& rad_dist_func, bool use_logarithm);
      static void recolorVoxbloxMeshMsgByRadiationIntensity(const Layer<RadiationVoxel>& radiation_layer,
                                                            const std::shared_ptr<ColorMap>& color_map,
                                                            const RDFType& rad_dist_func, bool use_logarithm,
                                                            voxblox_msgs::Mesh* mesh_msg);
      static void recolorVoxbloxMeshByRadiationIntensity(const Layer<RadiationVoxel>& radiation_layer,
                                                         const std::shared_ptr<ColorMap>& color_map,
                                                         const RDFType& rad_dist_func, bool use_logarithm,
                                                         std::string& ident_str, Mesh& mesh);

      /// whole block: RH
      void radiationSensorCallback(const abc_msgs_fkie::MeasurementRawConstPtr& msg);
      void saveMeshTriggerCallback(const std_msgs::StringConstPtr& msg);
      std::string vec2str(const std::vector<std::string> &v);
      std::string vec2str(const std::vector<bool> &v);

      /// Radiation distance functions (whole block: RH)
      static float rad_dist_func_increasing(float distance);
      static float rad_dist_func_decreasing(float distance);
      static float rad_dist_func_constant(float distance);
      static float rad_dist_func_infinity(float distance);

      /// Allowed values (whole block: RH)
//      const std::string allowed_radiation_distance_function_names_[3] = {"increasing", "decreasing", "constant"};
//      const std::string allowed_radiation_color_map_scheme_names_[6] = {"rainbow", "inverse_rainbow", "grayscale",
//                                                                        "inverse_grayscale", "ironbow", "traffic-light"};
      const std::map<std::string, RDFType>& getRDFMap() {
        static const auto* map = new std::map<std::string, RDFType>{
            {"increasing", rad_dist_func_increasing},
            {"decreasing", rad_dist_func_decreasing},
            {"constant", rad_dist_func_constant}
          };
          return *map;
      }

      static const std::map<std::string, ColorMap*>& getCMSMap() {
          const auto* map = new std::map<std::string, ColorMap*>{
                  {"rainbow", new RainbowColorMap()},
                  {"inverse_rainbow", new InverseRainbowColorMap()},
                  {"grayscale", new GrayscaleColorMap()},
                  {"inverse_grayscale", new InverseGrayscaleColorMap()},
                  {"ironbow", new IronbowColorMap()},
                  {"traffic-light", new TrafficLightColorMap()}
          };
          return *map;
      }
  };
}  // namespace voxblox

#endif  // VOXBLOX_ROS_RADIO_NUCLEAR_MAPPER_SERVER_H_
