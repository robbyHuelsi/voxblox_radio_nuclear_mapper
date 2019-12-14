#include "voxblox_ros/radio_nuclear_mapper_server.h"

namespace voxblox {

  RadioNuclearMapperServer::RadioNuclearMapperServer(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private):
  IntensityServer(nh, nh_private) {

    cache_mesh_ = true;

    intensity_layer_.reset(new Layer<IntensityVoxel>(tsdf_map_->getTsdfLayer().voxel_size(),
                                                     tsdf_map_->getTsdfLayer().voxels_per_side()));
    rnm_integrator_.reset(new RadioNuclearMapperIntegrator(tsdf_map_->getTsdfLayer(),
                                                           intensity_layer_.get()));

    /// Get ROS params for radiation sensor
    getServerConfigFromRosParam(nh_private_);

    /// Forward some paramters to integrator
    rnm_integrator_->setUseLogarithm(radiation_msg_use_log_);
//    rnm_integrator_->setRadiationSensorMinValue(radiation_msg_val_min_);  // TODO: Remove
//    rnm_integrator_->setRadiationSensorMaxValue(radiation_msg_val_max_);  // TODO: Remove
    rnm_integrator_->setMaxDistance(radiation_max_distance_);
    rnm_integrator_->setDistanceFunction(radiation_distance_function_);

    /// Set minumum and maximum value of colormap
    std::vector<float> intensity_extreme_values;
    float radiation_msg_extreme_values[] = {radiation_msg_val_min_, radiation_msg_val_max_};
    float distance_extreme_values[] = {0.0, radiation_max_distance_};
    float tmp_intensity, tmp_weight;
    for(float msg: radiation_msg_extreme_values){
      for(float dist: distance_extreme_values){
        rnm_integrator_->calcIntensityAndConfidence(msg, dist, tmp_intensity, tmp_weight);
        intensity_extreme_values.insert(intensity_extreme_values.end(), 1, tmp_intensity);
      }
    }
    float intensity_min_value = *std::min_element(intensity_extreme_values.begin(), intensity_extreme_values.end());
    float intensity_max_value = *std::max_element(intensity_extreme_values.begin(), intensity_extreme_values.end());
    color_map_->setMinValue(intensity_min_value);
    color_map_->setMaxValue(intensity_max_value);
    ROS_INFO_STREAM("Color map value range is: [" << intensity_min_value << ", " << intensity_max_value << "]");

    // Publishers for output.
    radiation_pointcloud_pub_ =
        nh_private_.advertise<pcl::PointCloud<pcl::PointXYZI> >(
            "radiation_pointcloud", 1, true);
    radiation_mesh_pub_ =
        nh_private_.advertise<voxblox_msgs::Mesh>("radiation_mesh", 1, true);

    // Subscribe nuclear intensity
    radiation_sensor_sub_ = nh_private_.subscribe(
        radiation_sensor_topic_, 1, &RadioNuclearMapperServer::radiationSensorCallback, this);
  }

  void RadioNuclearMapperServer::getServerConfigFromRosParam(
      const ros::NodeHandle& nh_private) {
    /// Define pre-sets of parameters
    radiation_sensor_topic_ = "";
    radiation_sensor_frame_id_ = "";
    radiation_max_distance_ = rnm_integrator_->getMaxDistance();
    radiation_distance_function_ = "constant";
    radiation_msg_val_min_ = 0.0;
    radiation_msg_val_max_ = 100000.0;
    radiation_msg_use_log_ = false;
    radiation_ang_res_y_ = 100;
    radiation_ang_res_z_ = 100;
    std::string color_map_scheme = "ironbow";

    /// Get ROS parameters
    nh_private.param<std::string>("radiation_sensor_topic", radiation_sensor_topic_, radiation_sensor_topic_);
    nh_private.param<std::string>("radiation_sensor_frame_id", radiation_sensor_frame_id_, radiation_sensor_frame_id_);
    nh_private.param("radiation_max_distance", radiation_max_distance_, radiation_max_distance_);
    nh_private.param<std::string>("radiation_distance_function", radiation_distance_function_, radiation_distance_function_);
    nh_private.param("radiation_msg_val_min", radiation_msg_val_min_, radiation_msg_val_min_);
    nh_private.param("radiation_msg_val_max", radiation_msg_val_max_, radiation_msg_val_max_);
    nh_private.param("radiation_msg_use_log", radiation_msg_use_log_, radiation_msg_use_log_);
    nh_private.param("radiation_ang_res_y_", radiation_ang_res_y_, radiation_ang_res_y_);
    nh_private.param("radiation_ang_res_z_", radiation_ang_res_z_, radiation_ang_res_z_);
    nh_private.param("radiation_colormap", color_map_scheme, color_map_scheme);

    /// Check parameter validity for sensor topic parameter and print it
    if(radiation_sensor_topic_.empty()){
      ROS_ERROR("ROS topic for radiological nuclear sensor data not specified.");
      ROS_INFO("Use parameter 'radiation_sensor_topic' to define.");
    }else {
      ROS_INFO_STREAM("Radiation sensor data topic: " << radiation_sensor_topic_);
    }

    /// Check parameter validity for sensor frame parameter and print it
    if (radiation_sensor_frame_id_.empty()) {
      ROS_ERROR("Frame id of radiological nuclear sensor not specified.");
      ROS_INFO("Use parameter 'radiation_sensor_frame_id' next time.");
    }else{
      ROS_INFO_STREAM("Radiation sensor frame id: " << radiation_sensor_frame_id_);
    }

    /// Check parameter validity for distance parameters and print it
    std::vector<std::string> allowed_funcs = rnm_integrator_->getAllowedDistanceFunctions();
    if(std::find(allowed_funcs.begin(), allowed_funcs.end(), radiation_distance_function_) == allowed_funcs.end()){
      // Distance function string is not allowed
      ROS_ERROR("Radiation distance function is called by a not supported string.");
      std::stringstream allowed_funcs_ss = std::stringstream();
      for(std::vector<std::string>::iterator it = allowed_funcs.begin(); it != allowed_funcs.end(); ++it) {
        allowed_funcs_ss << *it << ", ";
      }
      ROS_INFO_STREAM("Use one of the following commands for 'radiation_distance_function': " <<
                                                                                              allowed_funcs_ss.str().substr(0, allowed_funcs_ss.str().length()-2));
    }else{
      ROS_INFO_STREAM("Radiation will map in a radius of " << radiation_max_distance_ <<
                                                           " meters with the distance function '" << radiation_distance_function_ << "'.");
    }

    /// Print message value parameters and make it logarithmic if needed
    ROS_INFO_STREAM("Radiation sensor value range (w/o logarithm): [" << radiation_msg_val_min_ << ", " <<
                                                                      radiation_msg_val_max_ << "]");
    if(radiation_msg_use_log_){
      ROS_INFO_STREAM("Radiation sensor value range (with logarithm): [" <<
                      (log(radiation_msg_val_min_) < 0.0 ? 0.0 : log(radiation_msg_val_min_)) << ", " <<
                      log(radiation_msg_val_max_) << "]");
    }

    /// Print resolution parameters
    ROS_INFO_STREAM("Radiation angular resolution: (y=" << radiation_ang_res_y_ << "; z=" << radiation_ang_res_z_ << ")");

    /// Set color map by given color scheme
    bool color_map_scheme_valid = false;
    if (color_map_scheme == "rainbow") {
      color_map_.reset(new RainbowColorMap());
      color_map_scheme_valid = true;
    } else if (color_map_scheme == "inverse_rainbow") {
      color_map_.reset(new InverseRainbowColorMap());
      color_map_scheme_valid = true;
    } else if (color_map_scheme == "grayscale") {
      color_map_.reset(new GrayscaleColorMap());
      color_map_scheme_valid = true;
    } else if (color_map_scheme == "inverse_grayscale") {
      color_map_.reset(new InverseGrayscaleColorMap());
      color_map_scheme_valid = true;
    } else if (color_map_scheme == "ironbow") {
      color_map_.reset(new IronbowColorMap());
      color_map_scheme_valid = true;
    }
    if (color_map_scheme_valid) {
      ROS_INFO_STREAM("Color scheme for color map: " << color_map_scheme);
    } else {
      ROS_ERROR_STREAM("Invalid color scheme for color map: " << color_map_scheme);
      ROS_INFO_STREAM("Use one of the following commands for 'intensity_colormap': "<<
                      "rainbow, inverse_rainbow, grayscale, inverse_grayscale, ironbow");
    }
  }

  void RadioNuclearMapperServer::updateMesh() {
    TsdfServer::updateMesh();

    // Now recolor the mesh...
    timing::Timer publish_mesh_timer("radiation_mesh/publish");
    recolorVoxbloxMeshMsgByRadioNuclearIntensity(*intensity_layer_, color_map_,
                                     &cached_mesh_msg_);
    radiation_mesh_pub_.publish(cached_mesh_msg_);
    publish_mesh_timer.Stop();
  }

  void RadioNuclearMapperServer::publishPointclouds() {
    // Create a pointcloud with radiation = intensity.
    pcl::PointCloud<pcl::PointXYZI> pointcloud;

    createIntensityPointcloudFromIntensityLayer(*intensity_layer_, &pointcloud);

    pointcloud.header.frame_id = world_frame_;
    radiation_pointcloud_pub_.publish(pointcloud);

    TsdfServer::publishPointclouds();
  }

  void RadioNuclearMapperServer::radiationSensorCallback(const abc_msgs_fkie::MeasurementRawConstPtr& msg) {
    CHECK(intensity_layer_);
    CHECK(rnm_integrator_);
    CHECK(msg);

    // Get value from radiation sensor subscriber message
    float radiation_sensor_value = (float)msg->value;
    if (radiation_sensor_value < radiation_msg_val_min_) {
      ROS_WARN_STREAM("Radiation sensor value is smaller than minimum: " <<
                      radiation_sensor_value << " <" << radiation_msg_val_min_ << ")");
    }
    if (radiation_sensor_value > radiation_msg_val_max_) {
      ROS_WARN_STREAM("Radiation sensor value is higher than maximum:" <<
                      radiation_sensor_value << ">" << radiation_msg_val_max_ << ")");
    }

    // Look up transform
    Transformation T_G_C;
    if (!transformer_.lookupTransform(radiation_sensor_frame_id_, world_frame_, msg->header.stamp, &T_G_C)) {
      ROS_WARN_THROTTLE(10, "Failed to look up intensity transform!");
      return;
    }

    // Pre-allocate the bearing vectors and intensities.
    Pointcloud bearing_vectors;
    bearing_vectors.reserve(radiation_ang_res_z_ * radiation_ang_res_y_ + 1);

    for (int i = 0; i < radiation_ang_res_y_; i++) {
      float beta = (float)i / (float)radiation_ang_res_y_ * 2.0 * M_PI;
      for (int j = 0; j < radiation_ang_res_z_; j++) {
        float gamma = (float)j / (float)radiation_ang_res_z_ * 2.0 * M_PI;

        //  |  cos(c) -sin(c)    0    |   |  cos(b)    0     sin(b) |   | 1 |   | cos(b) * cos(c) |
        //  |  sin(c)  cos(c)    0    | * |    0       1       0    | * | 0 | = | cos(b) * sin(c) |
        //  |    0       0       1    |   | -sin(b)    0     cos(b) |   | 0 |   |     -sin(b)     |

        Ray bearing_vector = Point(cos(beta) * cos(gamma), cos(beta) * sin(gamma), -sin(beta));
        bearing_vectors.push_back(bearing_vector); // .normalized()
      }
    }
    // Put this into the integrator.
    rnm_integrator_->addRadiationSensorValueBearingVectors(
        T_G_C.getPosition(), bearing_vectors, radiation_sensor_value);
  }
}  // namespace voxblox
