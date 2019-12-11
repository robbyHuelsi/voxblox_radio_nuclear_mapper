#include "voxblox_ros/radio_nuclear_mapper_server.h"

namespace voxblox {

  RadioNuclearMapperServer::RadioNuclearMapperServer(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private):
  IntensityServer(nh, nh_private) {

    cache_mesh_ = true;

    intensity_layer_.reset(new Layer<IntensityVoxel>(tsdf_map_->getTsdfLayer().voxel_size(),
                                                        tsdf_map_->getTsdfLayer().voxels_per_side()));
    rnm_integrator_.reset(new RadioNuclearMapperIntegrator(tsdf_map_->getTsdfLayer(),
                                                           intensity_layer_.get()));

    // Get ROS params:  // TODO: Find a better solution instead min and max value
    float intensity_min_value = 10.0f;
    float intensity_max_value = 40.0f;
    nh_private_.param("intensity_min_value", intensity_min_value, intensity_min_value);
    nh_private_.param("intensity_max_value", intensity_max_value, intensity_max_value);

    FloatingPoint intensity_max_distance = rnm_integrator_->getMaxDistance();
    nh_private_.param("intensity_max_distance", intensity_max_distance, intensity_max_distance);
    rnm_integrator_->setMaxDistance(intensity_max_distance);

    // Get ROS params for radiological nuclear mapper
    std::string radiation_sensor_topic;
    nh_private_.param<std::string>("radiation_sensor_topic", radiation_sensor_topic, "");
    nh_private_.param<std::string>("radiation_sensor_frame_id", radiation_sensor_frame_id_, "");
    nh_private_.param("radiation_msg_val_min", radiation_msg_val_min_, 0.0f);
    nh_private_.param("radiation_msg_val_max", radiation_msg_val_max_, 100000.0f);
    nh_private_.param("radiation_msg_use_log", radiation_msg_use_log_, false);
    nh_private_.param("radiation_ang_res_z", radiation_ang_res_y, 100);
    nh_private_.param("radiation_ang_res_z", radiation_ang_res_z, 100);
    nh_private_.param("radiation_image_dispersion", radiation_image_dispersion_, 1.0f);

    ROS_INFO_STREAM("Radiation sensor data topic: " << radiation_sensor_topic);
    ROS_INFO_STREAM("Radiation sensor frame id: " << radiation_sensor_frame_id_);
    ROS_INFO_STREAM("Radiation sensor value range (no log): [" << radiation_msg_val_min_ << ", " <<
                                                               radiation_msg_val_max_ << "]");
    if(radiation_msg_use_log_){
      radiation_msg_val_min_ = log(radiation_msg_val_min_);
      radiation_msg_val_min_ = radiation_msg_val_min_ < 0.0 ? 0.0 : radiation_msg_val_min_;
      radiation_msg_val_max_ = log(radiation_msg_val_max_);
      ROS_INFO_STREAM("Radiation sensor value range (log): [" << radiation_msg_val_min_ << ", " <<
                                                              radiation_msg_val_max_ << "]");
    }
    ROS_INFO_STREAM("Radiation image size: " << radiation_ang_res_y << "*" << radiation_ang_res_z);

    // Check parameter validity
    if (radiation_sensor_topic.empty()) {
      ROS_ERROR("ROS topic for radiological nuclear sensor data not specified.");
      ROS_INFO("Use parameter 'radiation_sensor_topic' next time.");
//    ros::shutdown();
//    return EXIT_FAILURE;
    } else if (radiation_sensor_frame_id_.empty()) {
      ROS_ERROR("Frame id of radiological nuclear sensor not specified.");
      ROS_INFO("Use parameter 'radiation_sensor_frame_id' next time.");
//    ros::shutdown();
//    return EXIT_FAILURE;
    }

    // Setup radiological nuclear mapper parameters
//    radiation_image_max_dist_ = distance(radiation_ang_res_y/2, radiation_ang_res_z/2);
//  radiation_msg_step_ = 0;

    // Publishers for output.
    intensity_pointcloud_pub_ =
        nh_private_.advertise<pcl::PointCloud<pcl::PointXYZI> >(
            "intensity_pointcloud", 1, true);
    intensity_mesh_pub_ =
        nh_private_.advertise<voxblox_msgs::Mesh>("intensity_mesh", 1, true);

    color_map_.reset(new IronbowColorMap());  // TODO
    color_map_->setMinValue(intensity_min_value);
    color_map_->setMaxValue(intensity_max_value);

    // Subscribe nuclear intensity
    radiation_sensor_sub_ = nh_private_.subscribe(
        radiation_sensor_topic, 1, &RadioNuclearMapperServer::radiationSensorCallback, this);
//  radiation_sensor_sub_ = nh_private_.subscribe(
//      radiation_sensor_topic, 1, RadioNuclearMapperServer::radiationSensorCallback);

  }

//  float RadioNuclearMapperServer::distance(int x, int y) {  // TODO: Remove
//    return std::sqrt(x*x + y*y);
//  }
//
//  float RadioNuclearMapperServer::squared_distance(int x, int y) {
//    return x*x + y*y;
//  }


  void RadioNuclearMapperServer::updateMesh() {
    TsdfServer::updateMesh();

    // Now recolor the mesh...
    timing::Timer publish_mesh_timer("intensity_mesh/publish");
    recolorVoxbloxMeshMsgByRadioNuclearIntensity(*intensity_layer_, color_map_,
                                     &cached_mesh_msg_);
    intensity_mesh_pub_.publish(cached_mesh_msg_);
    publish_mesh_timer.Stop();
  }

//  void RadioNuclearMapperServer::publishPointclouds() {
//    // Create a pointcloud with temperature = intensity.
//    pcl::PointCloud<pcl::PointXYZI> pointcloud;
//
//    createIntensityPointcloudFromIntensityLayer(*intensity_layer_, &pointcloud);
//
//    pointcloud.header.frame_id = world_frame_;
//    intensity_pointcloud_pub_.publish(pointcloud);
//
//    TsdfServer::publishPointclouds();
//  }

  void RadioNuclearMapperServer::radiationSensorCallback(const abc_msgs_fkie::MeasurementRawConstPtr& msg) {
    CHECK(intensity_layer_);
    CHECK(rnm_integrator_);
    CHECK(msg);

    // TODO: Remove
    //ROS_INFO_STREAM("Msg value: " << msg->value);
    // Simulate nuclear intensity
//  float intensity = 0.5;
//  float intensity = sin((float)msg->header.seq*3.1415/10.0) / 2;

    //Get intensity from radiation sensor subscriber message
    float intensity = (float)msg->value;

    // Use logarithmic mapping if needed
    if(radiation_msg_use_log_){
      intensity = log(intensity);
    }

    // Normalize between 0.0 and 1.0
    intensity = radiation_msg_val_min_ + intensity / (radiation_msg_val_max_ - radiation_msg_val_min_);

    // TODO: Braucht man das noch?
    if (intensity < 0.0) {
      intensity = 0.0;
      //ROS_WARN_STREAM("Radiation sensor value is smaller than minimum (" << radiation_msg_val_min_ << ")");
    }
    if (intensity > 1.0) {
      intensity = 1.0;
      //ROS_WARN_STREAM("Radiation sensor value is higher than maximum (" << radiation_msg_val_max_ << ")");
    }
//  float intensity = (float)msg->value;
//    ROS_INFO_STREAM("Intensity (" << (radiation_msg_use_log_?"log":"no log") << "): " << intensity);

    // Look up transform
    Transformation T_G_C;
    if (!transformer_.lookupTransform(radiation_sensor_frame_id_, world_frame_, msg->header.stamp, &T_G_C)) {
      ROS_WARN_THROTTLE(10, "Failed to look up intensity transform!");
      return;
    }

    // Pre-allocate the bearing vectors and intensities.
    Pointcloud bearing_vectors;
    bearing_vectors.reserve(radiation_ang_res_z * radiation_ang_res_y + 1);
//    std::vector<float> intensities;
//    intensities.reserve(radiation_ang_res_z * radiation_ang_res_y + 1);  // TODO: Removw

    for (int i = 0; i < radiation_ang_res_z; i++) {
      float alpha = (float)i / (float)radiation_ang_res_z * 2.0 * M_PI;
      for (int j = 0; j < radiation_ang_res_y; j++) {
        float beta = (float)j / (float)radiation_ang_res_y * 2.0 * M_PI;
        //  |  cos(b) -sin(b)    0    |   |  cos(a)    0     sin(a) |   | 1 |   | cos(a) * cos(b) |
        //  |  sin(b)  cos(b)    0    | * |    0       1       0    | * | 0 | = | cos(a) * sin(b) |
        //  |    0       0       1    |   | -sin(a)    0     cos(a) |   | 0 |   |     -sin(a)     |
        Ray bearing_vector = Point(cos(alpha) * cos(beta), cos(alpha) * sin(beta), -sin(alpha));
//        ROS_INFO_STREAM("check: " << bearing_vector - bearing_vector.normalized()); // TODO: Remove
        bearing_vectors.push_back(bearing_vector); // .normalized()
      }
    }
    // Put this into the integrator.
    rnm_integrator_->addIntensityBearingVectors(
        T_G_C.getPosition(), bearing_vectors, intensity);

//  radiation_msg_step_++;  // TODO: Remove

  }
}  // namespace voxblox
