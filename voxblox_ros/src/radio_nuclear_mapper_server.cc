#include "voxblox_ros/radio_nuclear_mapper_server.h"

namespace voxblox {

  RadioNuclearMapperServer::RadioNuclearMapperServer(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private):
  IntensityServer(nh, nh_private), focal_length_px_(400.0f), subsample_factor_(12) {

    cache_mesh_ = true;

    intensity_layer_.reset(new Layer<IntensityVoxel>(tsdf_map_->getTsdfLayer().voxel_size(),
                                                        tsdf_map_->getTsdfLayer().voxels_per_side()));
    intensity_integrator_.reset(new IntensityIntegrator(tsdf_map_->getTsdfLayer(),
                                                           intensity_layer_.get()));

    // Get ROS params:
    nh_private_.param("intensity_focal_length", focal_length_px_, focal_length_px_);
    nh_private_.param("subsample_factor", subsample_factor_, subsample_factor_);

    float intensity_min_value = 10.0f;
    float intensity_max_value = 40.0f;
    nh_private_.param("intensity_min_value", intensity_min_value, intensity_min_value);
    nh_private_.param("intensity_max_value", intensity_max_value, intensity_max_value);

    FloatingPoint intensity_max_distance = intensity_integrator_->getMaxDistance();
    nh_private_.param("intensity_max_distance", intensity_max_distance, intensity_max_distance);
    intensity_integrator_->setMaxDistance(intensity_max_distance);

    // Get ROS params for radiological nuclear mapper
    std::string radiation_sensor_topic;
    nh_private_.param<std::string>("radiation_sensor_topic", radiation_sensor_topic, "");
    nh_private_.param<std::string>("radiation_sensor_frame_id", radiation_sensor_frame_id_, "");
    nh_private_.param("radiation_msg_val_min", radiation_msg_val_min_, 0.0f);
    nh_private_.param("radiation_msg_val_max", radiation_msg_val_max_, 100000.0f);
    nh_private_.param("radiation_msg_use_log", radiation_msg_use_log_, false);
    nh_private_.param("radiation_image_image_width", radiation_image_width_, 100);
    nh_private_.param("radiation_image_image_height", radiation_image_height_, 100);
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
    ROS_INFO_STREAM("Radiation image size: " << radiation_image_width_ << "*" << radiation_image_height_);

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
    radiation_image_max_dist_ = distance(radiation_image_width_/2, radiation_image_height_/2);
    intensity_test_image_header_.frame_id = radiation_sensor_frame_id_;
//  radiation_msg_step_ = 0;

    // Publishers for output.
    intensity_pointcloud_pub_ =
        nh_private_.advertise<pcl::PointCloud<pcl::PointXYZI> >(
            "intensity_pointcloud", 1, true);
    intensity_mesh_pub_ =
        nh_private_.advertise<voxblox_msgs::Mesh>("intensity_mesh", 1, true);

    color_map_.reset(new IronbowColorMap());
    color_map_->setMinValue(intensity_min_value);
    color_map_->setMaxValue(intensity_max_value);

//  // Set up subscriber.
//  intensity_image_sub_ = nh_private_.subscribe(
//      "intensity_image", 1, &RadioNuclearMapperServer::intensityImageCallback, this);


    intensity_test_image_publisher_ = nh_private_.advertise<sensor_msgs::Image>("intensity_test_image", 1);

    // Subscribe nuclear intensity
    radiation_sensor_sub_ = nh_private_.subscribe(
        radiation_sensor_topic, 1, &RadioNuclearMapperServer::radiationSensorCallback, this);
//  radiation_sensor_sub_ = nh_private_.subscribe(
//      radiation_sensor_topic, 1, RadioNuclearMapperServer::radiationSensorCallback);

  }

  float RadioNuclearMapperServer::distance(int x, int y) {
    return std::sqrt(x*x + y*y);
  }

  float RadioNuclearMapperServer::squared_distance(int x, int y) {
    return x*x + y*y;
  }


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

  void RadioNuclearMapperServer::intensityImageCallback(
      const sensor_msgs::ImageConstPtr& image) {
    CHECK(intensity_layer_);
    CHECK(intensity_integrator_);
    CHECK(image);
    // Look up transform first...
    Transformation T_G_C;
    if (!transformer_.lookupTransform(image->header.frame_id, world_frame_,
                                      image->header.stamp, &T_G_C)) {
      ROS_WARN_THROTTLE(10, "Failed to look up intensity transform!");
      return;
    }

    cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(image);

    CHECK(cv_ptr);

    const size_t num_pixels =
        cv_ptr->image.rows * cv_ptr->image.cols / subsample_factor_;

    float half_row = cv_ptr->image.rows / 2.0;
    float half_col = cv_ptr->image.cols / 2.0;

    // Pre-allocate the bearing vectors and intensities.
    Pointcloud bearing_vectors;
    bearing_vectors.reserve(num_pixels + 1);
    std::vector<float> intensities;
    intensities.reserve(num_pixels + 1);

    size_t k = 0;
    size_t m = 0;
    for (int i = 0; i < cv_ptr->image.rows; i++) {
      const float* image_row = cv_ptr->image.ptr<float>(i);
      for (int j = 0; j < cv_ptr->image.cols; j++) {
        if (m % subsample_factor_ == 0) {
          // Rotates the vector pointing from the camera center to the pixel
          // into the global frame, and normalizes it.
          bearing_vectors.push_back(
              T_G_C.getRotation().toImplementation() *
              Point(j - half_col, i - half_row, focal_length_px_).normalized());
          intensities.push_back(image_row[j]);
          k++;
        }
        m++;
      }
    }

    // Put this into the integrator.
    intensity_integrator_->addIntensityBearingVectors(
        T_G_C.getPosition(), bearing_vectors, intensities);
  }

  void RadioNuclearMapperServer::radiationSensorCallback(
//    abc_msgs_fkie::MeasurementRawConstPtr msg) {
      const abc_msgs_fkie::MeasurementRawConstPtr& msg) {
    CHECK(intensity_layer_);
    CHECK(intensity_integrator_);
    CHECK(msg);
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

    if (intensity < 0.0) {
      intensity = 0.0;
      //ROS_WARN_STREAM("Radiation sensor value is smaller than minimum (" << radiation_msg_val_min_ << ")");
    }
    if (intensity > 1.0) {
      intensity = 1.0;
      //ROS_WARN_STREAM("Radiation sensor value is higher than maximum (" << radiation_msg_val_max_ << ")");
    }
//  float intensity = (float)msg->value;
    ROS_INFO_STREAM("Intensity (" << (radiation_msg_use_log_?"log":"no log") << "): " << intensity);

    // Look up transform
    Transformation T_G_C;
    if (!transformer_.lookupTransform(radiation_sensor_frame_id_, world_frame_, msg->header.stamp, &T_G_C)) {
      ROS_WARN_THROTTLE(10, "Failed to look up intensity transform!");
      return;
    }

    // Initialize image with no intensity
    cv::Mat float_img = cv::Mat(radiation_image_height_, radiation_image_width_, CV_32FC1, cv::Scalar::all(0));
    cv::Mat uint_img = cv::Mat(radiation_image_height_, radiation_image_width_, CV_8UC1, cv::Scalar::all(0));

    // Create intensity image
    for (int row = 0; row < float_img.rows; ++row) {
      for (int col = 0; col < float_img.cols; ++col) {
        //float sq_dist = squared_distance(row - radiation_image_height_ / 2, col - radiation_image_width_ / 2) / radiation_image_max_dist_; //TODO
        //float sq_dist = distance(row - image_height_ / 2, col - image_width_ / 2) / max_dist_;
        //float value = (1.0-radiation_image_dispersion_ *sq_dist) * intensity;
        float value = intensity;
        value = value < 0.0 ? 0.0 : (value > 1.0 ? 1.0 : value);
        float_img.at<float>(row, col, 0) = value;
        uint_img.at<unsigned int>(row, col, 0) = (unsigned int)round(value * 255.0);
      }
    }

    const size_t num_pixels = radiation_image_height_ * radiation_image_width_ / subsample_factor_;

    float half_row = radiation_image_height_ / 2.0;
    float half_col = radiation_image_width_ / 2.0;

    // Pre-allocate the bearing vectors and intensities.
    Pointcloud bearing_vectors;
    bearing_vectors.reserve(num_pixels + 1);
    std::vector<float> intensities;
    intensities.reserve(num_pixels + 1);

////        ROS_INFO_STREAM("img dtype: " << img.type());
////        ROS_INFO_STREAM("img middle val: " << img.at<uchar>((int)image_height_/2, (int)image_width_/2));

    // Define header and send intensity image to voxblox
////    header_.seq = step_;
////    header_.stamp = ros::Time::now();
    intensity_test_image_header_.seq = msg->header.seq;
    intensity_test_image_header_.stamp = msg->header.stamp;
    intensity_test_image_publisher_.publish(cv_bridge::CvImage(intensity_test_image_header_, "mono8", uint_img).toImageMsg());

    size_t k = 0;
    size_t m = 0;
    for (int i = 0; i < float_img.rows; i++) {
      const float* image_row = float_img.ptr<float>(i);
      for (int j = 0; j < float_img.cols; j++) {
        if (m % subsample_factor_ == 0) {
          // Rotates the vector pointing from the camera center to the pixel
          // into the global frame, and normalizes it.
          bearing_vectors.push_back(
              T_G_C.getRotation().toImplementation() *
              Point(j - half_col, i - half_row, focal_length_px_).normalized());
          intensities.push_back(image_row[j]);
          k++;
        }
        m++;
      }
    }

    // Put this into the integrator.
    intensity_integrator_->addIntensityBearingVectors(
        T_G_C.getPosition(), bearing_vectors, intensities);

//  radiation_msg_step_++;

  }


}  // namespace voxblox
