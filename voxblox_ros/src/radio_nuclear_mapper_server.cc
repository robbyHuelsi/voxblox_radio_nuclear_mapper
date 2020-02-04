/// Plagiarism Notice:
/// The code in this file comes from file intensity_server.cc
/// and has been adapted for the special purpose of radiation mapping by Robert Hülsmann.
/// New/edited variables or methods of class RadioNuclearMapperServer are marked with the comment "RH" IN HEADER FILE.
/// IN ADDITION, new/edited lines of code in adopted methods are marked in the same way IN THIS FILE
/// (except simple renaming "intensity" to "radiation" etc.).

#include "voxblox_ros/radio_nuclear_mapper_server.h"

namespace voxblox {
  RadioNuclearMapperServer::RadioNuclearMapperServer(const ros::NodeHandle& nh,
                                                     const ros::NodeHandle& nh_private)
      : TsdfServer(nh, nh_private) {
    /// To recolor und publish cached parts of mesh, turn cache_mesh_ on
    cache_mesh_ = true;

    radiation_layer_.reset(new Layer<RadiationVoxel>(tsdf_map_->getTsdfLayer().voxel_size(),
                                                     tsdf_map_->getTsdfLayer().voxels_per_side()));
    radiation_integrator_.reset(new RadioNuclearMapperIntegrator(tsdf_map_->getTsdfLayer(),
                                                                 radiation_layer_.get()));

    /// Get ROS params for radiation sensor (RH)
    getServerConfigFromRosParam(nh_private_);

    /// Forward some paramters to integrator (RH)
    radiation_integrator_->setMaxDistance(radiation_max_distance_);

    /// Set min and max value for color map (RH)
    setCMExtrValByMostExtrPossible(radiation_msg_val_min_, radiation_msg_val_max_, radiation_distance_function_,
                                   radiation_use_logarithm_, radiation_max_distance_, radiation_color_map_);

    /// Generate Bearing Vectors to project radiation intensity onto voxels (RH)
    generateBearingVectors(radiation_bearing_vector_num_, bearing_vectors_);

    /// Publishers for output
    radiation_pointcloud_pub_ =
        nh_private_.advertise<pcl::PointCloud<pcl::PointXYZI> >(
            "radiation_pointcloud", 1, true);
    radiation_mesh_pub_ =
        nh_private_.advertise<voxblox_msgs::Mesh>("radiation_mesh", 1, true);

    /// Subscribe radiation intensity (whole block: RH)
    radiation_sensor_callback_counter_ = 0;
    radiation_sensor_sub_ = nh_private_.subscribe(
        radiation_sensor_topic_, 1, &RadioNuclearMapperServer::radiationSensorCallback, this);

    /// Subscribe save mesh trigger (RH)
    save_mesh_trigger_sub_ = nh_private_.subscribe(
        save_mesh_trigger_topic_, 1, &RadioNuclearMapperServer::saveMeshTriggerCallback, this);
  }

  void RadioNuclearMapperServer::getServerConfigFromRosParam(const ros::NodeHandle& nh_private) {
    /// Define pre-sets of parameters
    radiation_sensor_topic_ = "";
    radiation_sensor_frame_id_ = "";
    radiation_max_distance_ = radiation_integrator_->getMaxDistance();
    radiation_distance_function_name_ = "constant";
    radiation_msg_val_min_ = 0.0;
    radiation_msg_val_max_ = 100000.0;
    radiation_use_logarithm_ = false;
    radiation_bearing_vector_num_ = 10000;
    radiation_color_map_scheme_name_ = "ironbow";
    save_mesh_trigger_topic_ = "";

    /// Get ROS parameters
    nh_private.param<std::string>("radiation_sensor_topic",
                                  radiation_sensor_topic_, radiation_sensor_topic_);
    nh_private.param<std::string>("radiation_sensor_frame_id",
                                  radiation_sensor_frame_id_, radiation_sensor_frame_id_);
    nh_private.param("radiation_max_distance", radiation_max_distance_, radiation_max_distance_);
    nh_private.param<std::string>("radiation_distance_function",
                                  radiation_distance_function_name_, radiation_distance_function_name_);
    nh_private.param("radiation_use_logarithm", radiation_use_logarithm_, radiation_use_logarithm_);
    nh_private.param("radiation_msg_val_min", radiation_msg_val_min_, radiation_msg_val_min_);
    nh_private.param("radiation_msg_val_max", radiation_msg_val_max_, radiation_msg_val_max_);
    nh_private.param("radiation_bearing_vector_num",
                     radiation_bearing_vector_num_, radiation_bearing_vector_num_);
    nh_private.param("radiation_colormap",
                     radiation_color_map_scheme_name_, radiation_color_map_scheme_name_);
    nh_private.param<std::string>("save_mesh_trigger_topic",
                                  save_mesh_trigger_topic_, save_mesh_trigger_topic_);

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

    /// Set color map by given color scheme
    setColorMapScheme(radiation_color_map_scheme_name_, radiation_color_map_);

    /// Print message value parameters and make it logarithmic if needed
    ROS_INFO_STREAM("Radiation sensor value range (w/o logarithm): [" <<
                    radiation_msg_val_min_ << ", " << radiation_msg_val_max_ << "]");
    if(radiation_use_logarithm_){
      ROS_INFO_STREAM("Radiation sensor value range (with logarithm): " <<
                      "[" << (log(radiation_msg_val_min_) < 0.0 ? 0.0 : log(radiation_msg_val_min_)) <<
                      ", " << log(radiation_msg_val_max_) << "]");
    }

    /// Set radiation distance function by name as string
    getRadiationDistanceFunctionByName(radiation_distance_function_name_,
                                       radiation_distance_function_);

    /// Print resolution parameters
    ROS_INFO_STREAM("Number of bearing vectors: " << radiation_bearing_vector_num_);

    /// Check parameter validity for save mesh trigger parameter and print it
    if(!save_mesh_trigger_topic_.empty()){
      ROS_INFO_STREAM("Save generated mesh with message at this topic: " << save_mesh_trigger_topic_);
    }
  }

  bool RadioNuclearMapperServer::setColorMapScheme(const std::string& color_map_scheme_name,
                                                   std::shared_ptr<ColorMap>& color_map){
    /// Try to select color map scheme by incomming string (this block was mainly copied from tsdf_server.cc)
    bool color_map_scheme_valid = false;
    if (color_map_scheme_name == "rainbow") {
      color_map.reset(new RainbowColorMap());
      color_map_scheme_valid = true; /// RH
    } else if (color_map_scheme_name == "inverse_rainbow") {
      color_map.reset(new InverseRainbowColorMap());
      color_map_scheme_valid = true; /// RH
    } else if (color_map_scheme_name == "grayscale") {
      color_map.reset(new GrayscaleColorMap());
      color_map_scheme_valid = true; /// RH
    } else if (color_map_scheme_name == "inverse_grayscale") {
      color_map.reset(new InverseGrayscaleColorMap());
      color_map_scheme_valid = true; /// RH
    } else if (color_map_scheme_name == "ironbow") {
      color_map.reset(new IronbowColorMap());
      color_map_scheme_valid = true; /// RH
    } else if (color_map_scheme_name == "traffic-light") { /// RH
      color_map.reset(new TrafficLightColorMap()); /// RH
      color_map_scheme_valid = true; /// RH
    }

    /// Print hints
    if (color_map_scheme_valid) {
      ROS_INFO_STREAM("Color scheme for color map: " << color_map_scheme_name);
    } else {
      ROS_ERROR_STREAM("Invalid color scheme for color map: " << color_map_scheme_name);
      ROS_INFO_STREAM("Use one of the following commands for 'intensity_colormap': "<<
                      "rainbow, inverse_rainbow, grayscale, inverse_grayscale, ironbow, traffic-light");
    }
    return color_map_scheme_valid;
  }

  bool RadioNuclearMapperServer::getRadiationDistanceFunctionByName(const std::string& distance_function_name,
                                                                    RDFType& rad_dist_func){
    /// Check if the incoming string is allowed
    std::vector<std::string> allowed_funcs = {"increasing", "decreasing", "constant"};
    if(std::find(allowed_funcs.begin(), allowed_funcs.end(), distance_function_name) == allowed_funcs.end()){
      rad_dist_func = &RadioNuclearMapperServer::rad_dist_func_infinity;
      /// Distance function string is not allowed
      ROS_ERROR("Radiation distance function is called by a not supported string.");
      /// Generate hint for user
      std::stringstream allowed_funcs_ss = std::stringstream();
      for(std::vector<std::string>::iterator it = allowed_funcs.begin(); it != allowed_funcs.end(); ++it) {
        allowed_funcs_ss << *it << ", ";
      }
      ROS_INFO_STREAM("Use one of the following commands for 'radiation_distance_function': " <<
                      allowed_funcs_ss.str().substr(0, allowed_funcs_ss.str().length()-2));
      return false;
    }else{
      /// Distance function string is allowed -> select the right one
      if (distance_function_name == "increasing" or distance_function_name == "i"){
        rad_dist_func = &RadioNuclearMapperServer::rad_dist_func_increasing;
      }else if (distance_function_name == "decreasing" or distance_function_name == "d"){
        rad_dist_func = &RadioNuclearMapperServer::rad_dist_func_decreasing;
      }else if (distance_function_name == "constant" or distance_function_name == "c"){
        rad_dist_func = &RadioNuclearMapperServer::rad_dist_func_constant;
      }
      ROS_INFO_STREAM("Radiation will map in a radius of " << radiation_max_distance_ <<
                      " meters with the distance function '" << distance_function_name << "'.");
      return true;
    }


  }

  /**
   * Set minimum and maximum of color map using the most extreme possible values
   * given by the combination of maximum and minimum value in sensor message,
   * selected maximum distance and selected radiation distance function and
   * the choice whether to use logarithm or not.
   */
  void RadioNuclearMapperServer::setCMExtrValByMostExtrPossible(const float radiation_msg_val_min,
                                                                const float radiation_msg_val_max,
                                                                RDFType& rad_dist_func,
                                                                const bool use_logarithm,
                                                                const float radiation_max_distance,
                                                                const std::shared_ptr<ColorMap>& color_map){
    /// Combine values and put them into a vector
    std::vector<float> intensity_extreme_values;
    float radiation_msg_extreme_values[] = {radiation_msg_val_min, radiation_msg_val_max};
    float distance_extreme_values[] = {0.0, radiation_max_distance};
    float tmp_intensity;
    for(float ex_val: radiation_msg_extreme_values){
      for(float dist: distance_extreme_values){
        calcIntensity(ex_val, dist, rad_dist_func, use_logarithm, tmp_intensity);
        intensity_extreme_values.insert(intensity_extreme_values.end(), 1, tmp_intensity);
      }
    }

    /// Get extreme values from vector and write them into color map
    float intensity_min_value = *std::min_element(intensity_extreme_values.begin(), intensity_extreme_values.end());
    float intensity_max_value = *std::max_element(intensity_extreme_values.begin(), intensity_extreme_values.end());
    color_map->setMinValue(intensity_min_value);
    color_map->setMaxValue(intensity_max_value);

    //Print extreme values
    ROS_INFO_STREAM("Color map value range is: [" << intensity_min_value << ", " << intensity_max_value << "]");
  }

  void RadioNuclearMapperServer::generateBearingVectors(const int n, Pointcloud& bearing_vectors) {
    /// Idea from "A New Computationally Effifficient Method for Spacing n Points on a Sphere"
    /// by Jonathan Kogan, Columbia Grammar and Preparatory School, New York
    /// https://scholar.rose-hulman.edu/cgi/viewcontent.cgi?article=1387&context=rhumj

    // TODO: Add comments

    ROS_INFO_STREAM("Generating bearing vectors..." << std::flush);

    double wtf = 0.1 + 1.2 * n;  // TODO: Rename variables
    double start = (-1.0 + 1.0 / (n - 1.0));
    double increment = (2.0 - 2.0 / (n - 1.0)) / (n - 1.0);
    
    bearing_vectors.clear();
    bearing_vectors.reserve(n);
    double s, phi, theta, x, y, z;

    std::stringstream vec_ss;
    vec_ss << "[";
    for (int j = 0; j < n; j++) {
      s = start + (double)j * increment;
      phi = s * wtf;
      theta = M_PI / 2.0 * copysign(1, s) * (1.0 - sqrt(1.0 - abs(s)));
      x = cos(phi) * cos(theta);
      y = sin(phi) * cos(theta);
      z = sin(theta);
      bearing_vectors.push_back(Point(x, y, z));
      vec_ss << "(" << x << "," << y << "," << z << (j == n-1?")":"),");
    }
    vec_ss << "]";

    std::cout <<" Done." << std::endl;
//    ROS_INFO_STREAM(vec_ss.str() << std::endl);
  }

  inline void RadioNuclearMapperServer::calcIntensity(const float sensor_value, const float distance,
                                               RDFType& rad_dist_func, const bool use_logarithm, float& intensity) {
    /// Apply desired radiation distance function and calculate intensity by multiplying with sensor value
    /// Applying function looks crazy, but it works (https://stackoverflow.com/a/1486279)
    intensity = sensor_value * (*this.*rad_dist_func)(distance);

    /// Use logarithmic mapping if needed
    if(use_logarithm){
      intensity = log(intensity);

      /// To avoid extremely small intensity values,
      /// intensity below 1.0 (without logarithm) is limited to 0.0 (with logarithm)
      intensity = intensity < 0.0 ? 0.0 : intensity;
    }
  }

  inline Color RadioNuclearMapperServer::getColorForVoxelPointer(const RadiationVoxel* voxel,
                                                          const std::shared_ptr<ColorMap>& color_map,
                                                          RDFType& rad_dist_func, const bool use_logarithm){
    Color c;
    if (voxel != nullptr) {
      float intensity;
      calcIntensity(voxel->intensity, voxel->distance, rad_dist_func, use_logarithm, intensity);
      c = color_map->colorLookup(intensity);
    } else {
      /// If voxel cannot found (no voxel at requested position) color black
      c = Color(0.0, 0.0, 0.0, 0.0);
    }
    return c;
  }

  inline void RadioNuclearMapperServer::recolorVoxbloxMeshMsgByRadiationIntensity(
      const Layer<RadiationVoxel>& intensity_layer,
      const std::shared_ptr<ColorMap>& color_map,
      RDFType& rad_dist_func, const bool use_logarithm,
      voxblox_msgs::Mesh* mesh_msg) {
    CHECK_NOTNULL(mesh_msg);
    CHECK(color_map);

    /// Go over all the blocks in the mesh message.
    for (voxblox_msgs::MeshBlock& mesh_block : mesh_msg->mesh_blocks) {
      /// Look up verticies in the thermal layer.
      for (size_t vert_idx = 0u; vert_idx < mesh_block.x.size(); ++vert_idx) {
        /// Copied from https://github.com/ethz-asl/voxblox/blob/master/voxblox_rviz_plugin/src/voxblox_mesh_visual.cc
        /// line 40 - 56 (03.02.2020):
        // Each vertex is given as its distance from the blocks origin in units of
        // (2*block_size), see mesh_vis.h for the slightly convoluted
        // justification of the 2.
        constexpr float point_conv_factor = 2.0f / std::numeric_limits<uint16_t>::max();
        const float mesh_x =
            (static_cast<float>(mesh_block.x[vert_idx]) * point_conv_factor +
             static_cast<float>(mesh_block.index[0])) * mesh_msg->block_edge_length;
        const float mesh_y =
            (static_cast<float>(mesh_block.y[vert_idx]) * point_conv_factor +
             static_cast<float>(mesh_block.index[1])) * mesh_msg->block_edge_length;
        const float mesh_z =
            (static_cast<float>(mesh_block.z[vert_idx]) * point_conv_factor +
             static_cast<float>(mesh_block.index[2])) * mesh_msg->block_edge_length;

        /// Get color c from voxel v at position p
        Point p = Point(mesh_x, mesh_y, mesh_z);
        const RadiationVoxel* voxel = intensity_layer.getVoxelPtrByCoordinates(p);
        Color c = getColorForVoxelPointer(voxel, color_map, rad_dist_func, use_logarithm);

        /// Update mesh message color
        mesh_block.r[vert_idx] = c.r;
        mesh_block.g[vert_idx] = c.g;
        mesh_block.b[vert_idx] = c.b;
      }
    }
  }

  void RadioNuclearMapperServer::updateMesh() {
    /// Run updateMesh() of parent class TsdfServer
    TsdfServer::updateMesh();

    /// Use updated mesh, recolor and update it
    timing::Timer publish_mesh_timer("radiation_mesh/publish");
    recolorVoxbloxMeshMsgByRadiationIntensity(*radiation_layer_, radiation_color_map_,
                                              radiation_distance_function_, radiation_use_logarithm_,
                                              &cached_mesh_msg_);
    radiation_mesh_pub_.publish(cached_mesh_msg_);
    publish_mesh_timer.Stop();
  }

  bool RadioNuclearMapperServer::generateMesh() {
    /// Apply generateMesh() with preset parameters
    return generateMesh(radiation_distance_function_name_, radiation_use_logarithm_, radiation_color_map_scheme_name_);
  }

  bool RadioNuclearMapperServer::generateMesh(const std::string& distance_function_name,
                                              const bool use_logarithm,
                                              const std::string& color_map_scheme_name){
    timing::Timer generate_mesh_timer("radiation_mesh/generate");

    /// Get radiation distance function by given string
    RDFType rad_dist_func;
    getRadiationDistanceFunctionByName(distance_function_name, rad_dist_func);

    /// Create color map with wanted scheme
    std::shared_ptr<ColorMap> export_color_map;
    setColorMapScheme(color_map_scheme_name, export_color_map);
    setCMExtrValByMostExtrPossible(radiation_msg_val_min_, radiation_msg_val_max_, rad_dist_func, use_logarithm,
                                   radiation_max_distance_, export_color_map);

    /// Get mesh from parents (TSDF Server) mesh message
    Mesh mesh = Mesh(mesh_layer_->block_size(), Point::Zero());
    convertMeshLayerToMesh(*mesh_layer_, &mesh, true);
    float num_mesh_points = float(mesh.size());

    /// define a helper string
    std::string ident_str = distance_function_name + "-func_" + (use_logarithm ? "log" : "no-log") + "_";
    ident_str += color_map_scheme_name + "-cm";

    /// Go over all blocks in the mesh
    for (size_t i = 0; i < mesh.size(); i++) {

      /// Print status information
      if (i % 10 == 0){
        float percentage = std::round(float(i) / num_mesh_points * 1000.0) / 10.0;
        std::cout << "Recoloring (" << ident_str << "): " << percentage << " %    \r" <<std::flush;
      }

      /// Recolor mesh
      Point p = mesh.vertices[i];
      const RadiationVoxel* voxel = radiation_layer_->getVoxelPtrByCoordinates(p);
      Color c = getColorForVoxelPointer(voxel, export_color_map, rad_dist_func, use_logarithm);
      mesh.colors[i] = c;
    }

    std::cout << "Recoloring (" << ident_str << ") done.  " << std::endl;

    generate_mesh_timer.Stop();
    timing::Timer output_mesh_timer("radiation_mesh/output");

    /// Create file name including function, log, color map and time stamp
    time_t raw_time;
    struct tm * time_info;
    char time_buffer[80];
    time (&raw_time);
    time_info = localtime(&raw_time);
    strftime(time_buffer,sizeof(time_buffer),"%Y-%m-%d-%H-%M-%S",time_info);
    std::string time_str = time_buffer;
    std::string filename = ident_str + "_" + time_str + ".ply";

    /// Save mesh
    std::cout << "Exporting (" << ident_str << ") ... \r" <<std::flush;
    const bool success = outputMeshAsPly(filename, mesh);
    std::cout << "Exporting (" << ident_str << ") done." <<std::endl;
    output_mesh_timer.Stop();

    if (success) {
      ROS_INFO("Output file as PLY: %s", filename.c_str());
    } else {
      ROS_INFO("Failed to output mesh as PLY: %s", filename.c_str());
    }

    ROS_INFO_STREAM("Mesh Timings: " << std::endl << timing::Timing::Print());

    return true;
  }

  void RadioNuclearMapperServer::publishPointclouds() {
    /// Create a pointcloud with radiation = intensity.
    pcl::PointCloud<pcl::PointXYZI> pointcloud;

    createRadiationPointcloudFromRadiationLayer(*radiation_layer_, &pointcloud);

    pointcloud.header.frame_id = world_frame_;
    radiation_pointcloud_pub_.publish(pointcloud);

    TsdfServer::publishPointclouds();
  }

  void RadioNuclearMapperServer::radiationSensorCallback(const abc_msgs_fkie::MeasurementRawConstPtr& msg) {
    CHECK(radiation_layer_);
    CHECK(radiation_integrator_);
    CHECK(msg);

    size_t current_callback_num = radiation_sensor_callback_counter_++;

    /// Get value from radiation sensor subscriber message
    float radiation_sensor_value = (float)msg->value;
    ROS_INFO_STREAM(current_callback_num << ": New radiation value: " << radiation_sensor_value);

    /// Check if value between minimum and maximum
    if (radiation_sensor_value < radiation_msg_val_min_) {
      ROS_WARN_STREAM("Radiation sensor value is smaller than minimum: " <<
                      radiation_sensor_value << " <" << radiation_msg_val_min_ << ")");
    }
    if (radiation_sensor_value > radiation_msg_val_max_) {
      ROS_WARN_STREAM("Radiation sensor value is higher than maximum:" <<
                      radiation_sensor_value << ">" << radiation_msg_val_max_ << ")");
    }

    /// Try to look up transformation at the time from the message header,
    /// otherwise at the time of the last available transformation.
    Transformation T_G_C;
    if (transformer_.lookupTransform(radiation_sensor_frame_id_, world_frame_,
                                     msg->header.stamp,
                                      &T_G_C)) {
      /// Successful with time from header - do nothing
    } else if (transformer_.lookupTransform(radiation_sensor_frame_id_, world_frame_,
                                            ros::Time(0, 0), /// to get latest transformation
                                            &T_G_C)) {
      /// Successful with latest transformation - do nothing
    } else {
      /// Not successful
      ROS_WARN_STREAM(current_callback_num << ": Failed to look up intensity transform!");
      // ROS_WARN_THROTTLE(10, "Failed to look up intensity transform!");
      return;
    }

    /// Put this into the integrator.
    radiation_integrator_->addRadiationSensorValueBearingVectors(
        T_G_C.getPosition(), bearing_vectors_, radiation_sensor_value);

    ROS_INFO_STREAM(current_callback_num << ": Done.");
  }

  void RadioNuclearMapperServer::saveMeshTriggerCallback(const std_msgs::StringConstPtr& msg){
    CHECK(msg);

    /// Get value from subscriber message
    std::string message = msg->data.data();

    ROS_INFO_STREAM("Save Mesh Trigger Message: " << message);

    if (message.compare("original") == 0) {
      generateMesh();
    }else if (message.compare("constant") == 0) {
      generateMesh("constant", radiation_use_logarithm_, "traffic-light");
    }else if (message.compare("increasing") == 0) {
      generateMesh("increasing", radiation_use_logarithm_, "traffic-light");
    }else if (message.compare("decreasing") == 0) {
      generateMesh("decreasing", radiation_use_logarithm_, "traffic-light");
    }else if (message.compare("all") == 0) {
      const bool use_log_or_not [] = {false, true};
      const std::string distance_functions[] = {"constant", "increasing", "decreasing"};
      for (const bool log : use_log_or_not) {
        for (const std::string dist_func : distance_functions) {
          generateMesh(dist_func, log, "traffic-light");
        }
      }
    }
  }

  /**
   * Increasing Radiation Distant Function: f(d) = (d+1)^2
   * @param distance
   * @return
   */
  float RadioNuclearMapperServer::rad_dist_func_increasing(const float distance){
    return pow(distance + 1.0, 2);
  }

  /**
   * Decreasing Radiation Distant Function: f(d) = 1 / (d+1)^2
   * @param distance
   * @return
   */
  float RadioNuclearMapperServer::rad_dist_func_decreasing(const float distance){
    return 1.0 / pow(distance + 1.0, 2);
  }

  /**
   * Constant Radiation Distant Function: f(d) = 1
   * @param distance
   * @return
   */
  float RadioNuclearMapperServer::rad_dist_func_constant(const float distance){
    (void)distance;  /// To silence compiler
    return 1.0;
  }

  /**
   * Constant Radiation Distant Function: f(d) = \infty
   * @param distance
   * @return
   */
  float RadioNuclearMapperServer::rad_dist_func_infinity(const float distance){
    (void)distance; /// To silence compiler
    return std::numeric_limits<float>::infinity();
  }

}  /// namespace voxblox
