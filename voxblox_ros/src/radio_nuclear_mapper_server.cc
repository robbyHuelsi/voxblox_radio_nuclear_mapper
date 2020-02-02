#include "voxblox_ros/radio_nuclear_mapper_server.h"

#include "voxblox/core/common.h" // TODO

namespace voxblox {
  RadioNuclearMapperServer::RadioNuclearMapperServer(const ros::NodeHandle& nh,
                                                     const ros::NodeHandle& nh_private)
      : TsdfServer(nh, nh_private) {

    cache_mesh_ = true;

    radiation_layer_.reset(new Layer<IntensityVoxel>(tsdf_map_->getTsdfLayer().voxel_size(),
                                                     tsdf_map_->getTsdfLayer().voxels_per_side()));
    radiation_integrator_.reset(new RadioNuclearMapperIntegrator(tsdf_map_->getTsdfLayer(),
                                                                 radiation_layer_.get()));

    radiation_mesh_ = Mesh(mesh_layer_->block_size(), Point::Zero());

    /// Get ROS params for radiation sensor
    getServerConfigFromRosParam(nh_private_);

    /// Forward some paramters to integrator
    radiation_integrator_->setMaxDistance(radiation_max_distance_);

    /// Set min and max value for color map
    setCMExtrValByMostExtrPossible(radiation_msg_val_min_, radiation_msg_val_max_, radiation_distance_function_,
                                   radiation_msg_use_log_, radiation_max_distance_, color_map_);

    /// Publishers for output.
    radiation_pointcloud_pub_ =
        nh_private_.advertise<pcl::PointCloud<pcl::PointXYZI> >(
            "radiation_pointcloud", 1, true);
    radiation_mesh_pub_ =
        nh_private_.advertise<voxblox_msgs::Mesh>("radiation_mesh", 1, true);

    /// Subscribe radiation intensity
    radSenCallbackCounter = 0;
    radiation_sensor_sub_ = nh_private_.subscribe(
        radiation_sensor_topic_, 1, &RadioNuclearMapperServer::radiationSensorCallback, this);

    /// Subscribe save mesh trigger
    save_mesh_trigger_sub_ = nh_private_.subscribe(
        save_mesh_trigger_topic_, 1, &RadioNuclearMapperServer::saveMeshTriggerCallback, this);
  }

  void RadioNuclearMapperServer::getServerConfigFromRosParam(
      const ros::NodeHandle& nh_private) {
    //TODO: Vmtl hat das Überschreiben der Funktion von TsdfServer die Auswirkung, dass dort die Parameter nicht gesetzt werden und deshalb das Mesh nicht wie gewünscht aussieht

    /// Define pre-sets of parameters
    radiation_sensor_topic_ = "";
    radiation_sensor_frame_id_ = "";
    radiation_max_distance_ = radiation_integrator_->getMaxDistance();
    radiation_distance_function_name_ = "constant";
    radiation_msg_val_min_ = 0.0;
    radiation_msg_val_max_ = 100000.0;
    radiation_msg_use_log_ = false;
    radiation_ang_res_y_ = 100;
    radiation_ang_res_z_ = 100;
    std::string color_map_scheme_name = "ironbow";
    save_mesh_trigger_topic_ = "";

    /// Get ROS parameters
    nh_private.param<std::string>("radiation_sensor_topic",
                                  radiation_sensor_topic_, radiation_sensor_topic_);
    nh_private.param<std::string>("radiation_sensor_frame_id",
                                  radiation_sensor_frame_id_, radiation_sensor_frame_id_);
    nh_private.param("radiation_max_distance", radiation_max_distance_, radiation_max_distance_);
    nh_private.param<std::string>("radiation_distance_function",
                                  radiation_distance_function_name_, radiation_distance_function_name_);
    nh_private.param("radiation_msg_val_min", radiation_msg_val_min_, radiation_msg_val_min_);
    nh_private.param("radiation_msg_val_max", radiation_msg_val_max_, radiation_msg_val_max_);
    nh_private.param("radiation_msg_use_log", radiation_msg_use_log_, radiation_msg_use_log_);
    nh_private.param("radiation_ang_res_y_", radiation_ang_res_y_, radiation_ang_res_y_);
    nh_private.param("radiation_ang_res_z_", radiation_ang_res_z_, radiation_ang_res_z_);
    nh_private.param("radiation_colormap", color_map_scheme_name, color_map_scheme_name);
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
    setColorMapScheme(color_map_scheme_name, color_map_);

    /// Print message value parameters and make it logarithmic if needed
    ROS_INFO_STREAM("Radiation sensor value range (w/o logarithm): [" <<
                    radiation_msg_val_min_ << ", " << radiation_msg_val_max_ << "]");
    if(radiation_msg_use_log_){
      ROS_INFO_STREAM("Radiation sensor value range (with logarithm): " <<
                      "[" << (log(radiation_msg_val_min_) < 0.0 ? 0.0 : log(radiation_msg_val_min_)) <<
                      ", " << log(radiation_msg_val_max_) << "]");
    }

    /// Check parameter validity for distance parameters and print it //todo
    getRadiationDistanceFunctionByName(radiation_distance_function_name_, radiation_distance_function_);

    /// Print resolution parameters
    ROS_INFO_STREAM("Radiation angular resolution: " <<
                    "(y=" << radiation_ang_res_y_ << "; z=" << radiation_ang_res_z_ << ")");

    /// Check parameter validity for save mesh trigger parameter and print it
    if(!save_mesh_trigger_topic_.empty()){
      ROS_INFO_STREAM("Save generated mesh with message at this topic: " << save_mesh_trigger_topic_);
    }
  }

  bool RadioNuclearMapperServer::setColorMapScheme(const std::string color_map_scheme_name,
                                                   std::shared_ptr<ColorMap>& color_map){
    /// Try to select color map scheme by incomming string
    bool color_map_scheme_valid = false;
    if (color_map_scheme_name == "rainbow") {
      color_map.reset(new RainbowColorMap());
      color_map_scheme_valid = true;
    } else if (color_map_scheme_name == "inverse_rainbow") {
      color_map.reset(new InverseRainbowColorMap());
      color_map_scheme_valid = true;
    } else if (color_map_scheme_name == "grayscale") {
      color_map.reset(new GrayscaleColorMap());
      color_map_scheme_valid = true;
    } else if (color_map_scheme_name == "inverse_grayscale") {
      color_map.reset(new InverseGrayscaleColorMap());
      color_map_scheme_valid = true;
    } else if (color_map_scheme_name == "ironbow") {
      color_map.reset(new IronbowColorMap());
      color_map_scheme_valid = true;
    } else if (color_map_scheme_name == "traffic-light") {
      color_map.reset(new TrafficLightColorMap());
      color_map_scheme_valid = true;
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

  bool RadioNuclearMapperServer::getRadiationDistanceFunctionByName(const std::string distance_function_name,
                                                                    RDFType& rad_dist_func){
    //TODO: Comments (after replacing char with function pointer)
    /// Check if the incoming string is allowed
    std::vector<std::string> allowed_funcs = {"increasing", "decreasing", "constant"};
    if(std::find(allowed_funcs.begin(), allowed_funcs.end(), distance_function_name) == allowed_funcs.end()){
      rad_dist_func = &RadioNuclearMapperServer::rad_dist_func_zero;
      // Distance function string is not allowed
      ROS_ERROR("Radiation distance function is called by a not supported string.");
      std::stringstream allowed_funcs_ss = std::stringstream();
      for(std::vector<std::string>::iterator it = allowed_funcs.begin(); it != allowed_funcs.end(); ++it) {
        allowed_funcs_ss << *it << ", ";
      }
      ROS_INFO_STREAM("Use one of the following commands for 'radiation_distance_function': " <<
                      allowed_funcs_ss.str().substr(0, allowed_funcs_ss.str().length()-2));
      return false;
    }else{
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

  inline void RadioNuclearMapperServer::calcIntensity(const float sensor_value, const float distance,
                                               RDFType& rad_dist_func, const bool use_logarithm, float& intensity) {
    intensity = sensor_value;

    /// Apply the desired radiation distance function
    /// Looks crazy, but it works (https://stackoverflow.com/a/1486279)
    (*this.*rad_dist_func)(intensity, distance);

    /// Use logarithmic mapping if needed
    if(use_logarithm){
      intensity = log(intensity);
      intensity = intensity < 0.0 ? 0.0 : intensity;  // TODO: Comment
    }
  }

  // TODO: Continue tidy up here
  inline Color RadioNuclearMapperServer::getColorForVoxelPointer(const IntensityVoxel* voxel,
                                                          const std::shared_ptr<ColorMap>& color_map,
                                                          RDFType& rad_dist_func, const bool use_logarithm){
    Color c;
    if (voxel != nullptr && voxel->weight < std::numeric_limits<float>::infinity()) { // && 1.0 / voxel->weight > 0.0
      float intensity;
      calcIntensity(voxel->intensity, voxel->weight, rad_dist_func, use_logarithm, intensity);
      //printf("Intensity: %f", intensity);
//          std::cout << "Intensity: " << intensity << std::endl;
      c = color_map->colorLookup(intensity);

    } else {
//      c = color_map->colorLookup(0.0);
      c = Color(0.0, 0.0, 0.0, 0.0);
    }
    return c;
  }

  inline void RadioNuclearMapperServer::recolorVoxbloxMeshMsgByRadiationIntensity(
      const Layer<IntensityVoxel>& intensity_layer,
      const std::shared_ptr<ColorMap>& color_map,
      RDFType& rad_dist_func, const bool use_logarithm, //TODO: Use Pointer
      voxblox_msgs::Mesh* mesh_msg) {
    CHECK_NOTNULL(mesh_msg);
    CHECK(color_map);

/*  size_t vps = intensity_layer.voxels_per_side();
  size_t nps = vps * vps * vps;

  BlockIndexList blocks;
  intensity_layer.getAllAllocatedBlocks(&blocks);
 for (const BlockIndex& index : blocks){
    const Block<IntensityVoxel>& block = intensity_layer.getBlockByIndex(index);
    for (size_t linear_index = 0; linear_index < nps; ++linear_index) {
      Point coord = block.computeCoordinatesFromLinearIndex(linear_index);
      // std::cout << "Block: x: " << coord.x() << "; y: " << coord.y() << "; z: " << coord.z() << std::endl;
      const IntensityVoxel* voxel = intensity_layer.getVoxelPtrByCoordinates(coord);
      if (voxel != nullptr) {
        //std::cout << "Voxel intensity: " << voxel->intensity << std::endl;
        std::cout << "Block: x: " << coord.x() << "; y: " << coord.y() << "; z: " << coord.z() << std::endl;
      }
    }
  }*/

    // Go over all the blocks in the mesh message.
    for (voxblox_msgs::MeshBlock& mesh_block : mesh_msg->mesh_blocks) {
      // Look up verticies in the thermal layer.
      for (size_t vert_idx = 0u; vert_idx < mesh_block.x.size(); ++vert_idx) {

        // only needed if color information was originally missing
/*      mesh_block.r.resize(mesh_block.x.size());
      mesh_block.g.resize(mesh_block.x.size());
      mesh_block.b.resize(mesh_block.x.size());

      const IntensityVoxel* voxel = intensity_layer.getVoxelPtrByCoordinates(
          Point(mesh_block.x[vert_idx], mesh_block.y[vert_idx],
                mesh_block.z[vert_idx]));*/

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
        Point p = Point(mesh_x, mesh_y, mesh_z);

        const IntensityVoxel* voxel = intensity_layer.getVoxelPtrByCoordinates(p);

        //std::cout << "Voxel: x: " << mesh_x << std::endl;
//        std::cout << "Voxel: X: " << mesh_x << "; y: " << mesh_y << "; z: " << mesh_z << std::endl;

//        if (voxel == nullptr) {
//          //printf("voxel is null pointer!");
//        } else if (voxel->weight <= 0.0) {
////          std::cout << "weight too small" << std::endl;
//        }

        Color c = getColorForVoxelPointer(voxel, color_map, rad_dist_func, use_logarithm);

        //Update mesh message color
        mesh_block.r[vert_idx] = c.r;
        mesh_block.g[vert_idx] = c.g;
        mesh_block.b[vert_idx] = c.b;
      }
    }
  }

  void RadioNuclearMapperServer::updateMesh() {
    TsdfServer::updateMesh(); // TODO: Rewrite here and remove?

    // Now recolor the mesh...
    timing::Timer publish_mesh_timer("radiation_mesh/publish");
    recolorVoxbloxMeshMsgByRadiationIntensity(*radiation_layer_, color_map_, radiation_distance_function_, radiation_msg_use_log_,
                                              &cached_mesh_msg_);
    // generateMesh(tmp_mesh);

    radiation_mesh_pub_.publish(cached_mesh_msg_);
    publish_mesh_timer.Stop();
  }

  bool RadioNuclearMapperServer::generateMesh() {
    return generateMesh(radiation_distance_function_name_, radiation_msg_use_log_);
  }

  bool RadioNuclearMapperServer::generateMesh(const std::string& distance_function,
                                              const bool use_logarithm){
    std::string log_str = use_logarithm?"log":"no-log";

    timing::Timer generate_mesh_timer("mesh/generate");
    RDFType rad_dist_func;
    getRadiationDistanceFunctionByName(distance_function, rad_dist_func);
    std::shared_ptr<ColorMap> export_color_map;
    setColorMapScheme("traffic-light", export_color_map);
    setCMExtrValByMostExtrPossible(radiation_msg_val_min_, radiation_msg_val_max_, rad_dist_func, use_logarithm,
                                   radiation_max_distance_, export_color_map);

    Mesh mesh = Mesh(mesh_layer_->block_size(), Point::Zero());
    convertMeshLayerToMesh(*mesh_layer_, &mesh, true);
    float num_mesh_points = float(mesh.size());
    // Go over all the blocks in the mesh message.
    for (size_t i = 0; i < mesh.size(); i++) {
      if (i % 10 == 0){
        float percentage = std::round(float(i) / num_mesh_points * 1000.0) / 10.0;
        std::cout << "Recoloring (" << distance_function << " function, " << log_str << "): " << percentage << " %     \r" <<std::flush;
      }
      Point p = mesh.vertices[i];
      const IntensityVoxel* voxel = radiation_layer_->getVoxelPtrByCoordinates(p);
      Color c = getColorForVoxelPointer(voxel, export_color_map, rad_dist_func, use_logarithm);
      mesh.colors[i] = c;
    }
    std::cout << "Recoloring done.                                                    " << std::endl;
    generate_mesh_timer.Stop();

    timing::Timer output_mesh_timer("mesh/output");
    std::cout << "Exporting (" << distance_function << " function, " << log_str << ") ..." <<std::endl;
    time_t raw_time;
    struct tm * time_info;
    char time_buffer[80];
    time (&raw_time);
    time_info = localtime(&raw_time);
    strftime(time_buffer,sizeof(time_buffer),"%Y-%m-%d-%H-%M-%S",time_info);
    std::string time_str = time_buffer;
//    struct timeval seconds;
//    gettimeofday(&seconds, NULL);
//    std::string miliseconds_str = std::to_string((long int)seconds.tv_usec);

    std::string filename = "" + log_str + "_" + distance_function + "_" + time_str + ".ply";
    const bool success = outputMeshAsPly(filename, mesh);
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
    // Create a pointcloud with radiation = intensity.
    pcl::PointCloud<pcl::PointXYZI> pointcloud;

    createIntensityPointcloudFromIntensityLayer(*radiation_layer_, &pointcloud);

    pointcloud.header.frame_id = world_frame_;
    radiation_pointcloud_pub_.publish(pointcloud);

    TsdfServer::publishPointclouds();
  }

  void RadioNuclearMapperServer::radiationSensorCallback(const abc_msgs_fkie::MeasurementRawConstPtr& msg) {
    CHECK(radiation_layer_);
    CHECK(radiation_integrator_);
    CHECK(msg);

    size_t currCout = radSenCallbackCounter++;

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

    ROS_INFO_STREAM(currCout << ": New radiation value: " << radiation_sensor_value);

    // Look up transform
    Transformation T_G_C;
//    ros::Time t = ; //msg->header.stamp
    if (!transformer_.lookupTransform(radiation_sensor_frame_id_, world_frame_,
                                     ros::Time(0, 0), // To get latest positions
                                      &T_G_C)) {
      ROS_WARN_STREAM(currCout << ": Failed to look up intensity transform!");
//      ROS_WARN_THROTTLE(10, "Failed to look up intensity transform!");
      return;
    }

//    ROS_INFO_STREAM(currCout << ": Before bearing_vectors...");

    // Pre-allocate the bearing vectors and intensities.
    Pointcloud bearing_vectors;
    bearing_vectors.reserve(radiation_ang_res_z_ * radiation_ang_res_y_ + 1);

//    ROS_INFO_STREAM(currCout << ": Before loop...");

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

//    ROS_INFO_STREAM(currCout << ": Before integrator...");

    // Put this into the integrator.
    radiation_integrator_->addRadiationSensorValueBearingVectors(
        T_G_C.getPosition(), bearing_vectors, radiation_sensor_value);

    ROS_INFO_STREAM(currCout << ": Done.");
  }

  void RadioNuclearMapperServer::saveMeshTriggerCallback(const std_msgs::StringConstPtr& msg){
    CHECK(msg);

    // Get value from subscriber message
    std::string message = msg->data.data();

    ROS_INFO_STREAM("Save Message Trigger Message: " << message);

    if (message.compare("original") == 0) {
      generateMesh();
    }else if (message.compare("constant") == 0) {
      generateMesh("constant", radiation_msg_use_log_);
    }else if (message.compare("increasing") == 0) {
      generateMesh("increasing", radiation_msg_use_log_);
    }else if (message.compare("decreasing") == 0) {
      generateMesh("decreasing", radiation_msg_use_log_);
    }else if (message.compare("all") == 0) {
      const bool use_log_or_not [] = {false, true};
      const std::string distance_functions[] = {"constant", "increasing", "decreasing"};
      for (const bool log : use_log_or_not) {
        for (const std::string dist_func : distance_functions) {
          generateMesh(dist_func, log);
        }
      }
    }
  }

  void RadioNuclearMapperServer::rad_dist_func_increasing(float& intensity, const float& distance){
    intensity = intensity * pow(distance + 1.0, 2);
//    printf("increasing\n");
  }

  void RadioNuclearMapperServer::rad_dist_func_decreasing(float& intensity, const float& distance){
    intensity = intensity / pow(distance + 1.0, 2);
//    printf("decreasing\n");
  }

  void RadioNuclearMapperServer::rad_dist_func_constant(float& intensity, const float& distance){
    (void)intensity; // To silence compiler (intensity = intensity)
    (void)distance;  // To silence compiler
//    printf("constant\n");
  }

  void RadioNuclearMapperServer::rad_dist_func_zero(float& intensity, const float& distance){
    intensity = 0.0;
    (void)distance; //To silence compiler
//    printf("zero\n");
  }

}  // namespace voxblox
