#include "voxblox_ros/radio_nuclear_mapper_server.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "voxblox");
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, false);
  google::InstallFailureSignalHandler();
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  voxblox::RadioNuclearMapperServer node(nh, nh_private);

  ros::spin();
  return 0;
}
