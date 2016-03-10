#include <ros/ros.h>
#include <lsd_slam_interface/LSDSlamInterface.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "lsd_slam_interface");
  ros::NodeHandle n("~");

  LSDSlamInterface lsdsi;
  if (!lsdsi.Initialize(n)) {
    ROS_ERROR("%s: Failed to initialize LSD SLAM interface.",
              ros::this_node::getName().c_str());
    return EXIT_FAILURE;
  }
  ros::spin();

  return EXIT_SUCCESS;
}
