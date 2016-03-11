#ifndef LSD_SLAM_INTERFACE_H
#define LSD_SLAM_INTERFACE_H

#include <ros/ros.h>
#include <lsd_slam_viewer/keyframeMsg.h>
#include <lsd_slam_viewer/keyframeGraphMsg.h>
#include <lsd_slam_interface/RequestPointCloud.h>

#include <unordered_map>

class LSDSlamInterface {
 public:
  LSDSlamInterface();
  ~LSDSlamInterface();

  // Calls LoadParameters and RegisterCallbacks. Fails on failure of either.
  bool Initialize(const ros::NodeHandle& n);

 private:
  // Node initialization.
  bool LoadParameters(const ros::NodeHandle& n);
  bool RegisterCallbacks(const ros::NodeHandle& n);

  // Callbacks.
  void KeyframeCallback(const lsd_slam_viewer::keyframeMsg::ConstPtr& msg);
  void GraphCallback(const lsd_slam_viewer::keyframeGraphMsg::ConstPtr& msg);

  // Service for creating point clouds.
  bool CreatePointCloudService(
      lsd_slam_interface::RequestPointCloud::Request& req,
      lsd_slam_interface::RequestPointCloud::Response& res);

  // Structure for pulling information from LSD SLAM's published pose graph.
  struct GraphPose {
    int id;
    float cam_to_world[7];
  };

  // Structure for pulling information from LSD SLAM's published point clouds.
  struct Point {
    float idepth;
    float idepth_var;
    unsigned char color[4];
  };

  // The node's name.
  std::string name_;

  // Publishers and subscribers.
  ros::Subscriber keyframe_sub_;
  ros::Subscriber graph_sub_;
  ros::Publisher point_cloud_pub_;
  ros::Publisher pose_pub_;
  ros::ServiceServer point_cloud_service_;

  // The most recent optimized pose graph message for creating point clouds.
  lsd_slam_viewer::keyframeGraphMsg current_pose_graph_;

  // To map from keyframe IDs to keyframe objects.
  std::unordered_map<int, lsd_slam_viewer::keyframeMsg> keyframe_hashmap_;
};

#endif
