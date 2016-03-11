#include <lsd_slam_interface/LSDSlamInterface.h>
#include <sensor_msgs/PointCloud.h>
#include <nav_msgs/Path.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <sophus/sim3.hpp>

LSDSlamInterface::LSDSlamInterface() {}

LSDSlamInterface::~LSDSlamInterface() {}

bool LSDSlamInterface::Initialize(const ros::NodeHandle& n) {
  name_ = ros::names::append(n.getNamespace(), "LSDSlamInterface");

  if (!LoadParameters(n)) {
    ROS_ERROR("%s: Failed to load parameters.", name_.c_str());
    return false;
  }

  if (!RegisterCallbacks(n)) {
    ROS_ERROR("%s: Failed to register callbacks.", name_.c_str());
    return false;
  }

  return true;
}

bool LSDSlamInterface::LoadParameters(const ros::NodeHandle& n) {

  return true;
}

bool LSDSlamInterface::RegisterCallbacks(const ros::NodeHandle& n) {
  // Create a local nodehandle to manage callback subscriptions.
  ros::NodeHandle nl(n);

  keyframe_sub_ =
      nl.subscribe("keyframe", 100, &LSDSlamInterface::KeyframeCallback, this);

  graph_sub_ =
      nl.subscribe("graph", 100, &LSDSlamInterface::GraphCallback, this);

  point_cloud_pub_ =
      nl.advertise<sensor_msgs::PointCloud>("point_cloud", 10, false);

  pose_pub_ =
      nl.advertise<nav_msgs::Path>("poses", 10, false);

  point_cloud_service_ = nl.advertiseService(
      "publish_point_cloud", &LSDSlamInterface::CreatePointCloudService, this);

  return true;
}

void LSDSlamInterface::KeyframeCallback(
    const lsd_slam_viewer::keyframeMsg::ConstPtr& msg) {
  // Store the keyframe by ID for lookup at a later time.
  keyframe_hashmap_.insert(std::make_pair(msg->id, *msg));
}

void LSDSlamInterface::GraphCallback(
    const lsd_slam_viewer::keyframeGraphMsg::ConstPtr& msg) {

  // Store this as the most recent optimized pose graph so that we can use it to
  // create point clouds.
  current_pose_graph_ = *msg;

  // Make sure we have subscribers before doing work.
  if (pose_pub_.getNumSubscribers() == 0)
    return;

  // Iterate over keyframes in the optimized graph, publishing transforms and
  // timestamp data. Timestamps need to be looked up from the list of past
  // keyframes.
  std::vector<double> times;
  std::vector<std::vector<float> > transforms;

  nav_msgs::Path path;
  const GraphPose* optimized_poses = (GraphPose*)msg->frameData.data();
  for (size_t ii = 0; ii < msg->numFrames; ++ii) {
    const int pose_id = optimized_poses[ii].id;

    if (keyframe_hashmap_.count(pose_id) == 0) {
      ROS_ERROR("%s: Couldn't find keyframe referenced from graph.",
                name_.c_str());
      continue;
    }

    // Convert the SIM3 transform to an SE3 transform.
    Sophus::Sim3f sim3_transform;

    memcpy(sim3_transform.data(), optimized_poses[ii].cam_to_world,
           7 * sizeof(float));

    const Eigen::Vector3f translation = sim3_transform.translation();
    const Eigen::Quaternionf rotation(sim3_transform.rotationMatrix());

    geometry_msgs::PoseStamped pose;
    pose.header.stamp = ros::Time(keyframe_hashmap_[pose_id].time);
    pose.pose.position.x = translation.x();
    pose.pose.position.y = translation.y();
    pose.pose.position.z = translation.z();

    pose.pose.orientation.w = rotation.w();
    pose.pose.orientation.x = rotation.x();
    pose.pose.orientation.y = rotation.y();
    pose.pose.orientation.z = rotation.z();
    path.poses.push_back(pose);
  }

  // Publish the path message.
  path.header.stamp = ros::Time::now();
  path.header.frame_id = "world";
  pose_pub_.publish(path);
}

bool LSDSlamInterface::CreatePointCloudService(
    lsd_slam_interface::RequestPointCloud::Request& req,
    lsd_slam_interface::RequestPointCloud::Response& res) {

  // Iterate over poses in the optimized pose graph, grabbing a point cloud for
  // each frame and transforming it to world frame. This will create one big
  // world-frame point cloud.
  const GraphPose* optimized_poses =
      (GraphPose*)current_pose_graph_.frameData.data();

  sensor_msgs::PointCloud ros_point_cloud;
  ros_point_cloud.header.frame_id = "world";
  ros_point_cloud.header.stamp = ros::Time::now();
  ros_point_cloud.channels.resize(4);
  ros_point_cloud.channels[0].name = "r";
  ros_point_cloud.channels[1].name = "g";
  ros_point_cloud.channels[2].name = "b";
  ros_point_cloud.channels[3].name = "a";

  for (size_t ii = 0; ii < current_pose_graph_.numFrames; ++ii) {
    const int pose_id = optimized_poses[ii].id;
    if (keyframe_hashmap_.count(pose_id) == 0) {
      ROS_ERROR("%s: Couldn't find keyframe referenced from graph.",
                name_.c_str());
      continue;
    }

    // Look up the frame.
    const lsd_slam_viewer::keyframeMsg frame = keyframe_hashmap_[pose_id];

    // Convert the SIM3 transform to an SE3 transform.
    Sophus::Sim3f sim3_transform;
    memcpy(sim3_transform.data(), optimized_poses[ii].cam_to_world,
           7 * sizeof(float));
    const Eigen::Vector3f translation = sim3_transform.translation();
    const Eigen::Matrix3f rotation = sim3_transform.rotationMatrix();

    // Iterate over the inverse depth point cloud, transforming points in from
    // body frame to world frame using the keyframe's optimized pose.
    const float fxi = 1.f / frame.fx;
    const float fyi = 1.f / frame.fy;
    const float cxi = -frame.cx / frame.fx;
    const float cyi = -frame.cy / frame.fy;
    const unsigned int height = frame.height;
    const unsigned int width = frame.width;
    Point* point_cloud = new Point[width * height];
    memcpy(point_cloud, frame.pointcloud.data(),
           width * height * sizeof(Point));

    for (size_t row = 0; row < height; ++row) {
      for (size_t col = 0; col < width; ++col) {

        // Subsample points by 100 times.
        if (rand() % 100 != 0)
          continue;

        const float idepth = point_cloud[col + row * width].idepth;
        const float idepth_var = point_cloud[col + row * width].idepth_var;

        if (point_cloud[col + row * width].idepth <= 0)
          continue;

        const float depth = 1.f / idepth;
        const float depth4 = depth * depth * depth * depth;
        if (idepth_var * depth4 > 0.001f)
          continue;

        int near_support = 0;
        for (int dx = -1; dx < 2; dx++) {
          for (int dy = -1; dy < 2; dy++) {
            int idx = col + dx + (row + dy) * width;
            if (point_cloud[idx].idepth > 0) {
              float delta = point_cloud[idx].idepth - idepth;
              if (delta*delta < 2 * idepth_var)
                  near_support++;
            }
          }
        }
        if (near_support < 7)
          continue;

        Eigen::Vector3f point_body;
        point_body(0) = depth * (col * fxi + cxi);
        point_body(1) = depth * (row * fyi + cyi);
        point_body(2) = depth;

        const Eigen::Vector3f point_world = rotation * point_body + translation;
        if (std::isnan(point_world(0)) || std::isinf(point_world(0)) ||
            std::isnan(point_world(1)) || std::isinf(point_world(1)) ||
            std::isnan(point_world(2)) || std::isinf(point_world(2)))
          continue;

        geometry_msgs::Point32 ros_point_world;
        ros_point_world.x = point_world(0);
        ros_point_world.y = point_world(1);
        ros_point_world.z = point_world(2);
        ros_point_cloud.points.push_back(ros_point_world);

        const unsigned char r = point_cloud[col + row * width].color[0];
        const unsigned char g = point_cloud[col + row * width].color[1];
        const unsigned char b = point_cloud[col + row * width].color[2];
        const unsigned char a = point_cloud[col + row * width].color[3];

        ros_point_cloud.channels[0].values.push_back(r);
        ros_point_cloud.channels[1].values.push_back(g);
        ros_point_cloud.channels[2].values.push_back(b);
        ros_point_cloud.channels[3].values.push_back(a);
      }
    }

    delete[] point_cloud;
  }

  if (point_cloud_pub_.getNumSubscribers() > 0) {
    ROS_INFO("%s: Publishing %lu points\n", name_.c_str(),
             ros_point_cloud.points.size());
    point_cloud_pub_.publish(ros_point_cloud);
  }
}
