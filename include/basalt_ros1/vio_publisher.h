/* -*-c++-*--------------------------------------------------------------------
 * 2020 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#pragma once
#include <basalt/utils/imu_types.h>

#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>

//-----------
#include <tf2_ros/transform_listener.h>
#include <tf2/buffer_core.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/utils.h>
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include <string>

//-----------
#include <memory>

namespace basalt_ros1 {
class VIOPublisher {
 public:
  VIOPublisher(const ros::NodeHandle& node);

  void publish(const basalt::PoseVelBiasState::Ptr& data);

 private:
  // ----- variables --
  ros::NodeHandle node_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tfBroadCaster_;
  ros::Publisher pub_;
  ros::Publisher pub_flu_;
  boost::array<double, 36> cov_;
  nav_msgs::Odometry msg_;

  //---fix basalt map frame---
  tf2::Stamped<tf2::Transform> T_s_b; //sensor_link 2 base_link
  bool odom_flu_ = false;
  bool tf_flu_ = false;
  std::string odom_link_;
  std::string base_link_;
  std::string world_flu_link_;
  nav_msgs::Odometry msg_flu_;
};
}  // namespace basalt_ros1
