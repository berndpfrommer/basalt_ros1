/* -*-c++-*--------------------------------------------------------------------
 * 2020 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#pragma once
#include <basalt/utils/imu_types.h>

#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>

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
  boost::array<double, 36> cov_;
};
}  // namespace basalt_ros1
