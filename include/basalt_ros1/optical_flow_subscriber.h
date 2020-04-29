/* -*-c++-*--------------------------------------------------------------------
 * 2020 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#pragma once

#include <basalt_vio_ros1_msgs/OpticalFlowResult.h>

#include <basalt/optical_flow/optical_flow.h>

#include <ros/ros.h>

#include <tbb/concurrent_queue.h>

#include <memory>

namespace basalt_ros1 {
class OpticalFlowSubscriber {
 public:
  typedef basalt_vio_ros1_msgs::OpticalFlowResult OpticalFlowMsg;
  typedef boost::shared_ptr<const OpticalFlowMsg> OpticalFlowMsgConstPtr;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  OpticalFlowSubscriber(const ros::NodeHandle& node, const std::string& topic);

  void setQueue(
      tbb::concurrent_bounded_queue<basalt::OpticalFlowResult::Ptr>* q) {
    queue_ = q;
  }
  ~OpticalFlowSubscriber(){};

 private:
  void callback(OpticalFlowMsgConstPtr const& msg);
  // ----- variables --
  ros::NodeHandle node_;
  ros::Subscriber sub_;
  tbb::concurrent_bounded_queue<basalt::OpticalFlowResult::Ptr>* queue_{
      nullptr};

  long int max_q_{0};
};
}  // namespace basalt_ros1
