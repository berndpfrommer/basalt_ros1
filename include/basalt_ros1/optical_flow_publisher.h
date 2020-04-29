/* -*-c++-*--------------------------------------------------------------------
 * 2020 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#pragma once

#include <basalt/optical_flow/optical_flow.h>
#include <basalt_vio_ros1_msgs/OpticalFlowResult.h>

#include <tbb/concurrent_queue.h>

#include <ros/ros.h>

#include <memory>
#include <thread>

namespace basalt_ros1 {
class OpticalFlowPublisher {
 public:
  OpticalFlowPublisher(const ros::NodeHandle& node, const std::string& topic);

  tbb::concurrent_bounded_queue<basalt::OpticalFlowResult::Ptr>*
  getInputQueue() {
    return (&inputQueue_);
  }
  void start();

 private:
  void processingThread();
  // ----- variables --
  ros::NodeHandle node_;
  std::string topic_;
  ros::Publisher pub_;
  tbb::concurrent_bounded_queue<basalt::OpticalFlowResult::Ptr> inputQueue_;
  std::thread thread_;
};
}  // namespace basalt_ros1
