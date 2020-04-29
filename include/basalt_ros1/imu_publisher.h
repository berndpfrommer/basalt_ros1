/* -*-c++-*--------------------------------------------------------------------
 * 2020 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#pragma once

#include <basalt/imu/imu_types.h>

#include <tbb/concurrent_queue.h>

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

#include <memory>
#include <thread>

namespace basalt_ros1 {
  using sensor_msgs::Imu;
  using sensor_msgs::ImuConstPtr;

class IMUPublisher {
 public:
  typedef tbb::concurrent_bounded_queue<std::shared_ptr<basalt::ImuData>>
      ImuDataQueue;
  IMUPublisher(const ros::NodeHandle& node,
               const std::string& topic);

  ImuDataQueue* getInputQueue() { return (&inputQueue_); }
  void start();

 private:
  void processingThread();
  // ----- variables --
  ros::NodeHandle node_;
  std::string topic_;
  ros::Publisher pub_;
  ImuDataQueue inputQueue_;
  std::thread thread_;
  uint64_t framesPublished_{0};
  ros::Time lastTime_;
};
}  // namespace basalt_ros1
