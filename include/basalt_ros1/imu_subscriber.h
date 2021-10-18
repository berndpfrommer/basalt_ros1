/* -*-c++-*--------------------------------------------------------------------
 * 2020 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#pragma once

#include <basalt/imu/imu_types.h>

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

#include <tbb/concurrent_queue.h>

#include <memory>
#include <string>
#include <vector>

namespace basalt_ros1 {
using sensor_msgs::Imu;
using sensor_msgs::ImuConstPtr;

class IMUSubscriber {
 public:
  typedef tbb::concurrent_bounded_queue<std::shared_ptr<basalt::ImuData>>
      ImuDataQueue;
  typedef std::shared_ptr<ImuDataQueue> ImuDataQueuePtr;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  IMUSubscriber(const ros::NodeHandle& node,
                const std::vector<std::string>& topics);

  ~IMUSubscriber(){};

  ImuDataQueue* getQueue() { return queue_; }
  void setQueue(ImuDataQueue* q) { queue_ = q; }
  void printFrameRate();

 private:
  void callback_gyro(ImuConstPtr const& msg);
  void callback_accel(ImuConstPtr const& msg);
  void callback_combined(ImuConstPtr const& msg);
  // ----- variables --
  ros::NodeHandle node_;
  ros::Subscriber gyroSub_;
  ros::Subscriber accelSub_;
  ros::Subscriber combinedSub_;
  std::list<ImuConstPtr> gyroQueue_;
  ImuConstPtr prevAccel_;
  ImuDataQueue* queue_{nullptr};
  uint64_t combinedFramesReceived_{0};
  ros::Time lastTime_;
  ros::Time lastMsgTime_;
};
}  // namespace basalt_ros1
