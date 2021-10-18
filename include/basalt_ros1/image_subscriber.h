/* -*-c++-*--------------------------------------------------------------------
 * 2020 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#pragma once
#include <basalt/optical_flow/optical_flow.h>

#include <tbb/concurrent_queue.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>

#include <memory>
#include <string>

namespace basalt_ros1 {
using sensor_msgs::Image;
using sensor_msgs::ImageConstPtr;

class ImageSubscriber {
  typedef tbb::concurrent_bounded_queue<basalt::OpticalFlowInput::Ptr>
      OpticalFlowInputQueue;
  typedef std::shared_ptr<OpticalFlowInputQueue> OpticalFlowInputQueuePtr;

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  ImageSubscriber(const ros::NodeHandle& node, const std::string& topic_left,
                  const std::string& topic_right);

  ~ImageSubscriber(){};

  OpticalFlowInputQueue* getQueue() { return queue_; }
  void setQueue(OpticalFlowInputQueue* q) { queue_ = q; }

 private:
  void callback(ImageConstPtr const& left, ImageConstPtr const& right);
  // ----- variables --
  ros::NodeHandle node_;
  std::shared_ptr<message_filters::TimeSynchronizer<Image, Image>> sync_;
  std::vector<std::shared_ptr<message_filters::Subscriber<Image>>> subs_;
  OpticalFlowInputQueue* queue_{nullptr};
  uint64_t framesReceived_{0};
  ros::Time lastTime_;
};
}  // namespace basalt_ros1
