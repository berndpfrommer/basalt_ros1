/* -*-c++-*--------------------------------------------------------------------
 * 2020 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#pragma once

#include <basalt_ros1/image_subscriber.h>
#include <basalt_ros1/optical_flow_publisher.h>

#include <basalt/optical_flow/optical_flow.h>
#include <basalt/calibration/calibration.hpp>

#include <tbb/concurrent_queue.h>

#include <ros/ros.h>

namespace basalt_ros1 {
class VIOFrontEnd {
 public:
  VIOFrontEnd(const ros::NodeHandle& nh,
              const basalt::Calibration<double>& calib,
              const basalt::VioConfig& config);

  VIOFrontEnd(const VIOFrontEnd&) = delete;
  VIOFrontEnd& operator=(const VIOFrontEnd&) = delete;
  //
  tbb::concurrent_bounded_queue<basalt::OpticalFlowResult::Ptr>**
  getOpticalFlowQueue() {
    return (&opticalFlowOut_);
  }
  void setOpticalFlowQueue(
      tbb::concurrent_bounded_queue<basalt::OpticalFlowResult::Ptr>* q) {
    opticalFlowOut_ = q;
  }

  // ------ own methods
  bool initialize();

 private:
  // ------ variables --------
  ros::NodeHandle node_;
  basalt::OpticalFlowBase::Ptr opticalFlow_;
  basalt::Calibration<double> calibration_;
  basalt::VioConfig config_;
  std::shared_ptr<ImageSubscriber> imageSub_;
  std::shared_ptr<OpticalFlowPublisher> opticalFlowPub_;
  tbb::concurrent_bounded_queue<basalt::OpticalFlowResult::Ptr>*
      opticalFlowOut_ = nullptr;
};
}  // namespace basalt_ros1
