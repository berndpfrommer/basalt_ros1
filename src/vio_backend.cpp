/* -*-c++-*--------------------------------------------------------------------
 * 2020 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#include <basalt_ros1/vio_backend.h>

template <typename T>
static T get_param(const ros::NodeHandle& node, const std::string& key,
                   const T& def) {
  T p;
  node.param<T>(key, p, def);
  return (p);
}

namespace basalt_ros1 {
VIOBackEnd::VIOBackEnd(const ros::NodeHandle& node,
                       const basalt::Calibration<double>& calib,
                       const basalt::VioConfig& config,
                       OpticalFlowResultQueue** opt_flow_queue)
    : node_(node) {
  // create VIO object
  vio_ = basalt::VioEstimatorFactory::getVioEstimator(
      config, calib, basalt::constants::g, true);
  vio_->initialize(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());

  // if no external queue is provided, subscribe to
  // optical flow topic and pipe output into vio
  if (opt_flow_queue) {
    *opt_flow_queue = &vio_->vision_data_queue;
  } else {
    flowSub_ = std::make_shared<OpticalFlowSubscriber>(node_, "optical_flow");
    flowSub_->setQueue(&vio_->vision_data_queue);
  }

  // subscribe to IMU and pipe into the vio
  std::vector<std::string> imu_topics = {"gyro", "accel"};
  node.param<std::vector<std::string>>("imu_topics", imu_topics, imu_topics);

  imuSub_ = std::make_shared<IMUSubscriber>(node_, imu_topics);
  imuSub_->setQueue(&(vio_->imu_data_queue));

  // create publisher for odom
  publisher_ = std::make_shared<basalt_ros1::VIOPublisher>(node_);

  // connect the VIO output to our own queue for publishing
  vio_->out_state_queue = &outputQueue_;
}

void VIOBackEnd::start() {
  thread_ = std::thread(&VIOBackEnd::publishingThread, this);
  ROS_INFO("backend started!");
}

void VIOBackEnd::publishingThread() {
  basalt::PoseVelBiasState::Ptr data;
  for (;;) {
    outputQueue_.pop(data);
    if (!data.get()) {
      ROS_INFO("backend exiting!");
      break;
    }
    publisher_->publish(data);
  }
}

}  // namespace basalt_ros1
