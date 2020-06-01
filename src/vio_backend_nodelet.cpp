/* -*-c++-*--------------------------------------------------------------------
 * 2020 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#include <basalt_ros1/utils.h>
#include <basalt_ros1/vio_backend.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>

#include <string>

namespace basalt_ros1 {
class VIOBackEndNodelet : public nodelet::Nodelet {
 public:
  void onInit() override {
    nh_ = getPrivateNodeHandle();
    basalt::Calibration<double> calib;
    basalt::VioConfig vioConfig;
    basalt_ros1::load_calib_and_config(nh_, &calib, &vioConfig);

    basalt_ros1::VIOBackEnd::OpticalFlowResultQueue **q = NULL;

    node_.reset(new VIOBackEnd(nh_, calib, vioConfig, q));
    node_->start();
  }

 private:
  // ------ variables --------
  std::shared_ptr<basalt_ros1::VIOBackEnd> node_;
  ros::NodeHandle nh_;
};

}  // namespace basalt_ros1

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(basalt_ros1::VIOBackEndNodelet, nodelet::Nodelet)
