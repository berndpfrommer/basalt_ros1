/* -*-c++-*--------------------------------------------------------------------
 * 2020 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#include <basalt_ros1/utils.h>
#include <basalt_ros1/vio_frontend.h>

#include <nodelet/nodelet.h>
#include <ros/ros.h>

#include <string>

namespace basalt_ros1 {
class VIOFrontEndNodelet : public nodelet::Nodelet {
 public:
  void onInit() override {
    nh_ = getPrivateNodeHandle();
    basalt::Calibration<double> calib;
    basalt::VioConfig vioConfig;
    basalt_ros1::load_calib_and_config(nh_, &calib, &vioConfig);

    node_.reset(new VIOFrontEnd(nh_, calib, vioConfig));
    node_->initialize();
  }

 private:
  // ------ variables --------
  std::shared_ptr<VIOFrontEnd> node_;
  ros::NodeHandle nh_;
};
}  // namespace basalt_ros1

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(basalt_ros1::VIOFrontEndNodelet, nodelet::Nodelet)
