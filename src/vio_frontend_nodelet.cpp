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
    basalt::Calibration<double> calib;
    nh_ = getPrivateNodeHandle();
    std::string calibFile;
    if (!nh_.getParam("calibration_file", calibFile)) {
      ROS_ERROR_STREAM("must specify calibration_file!");
      throw std::invalid_argument("no calibration_file specified");
    }
    load_calib(&calib, calibFile);
    node_.reset(new VIOFrontEnd(nh_, calib));
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
