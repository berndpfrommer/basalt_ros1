/* -*-c++-*--------------------------------------------------------------------
 * 2020 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#include <basalt_ros1/utils.h>
#include <basalt_ros1/vio_backend.h>
#include <basalt_ros1/vio_frontend.h>

#include <nodelet/nodelet.h>
#include <ros/ros.h>

#include <memory>
#include <string>

namespace basalt_ros1 {
class VIONodelet : public nodelet::Nodelet {
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
    frontEnd_.reset(new basalt_ros1::VIOFrontEnd(nh_, calib));
    // front and backend communicate via a direct queue
    basalt_ros1::VIOBackEnd::OpticalFlowResultQueue** q =
        frontEnd_->getOpticalFlowQueue();
    backEnd_.reset(new basalt_ros1::VIOBackEnd(nh_, calib, q));
    // now start them both
    backEnd_->start();
    frontEnd_->initialize();
  }

 private:
  // ------ variables --------
  std::shared_ptr<basalt_ros1::VIOFrontEnd> frontEnd_;
  std::shared_ptr<basalt_ros1::VIOBackEnd> backEnd_;
  ros::NodeHandle nh_;
};
}  // namespace basalt_ros1

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(basalt_ros1::VIONodelet, nodelet::Nodelet)
