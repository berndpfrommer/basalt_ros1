/* -*-c++-*--------------------------------------------------------------------
 * 2020 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#include <basalt_ros1/utils.h>
#include <basalt_ros1/vio_backend.h>
#include <basalt_ros1/vio_frontend.h>

#include <ros/ros.h>

#include <string>

int main(int argc, char** argv) {
  ros::init(argc, argv, "vio_node");
  ros::NodeHandle pnh("~");

  try {
    basalt::Calibration<double> calib;
    std::string calibFile;
    if (!pnh.getParam("calibration_file", calibFile)) {
      ROS_ERROR_STREAM("must specify calibration_file!");
      throw std::invalid_argument("no calibration_file specified");
    }
    basalt_ros1::load_calib(&calib, calibFile);
    basalt_ros1::VIOFrontEnd frontEnd(pnh, calib);
    // front and backend communicate via a direct queue
    basalt_ros1::VIOBackEnd::OpticalFlowResultQueue** q =
        frontEnd.getOpticalFlowQueue();
    basalt_ros1::VIOBackEnd backEnd(pnh, calib, q);
    backEnd.start();
    frontEnd.initialize();
    ros::spin();
  } catch (const std::exception& e) {
    ROS_ERROR("%s: %s", pnh.getNamespace().c_str(), e.what());
    return (-1);
  }
  return (0);
}
