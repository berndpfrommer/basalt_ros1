/* -*-c++-*--------------------------------------------------------------------
 * 2020 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#include <basalt_ros1/utils.h>
#include <basalt_ros1/vio_frontend.h>

#include <ros/ros.h>

#include <string>

int main(int argc, char** argv) {
  ros::init(argc, argv, "vio_frontend_node");
  ros::NodeHandle pnh("~");

  try {
    basalt::Calibration<double> calib;
    basalt::VioConfig vioConfig;
    basalt_ros1::load_calib_and_config(pnh, &calib, &vioConfig);

    basalt_ros1::VIOFrontEnd node(pnh, calib, vioConfig);
    node.initialize();
    ros::spin();
  } catch (const std::exception& e) {
    ROS_ERROR("%s: %s", pnh.getNamespace().c_str(), e.what());
    return (-1);
  }
  return (0);
}
