/* -*-c++-*--------------------------------------------------------------------
 * 2020 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#pragma once
#include <basalt/utils/vio_config.h>
#include <basalt/calibration/calibration.hpp>

#include <ros/ros.h>

#include <string>

namespace basalt_ros1 {
void load_calib(basalt::Calibration<double> *calib,
                const std::string &calib_path);

void load_calib_and_config(ros::NodeHandle &nh,
                           basalt::Calibration<double> *calib,
                           basalt::VioConfig *config);

}  // namespace basalt_ros1
