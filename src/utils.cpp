/* -*-c++-*--------------------------------------------------------------------
 * 2020 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#include <basalt_ros1/utils.h>

// needed for loading calibration via cereal
#include <basalt/serialization/headers_serialization.h>

#include <ros/ros.h>

#include <fstream>

namespace basalt_ros1 {
void load_calib(basalt::Calibration<double> *calib,
                const std::string &calib_path) {
  if (calib_path.empty()) {
    ROS_ERROR("no calib file specified!");
    throw std::invalid_argument("no calibration file specified");
  }
  std::ifstream os(calib_path, std::ios::binary);
  if (os.is_open()) {
    cereal::JSONInputArchive archive(os);
    archive(*calib);
    ROS_INFO_STREAM("loaded calibration file with " << calib->intrinsics.size()
                                                    << " cameras");
  } else {
    ROS_ERROR_STREAM("could not load calibration file " << calib_path);
    throw std::ios_base::failure("could not find calibration file: " +
                                 calib_path);
  }
}

void load_calib_and_config(ros::NodeHandle &nh,
                           basalt::Calibration<double> *calib,
                           basalt::VioConfig *config) {
  std::string calibFile;
  if (!nh.getParam("calibration_file", calibFile)) {
    ROS_ERROR_STREAM("must specify calibration_file!");
    throw std::invalid_argument("no calibration_file specified");
  }
  load_calib(calib, calibFile);

  // load config
  config->optical_flow_skip_frames = 1;
  std::string vioConfigFile;
  if (nh.getParam("vio_config_file", vioConfigFile)) {
    config->load(vioConfigFile);
  }
  nh.param<bool>("debug_vio", config->vio_debug, false);
  nh.param<bool>("debug_bad_data", config->vio_debug_bad_data, false);
}
}  // namespace basalt_ros1
