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
}  // namespace basalt_ros1
