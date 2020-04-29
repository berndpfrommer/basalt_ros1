/* -*-c++-*--------------------------------------------------------------------
 * 2020 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#pragma once
#include <basalt/calibration/calibration.hpp>
#include <string>

namespace basalt_ros1 {
void load_calib(basalt::Calibration<double> *calib,
                const std::string &calib_path);
}  // namespace basalt_ros1
