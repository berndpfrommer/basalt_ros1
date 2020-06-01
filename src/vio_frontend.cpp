/* -*-c++-*--------------------------------------------------------------------
 * 2020 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#include <basalt_ros1/vio_frontend.h>

#include <fstream>
#include <ios>
#include <stdexcept>
#include <thread>

namespace basalt_ros1 {
VIOFrontEnd::VIOFrontEnd(const ros::NodeHandle& nh,
                         const basalt::Calibration<double>& calib,
                         const basalt::VioConfig& config)
    : node_(nh), calibration_(calib), config_(config) {}

bool VIOFrontEnd::initialize() {
  imageSub_ =
      std::make_shared<ImageSubscriber>(node_, "left_image", "right_image");
  basalt::VioConfig vio_config;
  opticalFlow_ =
      basalt::OpticalFlowFactory::getOpticalFlow(config_, calibration_);
  // connect image sub output queue to optical flow input queue
  imageSub_->setQueue(&(opticalFlow_->input_queue));

  if (opticalFlowOut_) {
    opticalFlow_->output_queue = opticalFlowOut_;
  } else {
    opticalFlowPub_ =
        std::make_shared<OpticalFlowPublisher>(node_, "optical_flow");
    opticalFlowPub_->start();
    opticalFlow_->output_queue = opticalFlowPub_->getInputQueue();
  }
  return (true);
}
}  // namespace basalt_ros1
