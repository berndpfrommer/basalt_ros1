/* -*-c++-*--------------------------------------------------------------------
 * 2020 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#include <basalt_ros1/optical_flow_publisher.h>
#include <boost/range/irange.hpp>

using boost::irange;

namespace basalt_ros1 {
OpticalFlowPublisher::OpticalFlowPublisher(
  const ros::NodeHandle &node, const std::string &topic)
    : node_(node), topic_(topic) {
  pub_ = node_.advertise<basalt_vio_ros1_msgs::OpticalFlowResult>(
      topic, 10);
}

void OpticalFlowPublisher::processingThread() {
  basalt::OpticalFlowResult::Ptr data;
  for (;;) {
    inputQueue_.pop(data);
    if (!data) {
      break;  // done!
    }
    if (pub_.getNumSubscribers() != 0) {
      basalt_vio_ros1_msgs::OpticalFlowResult msg;
      msg.header.frame_id = "frame_id";  // XXX make configurable
      msg.header.stamp.sec = data->t_ns / 1000000000LL;
      msg.header.stamp.nsec = data->t_ns % 1000000000LL;
      msg.observations.resize(data->observations.size());
      int obs_idx = 0;
      for (const Eigen::aligned_map<basalt::KeypointId, Eigen::AffineCompact2f>
               &map : data->observations) {
        msg.observations[obs_idx].keypoints.resize(map.size());
        int kp_idx = 0;
        for (const auto &kp : map) {
          basalt_vio_ros1_msgs::KeyPointToAffine kp2Aff;
          kp2Aff.id = kp.first;
          for (const auto i : irange(0, 6)) {
            kp2Aff.affine_transform[i] = kp.second.data()[i];
          }
          msg.observations[obs_idx].keypoints[kp_idx] = kp2Aff;
          kp_idx++;
        }
        obs_idx++;
      }
      pub_.publish(msg);
    }
  }
}

void OpticalFlowPublisher::start() {
  thread_ = std::thread(&OpticalFlowPublisher::processingThread, this);
}

}  // namespace basalt_ros1
