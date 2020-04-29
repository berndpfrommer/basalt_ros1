/* -*-c++-*--------------------------------------------------------------------
 * 2020 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#include <basalt_ros1/optical_flow_subscriber.h>

#include <boost/range/irange.hpp>

#include <functional>

using boost::irange;

using namespace std::placeholders;

namespace basalt_ros1 {
OpticalFlowSubscriber::OpticalFlowSubscriber(
  const ros::NodeHandle &node, const std::string& topic)
    : node_(node) {
  sub_ = node_.subscribe(topic, 10, &OpticalFlowSubscriber::callback, this);
}

void OpticalFlowSubscriber::callback(OpticalFlowMsgConstPtr const& msg) {
  if (!queue_) {
    return;
  }

  basalt::OpticalFlowResult::Ptr data(new basalt::OpticalFlowResult());
  data->t_ns = msg->header.stamp.sec * 1000000000LL + msg->header.stamp.nsec;
  data->observations.resize(msg->observations.size());
  for (const auto obs_idx : irange(0ul, msg->observations.size())) {
    const auto& msg_obs = msg->observations[obs_idx];
    auto& data_obs = data->observations[obs_idx];
    for (const auto kp_idx : irange(0ul, msg_obs.keypoints.size())) {
      const auto kp_id = msg_obs.keypoints[kp_idx].id;
      // copy affine transform
      for (const auto i : irange(0, 6)) {
        data_obs[kp_id].data()[i] =
            msg_obs.keypoints[kp_idx].affine_transform[i];
      }
    }
  }
  if (queue_) {
    if (!queue_->try_push(data)) {
      ROS_WARN_STREAM("optical flow data dropped due to overflow: q_len = "
                      << queue_->size());
    }
    max_q_ = std::max(queue_->size(), max_q_);
  }
}
}  // namespace basalt_ros1
