/* -*-c++-*--------------------------------------------------------------------
 * 2020 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#include <basalt_ros1/imu_subscriber.h>
#include <functional>

using namespace std::placeholders;

namespace basalt_ros1 {
IMUSubscriber::IMUSubscriber(const ros::NodeHandle &node,
                             const std::vector<std::string> &topics)
    : node_(node) {
  if (topics.size() > 1) {
    // gyro and acceleration are on different topics!
    gyroSub_ =
        node_.subscribe(topics[0], 100, &IMUSubscriber::callback_gyro, this);
    accelSub_ =
        node_.subscribe(topics[1], 100, &IMUSubscriber::callback_accel, this);
    ROS_INFO_STREAM("imu subscribing to topics: " << topics[0] << ", "
                                                  << topics[1]);
  } else if (topics.size() == 1) {
    combinedSub_ = node_.subscribe(topics[0], 100,
                                   &IMUSubscriber::callback_combined, this);
    ROS_INFO_STREAM("imu subscribing to topic: " << topics[0]);
  } else {
    ROS_ERROR("must specify 1 or 2 imu topics");
    throw std::invalid_argument("must specify 1 or 2 imu topics");
  }
  lastTime_ = ros::Time::now();
}

void IMUSubscriber::callback_gyro(ImuConstPtr const &msg) {
  // gyro data is stored until acceleration data comes in
  gyroQueue_.push_back(msg);
  printFrameRate();
}

static ros::Time stamp(ImuConstPtr const & m) {
  return (ros::Time(m->header.stamp.sec, m->header.stamp.nsec));
}

static double stamp_to_float(ImuConstPtr const & m) {
  return (m->header.stamp.sec * 1e3 + m->header.stamp.nsec * 1e-6);
}

void IMUSubscriber::callback_accel(ImuConstPtr const &accel) {
  if (!prevAccel_) {
    // need previous acceleration for interpolation
    prevAccel_ = accel;
    return;
  }
  const ros::Time prevAccelStamp = stamp(prevAccel_);
  while (!gyroQueue_.empty() && stamp(gyroQueue_.front()) < prevAccelStamp) {
    // drain old stuff
    gyroQueue_.pop_front();
  }
  const Eigen::Vector3d curr_acc(accel->linear_acceleration.x,
                                 accel->linear_acceleration.y,
                                 accel->linear_acceleration.z);
  const Eigen::Vector3d prev_acc(prevAccel_->linear_acceleration.x,
                                 prevAccel_->linear_acceleration.y,
                                 prevAccel_->linear_acceleration.z);
  // time of current accel message
  const double t_accel = stamp_to_float(accel);
  // time of previous accel message;
  const double t_prev_accel = stamp_to_float(prevAccel_);
  // total time passed since last accel message
  const double dt = t_accel - t_prev_accel;

  while (!gyroQueue_.empty()) {
    // pull gyro data out of queue (makes copy of pointer)
    const ImuConstPtr gyroMsg = gyroQueue_.front();
    const double t_gyro = stamp_to_float(gyroMsg);
    if (t_gyro >= t_accel) {
      break;  // done
    }
    gyroQueue_.pop_front();
    // interpolate acceleration data to the
    // same time stamp as the gyro data
    const double w0 = (t_accel - t_gyro) / dt;
    const double w1 = (t_gyro - t_prev_accel) / dt;

    const Eigen::Vector3d accel_interp = w0 * prev_acc + w1 * curr_acc;

    // make data packet in basalt format
    basalt::ImuData::Ptr data(new basalt::ImuData());
    data->t_ns = std::max(t_gyro, lastSentTimeStamp_) * 1e6;
    lastSentTimeStamp_ = data->t_ns;
    data->accel = accel_interp;
    data->gyro << gyroMsg->angular_velocity.x, gyroMsg->angular_velocity.y,
        gyroMsg->angular_velocity.z;
    if (queue_) {
      if (!queue_->try_push(data)) {
        ROS_WARN_STREAM("IMU data dropped due to overflow: q_len = "
                        << queue_->size());
      }
      max_q_ = std::max(queue_->size(), max_q_);
    }
  }
  prevAccel_ = accel;
}

void IMUSubscriber::printFrameRate() {
  if (++combinedFramesReceived_ == 1000) {
    const auto t = ros::Time::now();
    const auto dt = t - lastTime_;
    ROS_INFO_STREAM("received combined IMU frame rate: "
                    << combinedFramesReceived_ / dt.toSec());

    lastTime_ = t;
    combinedFramesReceived_ = 0;
  }
}

void IMUSubscriber::callback_combined(ImuConstPtr const & msg) {
  const double t = stamp_to_float(msg);
  // make data packet in basalt format
  basalt::ImuData::Ptr data(new basalt::ImuData());
  data->t_ns = std::max(t, lastSentTimeStamp_) * 1e6;
  lastSentTimeStamp_ = data->t_ns;
  data->accel << msg->linear_acceleration.x, msg->linear_acceleration.y,
      msg->linear_acceleration.z;
  data->gyro << msg->angular_velocity.x, msg->angular_velocity.y,
      msg->angular_velocity.z;
  if (queue_) {
    if (!queue_->try_push(data)) {
      ROS_WARN_STREAM("IMU data dropped due to overflow: q_len = "
                      << queue_->size());
    }
    max_q_ = std::max(queue_->size(), max_q_);
  }
  printFrameRate();
}
}  // namespace basalt_ros1
