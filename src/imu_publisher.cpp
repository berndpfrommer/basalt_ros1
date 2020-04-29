/* -*-c++-*--------------------------------------------------------------------
 * 2020 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#include <basalt_ros1/imu_publisher.h>

namespace basalt_ros1 {
IMUPublisher::IMUPublisher(const ros::NodeHandle &node,
                           const std::string &topic)
    : node_(node), topic_(topic) {
  pub_ = node_.advertise<Imu>(topic, 10);
  lastTime_ = ros::Time::now();
}

void IMUPublisher::processingThread() {
  basalt::ImuData::Ptr data;
  for (;;) {
    inputQueue_.pop(data);
    if (!data) {
      break;  // done!
    }
    if (pub_.getNumSubscribers() != 0) {
      Imu msg;
      msg.header.frame_id = "frame_id";  // XXX make configurable
      msg.header.stamp.sec = data->t_ns / 1000000000LL;
      msg.header.stamp.nsec = data->t_ns % 1000000000LL;

      msg.linear_acceleration.x = data->accel(0);
      msg.linear_acceleration.y = data->accel(1);
      msg.linear_acceleration.z = data->accel(2);

      msg.angular_velocity.x = data->gyro(0);
      msg.angular_velocity.y = data->gyro(1);
      msg.angular_velocity.z = data->gyro(2);
      pub_.publish(msg);
    }
    framesPublished_++;
    if (framesPublished_ == 1000) {
      const auto t = ros::Time::now();
      const auto dt = t - lastTime_;
      ROS_INFO_STREAM("published imu frame rate: " << framesPublished_ /
                                                          dt.toSec());
      framesPublished_ = 0;
      lastTime_ = t;
    }
  }
}

void IMUPublisher::start() {
  thread_ = std::thread(&IMUPublisher::processingThread, this);
}

}  // namespace basalt_ros1
