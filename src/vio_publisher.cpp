/* -*-c++-*--------------------------------------------------------------------
 * 2020 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#include <basalt_ros1/vio_publisher.h>

#include <fstream>
#include <ios>
#include <stdexcept>

static geometry_msgs::Point to_ros_point(const Eigen::Vector3d &v) {
  geometry_msgs::Point vv;
  vv.x = v[0];
  vv.y = v[1];
  vv.z = v[2];
  return (vv);
}

static geometry_msgs::Vector3 to_ros_vec(const Eigen::Vector3d &v) {
  geometry_msgs::Vector3 vv;
  vv.x = v[0];
  vv.y = v[1];
  vv.z = v[2];
  return (vv);
}

static geometry_msgs::Quaternion to_ros_quat(const Eigen::Quaterniond &q) {
  geometry_msgs::Quaternion qq;
  qq.x = q.x();
  qq.y = q.y();
  qq.z = q.z();
  qq.w = q.w();
  return (qq);
}

static geometry_msgs::TransformStamped to_tf_msg(
    const ros::Time &t, const Eigen::Quaterniond &q,
    const Eigen::Vector3d &trans, const std::string &parent,
    const std::string &child) {
  geometry_msgs::TransformStamped tf;
  tf.header.stamp = t;
  tf.header.frame_id = parent;
  tf.child_frame_id = child;

  tf.transform.translation = to_ros_vec(trans);
  tf.transform.rotation = to_ros_quat(q);
  return (tf);
}

namespace basalt_ros1 {
VIOPublisher::VIOPublisher(const ros::NodeHandle &node) : node_(node) {
  tfBroadCaster_ = std::make_shared<tf2_ros::TransformBroadcaster>();
  pub_ = node_.advertise<nav_msgs::Odometry>("odom", 10);
  // we don't do covariance quite yet....
  cov_ = {0.01, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.01, 0.00,
          0.00, 0.00, 0.00, 0.00, 0.00, 0.01, 0.00, 0.00, 0.00,
          0.00, 0.00, 0.00, 0.01, 0.00, 0.00, 0.00, 0.00, 0.00,
          0.00, 0.01, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.01};
  node_.param<std::string>("world_frame_id", msg_.header.frame_id, "world");
  node_.param<std::string>("odom_frame_id", msg_.child_frame_id, "odom");
}

void VIOPublisher::publish(const basalt::PoseVelBiasState::Ptr &data) {
  const Eigen::Vector3d T = data->T_w_i.translation();
  const Eigen::Quaterniond q = data->T_w_i.unit_quaternion();
  const Eigen::Vector3d ang_vel = data->vel_w_i;

  // make odometry message
  msg_.header.stamp.sec = data->t_ns / 1000000000LL;
  msg_.header.stamp.nsec = data->t_ns % 1000000000LL;

  msg_.pose.pose.position = to_ros_point(T);
  msg_.pose.pose.orientation = to_ros_quat(q);

  msg_.pose.covariance = cov_;
  msg_.twist.twist.linear = to_ros_vec(ang_vel);
  msg_.twist.covariance = cov_;  // zero matrix

  pub_.publish(msg_);
#if 0
  std::cout << "position: " << msg_.pose.pose.position.x << " "
            << msg_.pose.pose.position.y << " " << msg_.pose.pose.position.z
            << std::endl;
#endif
  // make transform message
  const ros::Time t(msg_.header.stamp.sec, msg_.header.stamp.nsec);
  const geometry_msgs::TransformStamped tf =
      to_tf_msg(t, q, T, msg_.header.frame_id, msg_.child_frame_id);

  tfBroadCaster_->sendTransform(tf);
}

}  // namespace basalt_ros1
