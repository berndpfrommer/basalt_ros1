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
  std::vector<double> ext_trans = {0, 0, 0};
  std::vector<double> ext_q = {1.0, 0, 0, 0};
  if (node_.getParam("extra_translation", ext_trans)) {
    if (ext_trans.size() != 3) {
      ROS_ERROR_STREAM("extra_translation must have 3 elements!");
      throw std::invalid_argument("extra translation must have 3 elements");
    }
  }
  if (node_.getParam("extra_rotation", ext_q)) {
    if (ext_q.size() != 4) {
      ROS_ERROR_STREAM("extra_rotation must have 4 elements!");
      throw std::invalid_argument("extra rotation must have 4 elements");
    }
  }

  T_extra_ = Eigen::Vector3d(ext_trans[0], ext_trans[1], ext_trans[2]);
  q_extra_ = Eigen::Quaterniond(ext_q[0], ext_q[1], ext_q[2], ext_q[3]);
  extraTF_ = T_extra_.norm() > 0.001 || q_extra_.w() < 0.99999;
}

void VIOPublisher::publish(const basalt::PoseVelBiasState::Ptr &data) {
  const Eigen::Vector3d T_orig = data->T_w_i.translation();
  const Eigen::Quaterniond q_orig = data->T_w_i.unit_quaternion();
  const Eigen::Vector3d ang_vel = data->vel_w_i;

  const Eigen::Vector3d T = extraTF_ ? (q_extra_ * T_orig + T_extra_) : T_orig;
  const Eigen::Quaterniond q = extraTF_ ? (q_extra_ * q_orig) : q_orig;

  // make odometry message
  msg_.header.stamp.sec = data->t_ns / 1000000000LL;
  msg_.header.stamp.nsec = data->t_ns % 1000000000LL;

  msg_.pose.pose.position = to_ros_point(T);
  msg_.pose.pose.orientation = to_ros_quat(q);

  msg_.pose.covariance = cov_;
  msg_.twist.twist.linear = to_ros_vec(ang_vel);
  msg_.twist.covariance = cov_;  // zero matrix

  pub_.publish(msg_);
  // make transform message
  const ros::Time t(msg_.header.stamp.sec, msg_.header.stamp.nsec);
  const geometry_msgs::TransformStamped tf =
      to_tf_msg(t, q, T, msg_.header.frame_id, msg_.child_frame_id);

  tfBroadCaster_->sendTransform(tf);
}

}  // namespace basalt_ros1
