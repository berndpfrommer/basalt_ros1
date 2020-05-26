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

static void tf_to_odom_msg(const tf2::Transform &tf, nav_msgs::Odometry &msg ){
    msg.pose.pose.position.x = tf.getOrigin().getX();
    msg.pose.pose.position.y = tf.getOrigin().getY();
    msg.pose.pose.position.z = tf.getOrigin().getZ();
    msg.pose.pose.orientation.x = tf.getRotation().getX();
    msg.pose.pose.orientation.y = tf.getRotation().getY();
    msg.pose.pose.orientation.z = tf.getRotation().getZ();
    msg.pose.pose.orientation.w = tf.getRotation().getW();
}

namespace basalt_ros1 {
VIOPublisher::VIOPublisher(const ros::NodeHandle &node) : node_(node) {
  tfBroadCaster_ = std::make_shared<tf2_ros::TransformBroadcaster>();
  pub_ = node_.advertise<nav_msgs::Odometry>("odom", 10);
  pub_flu_ = node_.advertise<nav_msgs::Odometry>("odom_flu",10);
  // we don't do covariance quite yet....
  cov_ = {0.01, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.01, 0.00,
          0.00, 0.00, 0.00, 0.00, 0.00, 0.01, 0.00, 0.00, 0.00,
          0.00, 0.00, 0.00, 0.01, 0.00, 0.00, 0.00, 0.00, 0.00,
          0.00, 0.01, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.01};
  node_.param<std::string>("world_frame_id", msg_.header.frame_id, "world");
  node_.param<std::string>("odom_frame_id", msg_.child_frame_id, "odom");

  node_.param<bool>("odom_flu", odom_flu_, false);
  node_.param<bool>("tf_flu", tf_flu_, false);
  node_.param<std::string>("base_link", base_link_, "t265_pose_frame");
  node_.param<std::string>("odom_link", odom_link_, "t265_gyro_optical_frame");
  node_.param<std::string>("world_flu_link", world_flu_link_, "0/map");
  //get static TF _gyro_optical_link -> _pose_frame
  std::unique_ptr<tf2_ros::Buffer>tf2_buffer = std::make_unique<tf2_ros::Buffer>(ros::Duration(100));
  std::unique_ptr<tf2_ros::TransformListener> tf2_listener =
            std::make_unique<tf2_ros::TransformListener>(*tf2_buffer, true);
  tf2_buffer->setUsingDedicatedThread(true);

  if(odom_flu_){
      ros::Rate rate(5);
      bool got_tf_offset = false;
      while (!got_tf_offset){
          try {
            //TFab|TF_A_B = position b in a frame | posiiton b wrt a
            //or TF_A_2_B aka TF_target_2_source
            // !!!lookupTransform(target_frame, source_frame...)--> returns position of source_frame wrt target_frame!!!!
            tf2::fromMsg(tf2_buffer->lookupTransform(odom_link_, base_link_, //target_frame source_frame
              ros::Time(0), ros::Duration(5.0)), T_s_b);

          } catch (tf2::LookupException e) {
            ROS_ERROR("[BASALTVIOPUB] No transform %s to %s: %s",
               odom_link_.c_str(), base_link_.c_str(), e.what());
            rate.sleep();
            continue;
          }
          got_tf_offset = true;
          ROS_INFO("[BASALTVIOPUBn] Initialzed got tf:\n"
            "target: %s(odom_link) wrt source: %s(base_link)", odom_link_.c_str(), base_link_.c_str());
          msg_flu_.header.frame_id = world_flu_link_;
          msg_flu_.child_frame_id = base_link_;
       }
    }//convert_flu_


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

  if(odom_flu_){
    //T_w_i -> T_m_b = T_mapflu_base_link
    //T_m_b = (T_w_i * T_i_b) * T_m_w == aka basalt_fix
    tf2::Quaternion q(msg_.pose.pose.orientation.x,
                      msg_.pose.pose.orientation.y,
                      msg_.pose.pose.orientation.z,
                      msg_.pose.pose.orientation.w);
    tf2::Vector3 trans(msg_.pose.pose.position.x,
                       msg_.pose.pose.position.y,
                       msg_.pose.pose.position.z);
    tf2::Transform odom_S(q, trans);
    tf2::Transform odom_B = odom_S * T_s_b;
    tf2::Transform rot_yaw_flu(tf2::Matrix3x3(0, -1, 0,
                                              1, 0, 0,
                                              0, 0, 1), tf2::Vector3());
    odom_B = odom_B * rot_yaw_flu;
    //apply rotation to translation
    odom_B.setOrigin(rot_yaw_flu.getBasis() * odom_B.getOrigin());
    //correct after rotation to make axis aligned on init
    odom_B.setRotation(tf2::Quaternion(
        -odom_B.getRotation().y(),
        odom_B.getRotation().x(),
        odom_B.getRotation().z(),
        odom_B.getRotation().w()));

    tf_to_odom_msg(odom_B, msg_flu_);
    msg_flu_.header.stamp.sec = msg_.header.stamp.sec;
    msg_flu_.header.stamp.nsec = msg_.header.stamp.nsec;
    pub_flu_.publish(msg_flu_);
    if(tf_flu_){
        //broadcast TF map_flu -> base_link
        tf2::Stamped<tf2::Transform> tf_odom_B(odom_B, msg_flu_.header.stamp, world_flu_link_);
        geometry_msgs::TransformStamped tfs_msg = tf2::toMsg(tf_odom_B);
        tfs_msg.child_frame_id = base_link_;
        tfBroadCaster_->sendTransform(tfs_msg);
    }

  }//convert_flu


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
