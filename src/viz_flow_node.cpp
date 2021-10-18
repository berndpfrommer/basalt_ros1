/* -*-c++-*--------------------------------------------------------------------
 * 2020 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#include <image_transport/image_transport.h>
#include <basalt_vio_ros1_msgs/OpticalFlowResult.h>
#include <sensor_msgs/Image.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <cv_bridge/cv_bridge.h>
#include <boost/range/irange.hpp>

#include <ros/ros.h>

using boost::irange;

class VizFlow {
public:
  typedef basalt_vio_ros1_msgs::OpticalFlowResult OptFlow;
  typedef sensor_msgs::Image Image;
  VizFlow(const ros::NodeHandle &nh) : node_(nh) {
  }
  void initialize() {
    sync_ = std::make_shared<
      message_filters::TimeSynchronizer<Image, OptFlow>>(10);
    it_ = std::make_shared<image_transport::ImageTransport>(node_);
    camSub_.subscribe(node_, "left_image", 2);
    flowSub_.subscribe(node_, "optical_flow", 2);
    sync_->connectInput(camSub_, flowSub_);
    sync_->registerCallback(&VizFlow::flowCallback, this);
    pub_ = it_->advertise("flow_image", 2);
  }
private:
 void flowCallback(const Image::ConstPtr& imgMsg,
                   const OptFlow::ConstPtr& optFlow) {
   const auto imgPtr =
       cv_bridge::toCvShare(imgMsg, sensor_msgs::image_encodings::MONO8);
   const int h = imgPtr->image.rows;
   const int w = imgPtr->image.cols;
   cv::Mat img(h, w, CV_8UC3);
   cv::cvtColor(imgPtr->image, img, CV_GRAY2RGB);
   cv::Scalar kpColor(0, 255, 0);
   for (const auto obs_idx : irange(size_t{0}, optFlow->observations.size())) {
     const auto& msg_obs = optFlow->observations[obs_idx];
     for (const auto kp_idx : irange(size_t{0}, msg_obs.keypoints.size())) {
       cv::Point2f p(msg_obs.keypoints[kp_idx].affine_transform[4],
                     msg_obs.keypoints[kp_idx].affine_transform[5]);
       cv::circle(img, p, 3, kpColor, -1);
     }
   }
   cv_bridge::CvImage outMsg(imgMsg->header, "bgr8", img);
   pub_.publish(outMsg.toImageMsg());
 }
 // ----------- variables ---------
 ros::NodeHandle node_;
 message_filters::Subscriber<Image> camSub_;
 message_filters::Subscriber<OptFlow> flowSub_;
 image_transport::Publisher pub_;

 std::shared_ptr<image_transport::ImageTransport> it_;
 std::shared_ptr<message_filters::TimeSynchronizer<Image, OptFlow>> sync_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "viz_flow");
  ros::NodeHandle pnh("~");

  try {
    VizFlow node(pnh);
    ROS_INFO_STREAM("init viz node");
    node.initialize();
    ros::spin();
  } catch (const std::exception& e) {
    ROS_ERROR("%s: %s", pnh.getNamespace().c_str(), e.what());
    return (-1);
  }
  return (0);
}
