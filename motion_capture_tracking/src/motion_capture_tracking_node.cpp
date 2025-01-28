#include <iostream>
#include <vector>
// ROS
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
// Motion Capture
#include <libmotioncapture/motioncapture.h>


int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("motion_capture_tracking_node");
  node->declare_parameter<std::string>("type", "vicon");
  node->declare_parameter<std::string>("hostname", "localhost");
  node->declare_parameter<std::string>("logfilepath", "");

  std::string motionCaptureType = node->get_parameter("type").as_string();
  std::string motionCaptureHostname = node->get_parameter("hostname").as_string();
  std::string logFilePath = node->get_parameter("logfilepath").as_string();

  auto node_parameters_iface = node->get_node_parameters_interface();

  // Make a new client
  std::map<std::string, std::string> cfg;
  cfg["hostname"] = motionCaptureHostname;

  libmotioncapture::MotionCapture *mocap = libmotioncapture::MotionCapture::connect(motionCaptureType, cfg);

  // prepare TF broadcaster
  tf2_ros::TransformBroadcaster tfbroadcaster(node);
  std::vector<geometry_msgs::msg::TransformStamped> transforms;


  for (size_t frameId = 0; rclcpp::ok(); ++frameId) {

    // Get a frame
    mocap->waitForNextFrame();
    auto time = node->now();

    transforms.clear();
    transforms.reserve(mocap->rigidBodies().size());
    for (const auto &iter : mocap->rigidBodies())
    {
      const auto& rigidBody = iter.second;

      transforms.resize(transforms.size() + 1);
      transforms.back().header.stamp = time;
      transforms.back().header.frame_id = "world";
      transforms.back().child_frame_id = rigidBody.name();
      transforms.back().transform.translation.x = rigidBody.position().x();
      transforms.back().transform.translation.y = rigidBody.position().y();
      transforms.back().transform.translation.z = rigidBody.position().z();
      transforms.back().transform.rotation.x = rigidBody.rotation().x();
      transforms.back().transform.rotation.y = rigidBody.rotation().y();
      transforms.back().transform.rotation.z = rigidBody.rotation().z();
      transforms.back().transform.rotation.w = rigidBody.rotation().w();
    }


    if (transforms.size() > 0) {
      // send TF. Since RViz and others can't handle nan's, report a fake oriention if needed
      for (auto& tf : transforms) {
        if (std::isnan(tf.transform.rotation.x)) {
          tf.transform.rotation.x = 0;
          tf.transform.rotation.y = 0;
          tf.transform.rotation.z = 0;
          tf.transform.rotation.w = 1;
        }
      }

      tfbroadcaster.sendTransform(transforms);
    }
    rclcpp::spin_some(node);
  }
  return 0;
  }
