#include <iostream>
#include <map>
#include <unordered_map>
#include <unordered_set>
#include <vector>
// ROS
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
// Motion Capture
#include <libmotioncapture/motioncapture.h>


int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("motion_capture_tracking_node");
  node->declare_parameter<std::string>("type", "vicon");
  node->declare_parameter<std::string>("hostname", "localhost");
  // ToDo: maybe we should log all tracked bodies in file.
  node->declare_parameter<std::string>("logfilepath", "");
  node->declare_parameter<bool>("publish_stamped_poses", false);
  node->declare_parameter<bool>("use_host_time", true);


  std::vector<std::string> publish_poses_for; // list from ros param that contains topic assignment
  std::vector<std::string> publish_poses_for_clean;  // clean list that only contains the target names
  std::unordered_map<std::string, std::string> map_target_topic_name;
  node->declare_parameter<std::vector<std::string> >("publish_stamped_poses_for", publish_poses_for);

  const std::string motionCaptureType = node->get_parameter("type").as_string();
  RCLCPP_INFO_STREAM(node->get_logger(), "motion capture type: " << motionCaptureType);
  const std::string motionCaptureHostname = node->get_parameter("hostname").as_string();
  RCLCPP_INFO_STREAM(node->get_logger(), "hostname: " << motionCaptureHostname);
  const auto use_host_time = node->get_parameter("use_host_time").as_bool();
  RCLCPP_INFO_STREAM(node->get_logger(), "Use host time: " << (use_host_time ? "true" : "false"));
  const auto publish_stamped_poses = node->get_parameter("publish_stamped_poses").as_bool();
  RCLCPP_INFO_STREAM(node->get_logger(), "Publish stamped poses: " << (publish_stamped_poses ? "true" : "false"));

  publish_poses_for = node->get_parameter("publish_stamped_poses_for").as_string_array();
  auto publish_stamped_poses_all = false;
  if (publish_stamped_poses && publish_poses_for.empty()) {
    RCLCPP_WARN(node->get_logger(),
                "Publish stamped poses set but no target has been assigned, publishing all targets' poses");
    publish_stamped_poses_all = true;
  }
  if (!publish_poses_for.empty()) {
    publish_poses_for_clean.reserve(publish_poses_for.size());
    RCLCPP_INFO(node->get_logger(), "Publish stamped poses for the following targets:");
    for (const auto &target: publish_poses_for) {
      const size_t pos = target.find(':');
      if (pos != std::string::npos) {
        const auto target_name = target.substr(0, pos);
        const auto target_topic = target.substr(pos + 1);
        map_target_topic_name.emplace(
          target_name,
          target_topic);
        RCLCPP_INFO_STREAM(node->get_logger(), "--- " << target_name << " with topic: " << target_topic);
        publish_poses_for_clean.push_back(target_name);
      }
      else {
        publish_poses_for_clean.push_back(target);
        RCLCPP_INFO_STREAM(node->get_logger(), "--- " << target);
      }
    }
  }

  std::string logFilePath = node->get_parameter("logfilepath").as_string();
  RCLCPP_INFO_STREAM(node->get_logger(), "logfilepath: " << logFilePath);

  std::unordered_map<std::string, rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr> map_target_pose_publishers;
  std::unordered_set<std::string> publish_poses_for_set(publish_poses_for_clean.begin(), publish_poses_for_clean.end());

  RCLCPP_INFO(node->get_logger(), " ****** Starting MotionCapture Interface ******* ");
  auto node_parameters_iface = node->get_node_parameters_interface();

  // Make a new client
  std::map<std::string, std::string> cfg;
  cfg["hostname"] = motionCaptureHostname;

  libmotioncapture::MotionCapture *mocap = libmotioncapture::MotionCapture::connect(motionCaptureType, cfg);

  // prepare TF broadcaster
  tf2_ros::TransformBroadcaster tfbroadcaster(node);
  std::vector<geometry_msgs::msg::TransformStamped> transforms;

  // lambda functions
  auto ensure_map_target_publisher = [&map_target_pose_publishers, &map_target_topic_name, node](const std::string &name) {
    if (!map_target_pose_publishers.contains(name)) {
      std::string topic;
      if (map_target_topic_name.contains(name))
        topic = map_target_topic_name.at(name);
      else
        topic = "stamped_pose_" + name;
      auto pub = node->create_publisher<geometry_msgs::msg::PoseStamped>(topic,
                                                                rclcpp::SystemDefaultsQoS());
      map_target_pose_publishers.emplace(
        name,
        std::move(pub));
    }
  };

  auto generate_and_publish_stamped_pose = [&map_target_pose_publishers](const std::string &name,
                                                                         const rclcpp::Time &time,
                                                                         const auto &rigidBody) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = time;
    pose.header.frame_id = name;

    pose.pose.position.x = rigidBody.position().x();
    pose.pose.position.y = rigidBody.position().y();
    pose.pose.position.z = rigidBody.position().z();

    pose.pose.orientation.x = rigidBody.rotation().x();
    pose.pose.orientation.y = rigidBody.rotation().y();
    pose.pose.orientation.z = rigidBody.rotation().z();
    pose.pose.orientation.w = rigidBody.rotation().w();
    std::cout << "publishing " << name << std::endl;
    map_target_pose_publishers.at(name)->publish(pose);
  };

  while ( rclcpp::ok() ) {
    // Get a frame
    mocap->waitForNextFrame();

    rclcpp::Time time;
    if (use_host_time)
       time = node->now();
    else
      time = rclcpp::Time(mocap->timeStamp());

    transforms.clear();
    transforms.reserve(mocap->rigidBodies().size());
    for (const auto &iter: mocap->rigidBodies()) {
      const auto &rigidBody = iter.second;
      const auto name = rigidBody.name();

      if (publish_stamped_poses &&
          (publish_stamped_poses_all || publish_poses_for_set.contains(name))) {
        ensure_map_target_publisher(name);
        generate_and_publish_stamped_pose(name, time, rigidBody);
      }

      transforms.resize(transforms.size() + 1);
      transforms.back().header.stamp = time;
      transforms.back().header.frame_id = "world";
      transforms.back().child_frame_id = name;
      transforms.back().transform.translation.x = rigidBody.position().x();
      transforms.back().transform.translation.y = rigidBody.position().y();
      transforms.back().transform.translation.z = rigidBody.position().z();
      transforms.back().transform.rotation.x = rigidBody.rotation().x();
      transforms.back().transform.rotation.y = rigidBody.rotation().y();
      transforms.back().transform.rotation.z = rigidBody.rotation().z();
      transforms.back().transform.rotation.w = rigidBody.rotation().w();
    }

    if (!transforms.empty()) {
      // send TF. Since RViz and others can't handle nan's, report a fake orientation if needed
      for (auto &tf: transforms) {
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
