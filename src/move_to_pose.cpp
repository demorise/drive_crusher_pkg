#include "rclcpp/logging.hpp"
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include <fstream>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometric_shapes/shapes.h>
#include <geometric_shapes/shape_operations.h>
#include <memory>
#include <thread>

geometry_msgs::msg::Pose applyRotation(const geometry_msgs::msg::Pose& pose, double roll, double pitch, double yaw)
{
  tf2::Quaternion q_orig, q_rot, q_new;
  tf2::convert(pose.orientation, q_orig); 
  q_rot.setRPY(roll*(M_PI/180.0), pitch*(M_PI/180.0), yaw*(M_PI/180.0));
  q_new =  q_orig*q_rot;
  q_new.normalize();
  geometry_msgs::msg::Quaternion msg_quat = tf2::toMsg(q_new);
  geometry_msgs::msg::Pose pose_new;
  pose_new.position = pose.position;
  pose_new.orientation = msg_quat;
  return pose_new;
}

int main(int argc, char ** argv)
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
      "move_to_pose", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  // Create a ROS logger
  auto const logger = rclcpp::get_logger("move_to_pose");

  // We spin up a SingleThreadedExecutor for the current state monitor to get
  // information about the robot's state.
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  auto spinner = std::thread([&executor]() { executor.spin(); });

  geometry_msgs::msg::Pose target_pose;

  auto marker_pub = node->create_publisher<visualization_msgs::msg::Marker>("target_pose", 1);

  visualization_msgs::msg::MarkerArray marker_array;
  visualization_msgs::msg::Marker marker;
  // Set the frame ID and timestamp
  marker.header.frame_id = "base_link";
  marker.header.stamp = rclcpp::Clock().now();

  // Set the namespace and id for this marker. This serves to create a unique ID
  marker.id = 0;
  marker.ns = "visualize_pose";

  // Set the marker type to a sphere
  marker.type = visualization_msgs::msg::Marker::SPHERE;;

  // Set the marker action
  marker.action = visualization_msgs::msg::Marker::ADD;

  target_pose.position.x = -0.035;
  target_pose.position.y = -0.214;
  target_pose.position.z = 0.353;
  target_pose.orientation.w = 1.0;
  target_pose = applyRotation(target_pose,-106.984, -23.960, -172.926);

  marker.pose = target_pose;

  // Set the scale of the marker
  marker.scale.x = 0.1;
  marker.scale.y = 0.1;
  marker.scale.z = 0.1;

  // Set the color
  marker.color.r = 0.0f;
  marker.color.g = 1.0f;
  marker.color.b = 0.0f;
  marker.color.a = 1.0; //alpha

  // Set the lifetime of the marker -- 0 indicates forever
  marker.lifetime = rclcpp::Duration::from_nanoseconds(0);

  // Publish the marker
  marker_pub->publish(marker);
  marker_array.markers.push_back(marker);  


  // Create the MoveIt MoveGroup Interface
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, "arm_group");

  // Set a target Pose for MOveIt
  move_group_interface.setPoseTarget(target_pose);

  auto const [success, plan] = [&move_group_interface] {
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(move_group_interface.plan(msg));
    return std::make_pair(ok, msg);
  }();

  // Execute the plan
  if (success)
    move_group_interface.execute(plan);
  else
    RCLCPP_ERROR(logger, "Planning failed!");

  // Shutdown ROS
  // rclcpp::shutdown();
  spinner.join();
  return 0;
}
