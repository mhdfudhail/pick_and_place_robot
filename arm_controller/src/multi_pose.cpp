#include <geometry_msgs/msg/pose_stamped.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

int main(int argc, char * argv[])
{
  // Start up ROS 2
  rclcpp::init(argc, argv);

  // Creates a node named "hello_moveit"
  auto const node = std::make_shared<rclcpp::Node>(
    "hello_moveit",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  // Create a logger
  auto const logger = rclcpp::get_logger("hello_moveit");

  // Create the MoveIt MoveGroup Interface for the arm
  using moveit::planning_interface::MoveGroupInterface;
  auto arm_group_interface = MoveGroupInterface(node, "arm");

  // Set planning parameters
  arm_group_interface.setPlanningPipelineId("ompl");
  arm_group_interface.setPlannerId("RRTConnectkConfigDefault");
  arm_group_interface.setPlanningTime(5.0);
  arm_group_interface.setMaxVelocityScalingFactor(1.0);
  arm_group_interface.setMaxAccelerationScalingFactor(1.0);

  // Logging planning parameters
  RCLCPP_INFO(logger, "Planning pipeline: %s", arm_group_interface.getPlanningPipelineId().c_str());
  RCLCPP_INFO(logger, "Planner ID: %s", arm_group_interface.getPlannerId().c_str());
  RCLCPP_INFO(logger, "Planning time: %.2f", arm_group_interface.getPlanningTime());

  // Define multiple target poses for the end effector
  std::vector<geometry_msgs::msg::PoseStamped> target_poses;

  // First target pose
  auto arm_target_pose_1 = [&node] {
    geometry_msgs::msg::PoseStamped msg;
    msg.header.frame_id = "base_link";
    msg.header.stamp = node->now();
    msg.pose.position.x = 0.1;
    msg.pose.position.y = -0.1;
    msg.pose.position.z = 0.1;
    msg.pose.orientation.x = 1.0;
    msg.pose.orientation.y = 0.0;
    msg.pose.orientation.z = 0.0;
    msg.pose.orientation.w = 0.0;
    return msg;
  }();
  target_poses.push_back(arm_target_pose_1);

  // Second target pose
  auto arm_target_pose_2 = [&node] {
    geometry_msgs::msg::PoseStamped msg;
    msg.header.frame_id = "base_link";
    msg.header.stamp = node->now();
    msg.pose.position.x = 0.15;
    msg.pose.position.y = -0.1;
    msg.pose.position.z = 0.2;
    msg.pose.orientation.x = 0.707;
    msg.pose.orientation.y = 0.0;
    msg.pose.orientation.z = 0.707;
    msg.pose.orientation.w = 0.0;
    return msg;
  }();
  target_poses.push_back(arm_target_pose_2);

  // Third target pose (add more as needed)
  auto arm_target_pose_3 = [&node] {
    geometry_msgs::msg::PoseStamped msg;
    msg.header.frame_id = "base_link";
    msg.header.stamp = node->now();
    msg.pose.position.x = 0.2;
    msg.pose.position.y = -0.1;
    msg.pose.position.z = 0.2;
    msg.pose.orientation.x = 0.707;
    msg.pose.orientation.y = 0.0;
    msg.pose.orientation.z = 0.707;
    msg.pose.orientation.w = 0.0;
    return msg;
  }();
  target_poses.push_back(arm_target_pose_3);

  // Loop through each pose, plan, and execute sequentially
  for (const auto& target_pose : target_poses)
  {
    // Set the pose target for the arm
    arm_group_interface.setPoseTarget(target_pose);

    // Plan the motion to that target pose
    auto const [success, plan] = [&arm_group_interface] {
      moveit::planning_interface::MoveGroupInterface::Plan msg;
      auto const ok = static_cast<bool>(arm_group_interface.plan(msg));
      return std::make_pair(ok, msg);
    }();

    // Execute the movement plan if it was created successfully
    if (success)
    {
      RCLCPP_INFO(logger, "Executing movement to target pose...");
      arm_group_interface.execute(plan);

      RCLCPP_INFO(logger, "Waiting for 2 seconds before moving to the next pose...");
      rclcpp::sleep_for(std::chrono::seconds(2));  // 2-second delay
    }
    else
    {
      RCLCPP_ERROR(logger, "Planning failed for this target pose!");
    }

  }

  // Shut down ROS 2 cleanly when done
  rclcpp::shutdown();
  return 0;
}