#include <geometry_msgs/msg/pose_stamped.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <example_interfaces/srv/add_two_ints.hpp>  // Replace with the correct path to your service definition


int main(int argc, char** argv){

    rclcpp::init(argc, argv);
    
    // creating node
    auto const node = std::make_shared<rclcpp::Node>(
        "pick_and_place",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
    );
    auto const logger = rclcpp::get_logger("pick and place node has been created !");
     
    //  -----------service client---------------
    // creating service client
    auto client = node->create_client<example_interfaces::srv::AddTwoInts>("sum_service");
    // Wait for the service to become available
    while (!client->wait_for_service(std::chrono::seconds(1))){
        RCLCPP_INFO(node->get_logger(), "Waiting for the service to be available...");
    }
    auto request = std::make_shared<example_interfaces::srv::AddTwoInts::Request>();
    request->a = 10;  // First integer to add
    request->b = 20;  // Second integer to add

    // ------------moveit2 api configurations---------------
    using moveit::planning_interface::MoveGroupInterface;
    auto arm_group_interface = MoveGroupInterface(node, "arm");
    // planning parameters
    arm_group_interface.setPlanningPipelineId("ompl");
    arm_group_interface.setPlannerId("RRTConnectkConfigDefault");
    arm_group_interface.setPlanningTime(5.0);
    arm_group_interface.setMaxVelocityScalingFactor(1.0);
    arm_group_interface.setMaxAccelerationScalingFactor(1.0);
    // Logging planning parameters
    RCLCPP_INFO(logger, "Planning pipeline: %s", arm_group_interface.getPlanningPipelineId().c_str());
    RCLCPP_INFO(logger, "Planner ID: %s", arm_group_interface.getPlannerId().c_str());
    RCLCPP_INFO(logger, "Planning time: %.2f", arm_group_interface.getPlanningTime());

    // created vector for multi pose
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
        msg.pose.position.x = 0.1;
        msg.pose.position.y = -0.1;
        msg.pose.position.z = 0.15;
        msg.pose.orientation.x = 1.0;
        msg.pose.orientation.y = 0.0;
        msg.pose.orientation.z = 0.0;
        msg.pose.orientation.w = 0.0;
        return msg;
    }();
    target_poses.push_back(arm_target_pose_2);

    // Third target pose
    auto arm_target_pose_3 = [&node] {
        geometry_msgs::msg::PoseStamped msg;
        msg.header.frame_id = "base_link";
        msg.header.stamp = node->now();
        msg.pose.position.x = 0.1;
        msg.pose.position.y = -0.1;
        msg.pose.position.z = 0.2;
        msg.pose.orientation.x = 1.0;
        msg.pose.orientation.y = 0.0;
        msg.pose.orientation.z = 0.0;
        msg.pose.orientation.w = 0.0;
        return msg;
    }();
    target_poses.push_back(arm_target_pose_3);

    // iterating through each poses
    for(const auto& target_pose : target_poses){
        arm_group_interface.setPoseTarget(target_pose);

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
        rclcpp::sleep_for(std::chrono::seconds(2));

        // calling service
        auto future = client->async_send_request(request);
        if (rclcpp::spin_until_future_complete(node, future) == rclcpp::FutureReturnCode::SUCCESS) {
            RCLCPP_INFO(node->get_logger(), "Result of addtwoints: %ld", future.get()->sum);
            } else {
                RCLCPP_ERROR(node->get_logger(), "Failed to call service addtwoints");
            }

        RCLCPP_INFO(logger, "Waiting for 2 seconds before moving to the next pose...");
        rclcpp::sleep_for(std::chrono::seconds(2));  // 2-second delay
        }
        else
        {
        RCLCPP_ERROR(logger, "Planning failed for this target pose!");
        }
    }



    rclcpp::shutdown();
    return 0;
}