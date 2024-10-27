#include <geometry_msgs/msg/pose_stamped.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <linkattacher_msgs/srv/attach_link.hpp>   
#include <linkattacher_msgs/srv/detach_link.hpp> 


int main(int argc, char** argv){

    rclcpp::init(argc, argv);
    bool attach = false;
    // creating node
    auto const node = std::make_shared<rclcpp::Node>(
        "pick_and_place",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
    );
    auto const logger = rclcpp::get_logger("pick and place node is running...");
     
    //  ----------- Attach service client---------------
    auto client_attach = node->create_client<linkattacher_msgs::srv::AttachLink>("/ATTACHLINK");
    // Wait for the service to become available
    while (!client_attach->wait_for_service(std::chrono::seconds(1))){
        RCLCPP_INFO(node->get_logger(), "Waiting for the Attach service to be available...");
    }
    auto request_attach = std::make_shared<linkattacher_msgs::srv::AttachLink::Request>();
    request_attach->model1_name = "mycobot_280";  // First integer to add
    request_attach->link1_name = "gripper_right3";  // Second integer to add
    request_attach->model2_name = "wood_cube_blue";
    request_attach->link2_name = "link";

    rclcpp::sleep_for(std::chrono::seconds(2));

    //  ----------- Dittach service client---------------
    auto client_detach = node->create_client<linkattacher_msgs::srv::DetachLink>("/DETACHLINK");
    // Wait for the service to become available
    while (!client_detach->wait_for_service(std::chrono::seconds(1))){
        RCLCPP_INFO(node->get_logger(), "Waiting for the Detach service to be available...");
    }
    auto request_detach = std::make_shared<linkattacher_msgs::srv::DetachLink::Request>();
    request_detach->model1_name = "mycobot_280";  // First integer to add
    request_detach->link1_name = "gripper_right3";  // Second integer to add
    request_detach->model2_name = "wood_cube_blue";
    request_detach->link2_name = "link";

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

    // Blue cube pose
    auto arm_target_pose_1 = [&node] {
        geometry_msgs::msg::PoseStamped msg;
        msg.header.frame_id = "base_link";
        msg.header.stamp = node->now();
        msg.pose.position.x = 0.19;
        msg.pose.position.y = -0.17;
        msg.pose.position.z = 0.1;
        msg.pose.orientation.x = 1.0;
        msg.pose.orientation.y = 0.0;
        msg.pose.orientation.z = 0.0;
        msg.pose.orientation.w = 0.0;
        return msg;
    }();
    target_poses.push_back(arm_target_pose_1);

    // bowl pose
    auto arm_target_pose_2 = [&node] {
        geometry_msgs::msg::PoseStamped msg;
        msg.header.frame_id = "base_link";
        msg.header.stamp = node->now();
        msg.pose.position.x = -0.15;
        msg.pose.position.y = 0.05;
        msg.pose.position.z = 0.1;
        msg.pose.orientation.x = 1.0;
        msg.pose.orientation.y = 0.0;
        msg.pose.orientation.z = 0.0;
        msg.pose.orientation.w = 0.0;
        return msg;
    }();
    target_poses.push_back(arm_target_pose_2);

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
        if(!attach){
            // calling Attach service if near the target
            auto future_attach = client_attach->async_send_request(request_attach);
            if (rclcpp::spin_until_future_complete(node, future_attach) == rclcpp::FutureReturnCode::SUCCESS) {
                RCLCPP_INFO(node->get_logger(), "Result of Attach: %d", future_attach.get()->success);
                attach = true;
                } else {
                    RCLCPP_ERROR(node->get_logger(), "Failed to call service attachlink");
                }
        }else{
            // calling Detach service if near the target
            auto future_detach = client_detach->async_send_request(request_detach);
            if (rclcpp::spin_until_future_complete(node, future_detach) == rclcpp::FutureReturnCode::SUCCESS) {
                RCLCPP_INFO(node->get_logger(), "Result of Detach: %d", future_detach.get()->success);
                attach = false;
                } else {
                    RCLCPP_ERROR(node->get_logger(), "Failed to call service Detachlink");
                }
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