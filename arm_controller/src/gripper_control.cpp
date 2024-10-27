#include "rclcpp/rclcpp.hpp"
#include "robot_interface/srv/gripper_command.hpp"

class SumClientNode : public rclcpp::Node
{
public: 
    SumClientNode(float grip_value) : Node("sum_client_node")
    {   
        RCLCPP_INFO(this->get_logger(), "Service client C++ node has been created");


        //Parameters for AddTwoInts
        a_ = 0.0;
        b_ = grip_value;
        threads_.push_back(std::thread(std::bind(&SumClientNode::CallGripperServer, this, b_)));
    }
    void CallGripperServer(float value)
    {
        //create client int
        client_ = this->create_client<robot_interface::srv::GripperCommand>("gripper_cmd");
        while (!client_->wait_for_service(std::chrono::seconds(1))){
            RCLCPP_WARN(this->get_logger(), "Waiting for the Server...");
        }
        
        //create request message
        auto request = std::make_shared<robot_interface::srv::GripperCommand::Request>();
        request->opening = value;
        

        // future = client.call_async
        auto future = client_->async_send_request(request);
        RCLCPP_INFO(this->get_logger(), "value sent: %f", (float)request->opening);
        try{
            auto response = future.get();
            // response->status ? "Completed":"not Completed"
            RCLCPP_INFO(this->get_logger(), "Gripper closed : %d", (int)response->status);
        }
        catch(const std::exception& e){
            RCLCPP_ERROR(this->get_logger(), "Service call failed");
        }
    }
private:
    rclcpp::Client<robot_interface::srv::GripperCommand>::SharedPtr client_;
    std::vector<std::thread> threads_;
    float a_, b_;
};  

int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    float value = std::stod(argv[1]);
    auto node = std::make_shared<SumClientNode>(value);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
