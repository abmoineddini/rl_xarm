#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "xarm_msgs/srv/plan_exec.hpp"
#include "xarm_msgs/srv/plan_joint.hpp"


using namespace std::chrono_literals;

using namespace std;

class xarm_controller : public rclcpp::Node
{
public:
    xarm_controller() : Node("xarm_joint_controller_node")
    {
        callback_exec_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        callback_plan_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        callback_subscriber_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);


        client_ptr_exec_ = this->create_client<xarm_msgs::srv::PlanExec>("xarm_exec_plan", rmw_qos_profile_services_default,
                                                                callback_exec_group_);

        client_ptr_plan_ = this->create_client<xarm_msgs::srv::PlanJoint>("xarm_joint_plan", rmw_qos_profile_services_default,
                                                                callback_exec_group_);

        rclcpp::SubscriptionOptions options;
        options.callback_group = callback_subscriber_group_;

        subscriber_ = this->create_subscription<std_msgs::msg::Float64MultiArray>("xarm_joint_controller", 10, std::bind(&xarm_controller::callback, 
                                                                                    this, std::placeholders::_1));
    }

private:
    rclcpp::CallbackGroup::SharedPtr callback_exec_group_;
    rclcpp::CallbackGroup::SharedPtr callback_plan_group_;
    rclcpp::CallbackGroup::SharedPtr callback_subscriber_group_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr subscriber_;
    rclcpp::Client<xarm_msgs::srv::PlanExec>::SharedPtr client_ptr_exec_;
    rclcpp::Client<xarm_msgs::srv::PlanJoint>::SharedPtr client_ptr_plan_;
    rclcpp::TimerBase::SharedPtr timer_ptr_;

    void callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Sending request");
        auto request_plan = std::make_shared<xarm_msgs::srv::PlanJoint::Request>();
        request_plan-> target = msg->data;
        auto result_future_plan = client_ptr_plan_->async_send_request(request_plan);

        // if (rclcpp::spin_until_future_complete(client_ptr_plan_, result_future_plan) == rclcpp::FutureReturnCode::SUCCESS){
        //     RCLCPP_INFO(this->get_logger(), "Received plan response");
        //     auto request_exec = std::make_shared<xarm_msgs::srv::PlanExec::Request>();
        //     request_exec->wait = true;
        //     auto result_future_exec = client_ptr_exec_->async_send_request(request_exec);
        //     if (rclcpp::spin_until_future_complete(client_ptr_exec_, result_future_exec) == rclcpp::FutureReturnCode::SUCCESS){
        //         RCLCPP_INFO(this->get_logger(), "Received exec response");
        //     }
        // }
        auto result = result_future_plan.get();
        if (result->success == true) {
            RCLCPP_INFO(this->get_logger(), "Received plan response");
            auto request_exec = std::make_shared<xarm_msgs::srv::PlanExec::Request>();
            request_exec->wait = true;
            auto result_future_exec = client_ptr_exec_->async_send_request(request_exec);
            auto result = result_future_exec.get();
            if (result->success == true) {
                RCLCPP_INFO(this->get_logger(), "Received exec response");
            }
        }


    }
    
};  // class DemoNode


int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    std::shared_ptr<xarm_controller> xarm_conttoller_node =
      std::make_shared<xarm_controller>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(xarm_conttoller_node);

    RCLCPP_INFO(xarm_conttoller_node->get_logger(), "Starting client node, shut down with CTRL-C");
    executor.spin();
    RCLCPP_INFO(xarm_conttoller_node->get_logger(), "Keyboard interrupt, shutting down.\n");

    rclcpp::shutdown();
    return 0;
}