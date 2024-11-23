#include "walker/robot_control.hpp"
#include "walker/robot_states.hpp"
#include <algorithm>

RobotControl::RobotControl() 
    : Node("reading_laser"), 
      rotateClockwise_(true) {
    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 1);
    
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10))
        .best_effort()
        .reliable();

    subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan",
        qos,
        std::bind(&RobotControl::info, this, std::placeholders::_1)
    );
    
    currentState_ = std::make_unique<MovingState>();
}

void RobotControl::info(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    std::map<std::string, float> collisionRegion;
    
    auto rightRange = std::vector<float>(msg->ranges.begin() + 55, msg->ranges.begin() + 90);
    collisionRegion["right"] = *std::min_element(rightRange.begin(), rightRange.end());
    
    auto frightRange = std::vector<float>(msg->ranges.begin() + 19, msg->ranges.begin() + 27);
    collisionRegion["fright"] = *std::min_element(frightRange.begin(), frightRange.end());
    
    auto frontRange1 = std::vector<float>(msg->ranges.begin(), msg->ranges.begin() + 18);
    auto frontRange2 = std::vector<float>(msg->ranges.begin() + 342, msg->ranges.end());
    collisionRegion["front"] = std::min(
        *std::min_element(frontRange1.begin(), frontRange1.end()),
        *std::min_element(frontRange2.begin(), frontRange2.end())
    );
    
    auto fleftRange = std::vector<float>(msg->ranges.begin() + 335, msg->ranges.begin() + 341);
    collisionRegion["fleft"] = *std::min_element(fleftRange.begin(), fleftRange.end());
    
    auto leftRange = std::vector<float>(msg->ranges.begin() + 270, msg->ranges.begin() + 305);
    collisionRegion["left"] = *std::min_element(leftRange.begin(), leftRange.end());
    
    currentState_->handleAction(this, collisionRegion);
}

void RobotControl::setState(std::unique_ptr<RobotState> newState) {
    currentState_ = std::move(newState);
    RCLCPP_INFO(this->get_logger(), "Switching to state: %s", currentState_->getStateName().c_str());
}

void RobotControl::publishVelocity(const geometry_msgs::msg::Twist& msg) {
    publisher_->publish(msg);
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RobotControl>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}