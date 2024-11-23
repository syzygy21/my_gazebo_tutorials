#ifndef ROBOT_CONTROL_HPP
#define ROBOT_CONTROL_HPP

#include "robot_states.hpp"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <memory>

class RobotControl : public rclcpp::Node {
public:
    RobotControl();
    
    void info(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void setState(std::unique_ptr<RobotState> newState);
    void publishVelocity(const geometry_msgs::msg::Twist& msg);
    // Removed const qualifier from getCurrentTime
    rclcpp::Time getCurrentTime() { return this->get_clock()->now(); }
    bool getRotateDirection() const { return rotateClockwise_; }
    void toggleRotateDirection() { rotateClockwise_ = !rotateClockwise_; }
    
protected:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
    std::unique_ptr<RobotState> currentState_;
    bool rotateClockwise_;
};

#endif // ROBOT_CONTROL_HPP
