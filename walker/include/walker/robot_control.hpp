// robot_control.hpp
/**
 * @file robot_control.hpp
 * @brief Header file for the RobotControl class that manages robot navigation
 * @author Navdeep Singh
 * @date 2024
 */

#ifndef ROBOT_CONTROL_HPP
#define ROBOT_CONTROL_HPP

#include "robot_states.hpp"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <memory>

/**
 * @class RobotControl
 * @brief Main control class for robot navigation using laser scan data
 * 
 * This class implements a state machine-based controller that manages
 * robot movement and obstacle avoidance using laser scan data.
 */
class RobotControl : public rclcpp::Node {
public:
    /**
     * @brief Constructor that initializes the ROS 2 node and sets up publishers/subscribers
     */
    RobotControl();

    /**
     * @brief Callback function for processing laser scan data
     * @param msg Shared pointer to the laser scan message
     */
    void info(const sensor_msgs::msg::LaserScan::SharedPtr msg);

    /**
     * @brief Changes the current state of the robot
     * @param newState Unique pointer to the new state object
     */
    void setState(std::unique_ptr<RobotState> newState);

    /**
     * @brief Publishes velocity commands to the robot
     * @param msg Twist message containing linear and angular velocities
     */
    void publishVelocity(const geometry_msgs::msg::Twist& msg);

    /**
     * @brief Gets the current time from the node's clock
     * @return Current ROS Time
     */
    rclcpp::Time getCurrentTime() { return this->get_clock()->now(); }

    /**
     * @brief Gets the current rotation direction
     * @return true if rotating clockwise, false otherwise
     */
    bool getRotateDirection() const { return rotateClockwise_; }

    /**
     * @brief Toggles the rotation direction between clockwise and counter-clockwise
     */
    void toggleRotateDirection() { rotateClockwise_ = !rotateClockwise_; }

protected:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_; ///< Publisher for velocity commands
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_; ///< Subscriber for laser scan data
    std::unique_ptr<RobotState> currentState_; ///< Current state of the robot
    bool rotateClockwise_; ///< Flag indicating rotation direction
};

#endif // ROBOT_CONTROL_HPP