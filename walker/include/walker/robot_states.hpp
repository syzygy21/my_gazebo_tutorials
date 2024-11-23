#ifndef ROBOT_STATES_HPP
#define ROBOT_STATES_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <map>
#include <string>
#include <memory>

// Forward declarations
class RobotControl;
class MovingState;
class RotatingState;

// Abstract State classc
class RobotState {
public:
    virtual ~RobotState() = default;
    virtual void handleAction(RobotControl* robot, 
                            const std::map<std::string, float>& collisionRegion) = 0;
    virtual std::string getStateName() const = 0;
protected:
    void publishVelocity(RobotControl* robot, float linearX, float angularZ);
    const double ROTATION_DURATION = 10.0;
    const float ROTATION_SPEED = 0.3;
    const float FORWARD_SPEED = 0.6;
    const float DISTANCE_THRESHOLD = 0.7;
};

// Moving state
class MovingState : public RobotState {
public:
    void handleAction(RobotControl* robot, 
                     const std::map<std::string, float>& collisionRegion) override;
    std::string getStateName() const override { return "Moving Forward"; }
};

// Rotating state
class RotatingState : public RobotState {
public:
    explicit RotatingState(bool clockwise);
    void handleAction(RobotControl* robot, 
                     const std::map<std::string, float>& collisionRegion) override;
    std::string getStateName() const override { return "Rotating"; }
private:
    bool clockwise_;
    bool isFirstTime_ = true;
    rclcpp::Time rotationStartTime_{0, 0};
};

#endif // ROBOT_STATES_HPP