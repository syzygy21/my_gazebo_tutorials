#include "walker/robot_states.hpp"
#include "walker/robot_control.hpp"
#include <thread>
#include <chrono>

using namespace std::chrono_literals;

void RobotState::publishVelocity(RobotControl* robot, float linearX, float angularZ) {
    auto msg = geometry_msgs::msg::Twist();
    msg.linear.x = linearX;
    msg.angular.z = angularZ;
    robot->publishVelocity(msg);
}

void MovingState::handleAction(RobotControl* robot, 
                             const std::map<std::string, float>& collisionRegion) {
    if (collisionRegion.at("front") < DISTANCE_THRESHOLD || 
        collisionRegion.at("fleft") < DISTANCE_THRESHOLD || 
        collisionRegion.at("fright") < DISTANCE_THRESHOLD) {
        robot->setState(std::make_unique<RotatingState>(robot->getRotateDirection()));
    } else {
        publishVelocity(robot, FORWARD_SPEED, 0.0);
    }
}

RotatingState::RotatingState(bool clockwise) 
    : clockwise_(clockwise), 
      isFirstTime_(true),
      rotationStartTime_(0, 0) {
}

void RotatingState::handleAction(RobotControl* robot, 
                               const std::map<std::string, float>& collisionRegion) {
    if (isFirstTime_) {
        rotationStartTime_ = robot->getCurrentTime();
        isFirstTime_ = false;
    }

    auto currentTime = robot->getCurrentTime();
    if ((currentTime - rotationStartTime_).seconds() < ROTATION_DURATION) {
        // Still rotating
        float angularZ = clockwise_ ? -ROTATION_SPEED : ROTATION_SPEED;
        publishVelocity(robot, 0.0, angularZ);
        std::this_thread::sleep_for(50ms);
    }
    else {
        // Rotation completed, decide next state
        if (collisionRegion.at("front") > DISTANCE_THRESHOLD && 
            collisionRegion.at("fleft") > DISTANCE_THRESHOLD && 
            collisionRegion.at("fright") > DISTANCE_THRESHOLD) {
            // Path is clear, move forward
            robot->toggleRotateDirection();  // Prepare for next rotation
            robot->setState(std::make_unique<MovingState>());
        } else {
            // Path still blocked, rotate in opposite direction
            robot->toggleRotateDirection();
            robot->setState(std::make_unique<RotatingState>(robot->getRotateDirection()));
        }
    }
}