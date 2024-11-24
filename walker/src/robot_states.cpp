/**
 * @file <filename>
 * @brief <brief description>
 * @author Navdeep Singh
 * @date 2024
 *
 * Copyright 2024 Navdeep Singh
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
/**
 * @file robot_states.cpp
 * @brief Implementation of the robot state classes
 * @author Navdeep Singh
 * @date 2024
 */

#include "walker/robot_states.hpp"

#include <chrono>
#include <thread>

#include "walker/robot_control.hpp"

using namespace std::chrono_literals;

void RobotState::publishVelocity(RobotControl* robot, float linearX,
                                 float angularZ) {
  auto msg = geometry_msgs::msg::Twist();
  msg.linear.x = linearX;
  msg.angular.z = angularZ;
  robot->publishVelocity(msg);
}

void MovingState::handleAction(
    RobotControl* robot, const std::map<std::string, float>& collisionRegion) {
  // Check if obstacles are detected in critical regions
  if (collisionRegion.at("front") < DISTANCE_THRESHOLD ||
      collisionRegion.at("fleft") < DISTANCE_THRESHOLD ||
      collisionRegion.at("fright") < DISTANCE_THRESHOLD) {
    // Switch to rotating state if obstacles detected
    robot->setState(
        std::make_unique<RotatingState>(robot->getRotateDirection()));
  } else {
    // Continue moving forward if path is clear
    publishVelocity(robot, FORWARD_SPEED, 0.0);
  }
}

RotatingState::RotatingState(bool clockwise)
    : clockwise_(clockwise), isFirstTime_(true), rotationStartTime_(0, 0) {}

void RotatingState::handleAction(
    RobotControl* robot, const std::map<std::string, float>& collisionRegion) {
  // Initialize rotation start time on first execution
  if (isFirstTime_) {
    rotationStartTime_ = robot->getCurrentTime();
    isFirstTime_ = false;
  }

  auto currentTime = robot->getCurrentTime();
  if ((currentTime - rotationStartTime_).seconds() < ROTATION_DURATION) {
    // Continue rotation
    float angularZ = clockwise_ ? -ROTATION_SPEED : ROTATION_SPEED;
    publishVelocity(robot, 0.0, angularZ);
    std::this_thread::sleep_for(50ms);
  } else {
    // Check path after rotation
    if (collisionRegion.at("front") > DISTANCE_THRESHOLD &&
        collisionRegion.at("fleft") > DISTANCE_THRESHOLD &&
        collisionRegion.at("fright") > DISTANCE_THRESHOLD) {
      // Switch to moving state if path is clear
      robot->toggleRotateDirection();
      robot->setState(std::make_unique<MovingState>());
    } else {
      // Continue rotating in opposite direction if path is blocked
      robot->toggleRotateDirection();
      robot->setState(
          std::make_unique<RotatingState>(robot->getRotateDirection()));
    }
  }
}
