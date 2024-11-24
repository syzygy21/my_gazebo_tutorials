// robot_states.hpp
/**
 * @file robot_states.hpp
 * @brief Header file defining the state machine classes for robot control
 * @author Navdeep Singh
 * @date 2024
 */

#ifndef ROBOT_STATES_HPP
#define ROBOT_STATES_HPP

#include <geometry_msgs/msg/twist.hpp>
#include <map>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>

// Forward declarations
class RobotControl;
class MovingState;
class RotatingState;

/**
 * @class RobotState
 * @brief Abstract base class for robot states
 *
 * Defines the interface for different robot states and provides
 * common functionality for velocity control.
 */
class RobotState {
 public:
  virtual ~RobotState() = default;

  /**
   * @brief Pure virtual function to handle state-specific actions
   * @param robot Pointer to the robot control object
   * @param collisionRegion Map containing distances to obstacles in different
   * regions
   */
  virtual void handleAction(
      RobotControl* robot,
      const std::map<std::string, float>& collisionRegion) = 0;

  /**
   * @brief Pure virtual function to get the name of the current state
   * @return String containing the state name
   */
  virtual std::string getStateName() const = 0;

 protected:
  /**
   * @brief Helper function to publish velocity commands
   * @param robot Pointer to the robot control object
   * @param linearX Linear velocity in x direction
   * @param angularZ Angular velocity around z axis
   */
  void publishVelocity(RobotControl* robot, float linearX, float angularZ);

  const double ROTATION_DURATION = 10.0;  ///< Duration of rotation in seconds
  const float ROTATION_SPEED = 0.3;       ///< Angular velocity for rotation
  const float FORWARD_SPEED = 0.6;  ///< Linear velocity for forward movement
  const float DISTANCE_THRESHOLD = 0.7;  ///< Minimum safe distance to obstacles
};

/**
 * @class MovingState
 * @brief State class for forward movement
 */
class MovingState : public RobotState {
 public:
  void handleAction(
      RobotControl* robot,
      const std::map<std::string, float>& collisionRegion) override;
  std::string getStateName() const override { return "Moving Forward"; }
};

/**
 * @class RotatingState
 * @brief State class for rotation movement
 */
class RotatingState : public RobotState {
 public:
  /**
   * @brief Constructor that sets rotation direction
   * @param clockwise Direction of rotation
   */
  explicit RotatingState(bool clockwise);

  void handleAction(
      RobotControl* robot,
      const std::map<std::string, float>& collisionRegion) override;
  std::string getStateName() const override { return "Rotating"; }

 private:
  bool clockwise_;                        ///< Direction of rotation
  bool isFirstTime_ = true;               ///< Flag for first execution
  rclcpp::Time rotationStartTime_{0, 0};  ///< Time when rotation started
};

#endif  // ROBOT_STATES_HPP
