/**
 * @file walker.hpp
 * @author Rishie Raj
 * @brief Header file for the walker node
 * @version 0.1
 * @date 2024-11-23
 *
 * @copyright Copyright (c) 2024
 *
 */
#pragma once

#include <geometry_msgs/msg/twist.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

// Forward declaration of Walker class
class Walker;

/**
 * @brief Class to define the state of the robot
 *
 */
class RobotState {
 public:
  virtual void execute(Walker &context, float min_distance) = 0;
  virtual ~RobotState() = default;
};

/**
 * @brief Class to move the robot forward
 *
 */
class MovingForward : public RobotState {
  void execute(Walker &context, float min_distance) override;
};

/**
 * @brief Class to rotate the robot clockwise
 *
 */
class RotatingClockwise : public RobotState {
  void execute(Walker &context, float min_distance) override;
};

/**
 * @brief Class to rotate the robot counter clockwise
 *
 */
class RotatingCounterClockwise : public RobotState {
  void execute(Walker &context, float min_distance) override;
};

/**
 * @brief Class to control the robot
 *
 */
class Walker : public rclcpp::Node {
 public:
  Walker();
  // callback function for laser scan data
  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  // function to change the state of the robot
  void changeState(std::shared_ptr<RobotState> new_state);
  // create a publisher for velocity commands
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  // create a subscriber for laser scan data
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
  // attribute to store the current state of the robot
  std::string current_state_;
  // attribute to store the previous state of the robot
  std::string previous_state_;
  // robot state function attribute
  std::shared_ptr<RobotState> state_;
  // attribute to store the previous rotation direction
  bool rotating_clockwise_;
};