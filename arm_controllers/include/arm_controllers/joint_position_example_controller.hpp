// Copyright (c) 2025 Tampere University, Autonomous Mobile Machines
// Licensed under the MIT License.
#pragma once
#include <string>
#include <array>
#include <Eigen/Eigen>
#include <controller_interface/controller_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace arm_controllers {

class JointPositionExampleController : public controller_interface::ControllerInterface {
 public:
  [[nodiscard]] controller_interface::InterfaceConfiguration command_interface_configuration()
      const override;
  [[nodiscard]] controller_interface::InterfaceConfiguration state_interface_configuration()
      const override;
  controller_interface::return_type update(const rclcpp::Time& time,
                                           const rclcpp::Duration& period) override;
  CallbackReturn on_init() override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;

 private:
  bool is_gazebo_{false};
  std::string robot_description_;
  
  const int num_joints = 7;  // arm joints only
  std::array<double, 9> initial_q_{0, 0, 0, 0, 0, 0, 0, 0, 0};  // 7 arm + 2 finger
  std::array<double, 9> target_q_{0, 0, 0, 0, 0, 0, 0, 0, 0};   // 7 arm + 2 finger
  
  const double trajectory_period{0.001};
  const std::string k_HW_IF_INITIAL_POSITION = "initial_joint_position";
  bool initialization_flag_{true};
  rclcpp::Time start_time_;

  // Joint command subscriber
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_command_sub_;
  void joint_command_callback(const sensor_msgs::msg::JointState::SharedPtr msg);
};

}  // namespace arm_controllers
