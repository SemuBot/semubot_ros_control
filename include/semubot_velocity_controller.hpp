#ifndef semubot_velocity_controller__semubot_velocity_controller_HPP_
#define semubot_velocity_controller__semubot_velocity_controller_HPP_

#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include <array>

#include "controller_interface/controller_interface.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "hardware_interface/handle.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "realtime_tools/realtime_publisher.hpp"
#include <eigen3/Eigen/Dense>
#include <tf2/LinearMath/Quaternion.h>

namespace semubot_velocity_controller
{

class SemubotVelocityController : public controller_interface::ControllerInterface
{
public:
  SemubotVelocityController();

  controller_interface::CallbackReturn on_init() override;

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:

  struct PidState {
    double integral   = 0.0;
    double prev_error = 0.0;
  };

  //static constexpr double KP             = 0.5;
  //static constexpr double KI             = 0.2;
  //static constexpr double KD             = 0.01;
  double kp_;
  double ki_;
  double kd_;
  static constexpr double INTEGRAL_LIMIT = 5.0;
  static constexpr double MAX_WHEEL_RADS = 20.0;  

  std::array<PidState, 3> pid_states_;
  // Parameters
  double wheel_radius_;
  double base_radius_;
  std::vector<std::string> wheel_names_;
  std::vector<double> wheel_angles_;
  double max_linear_velocity_;
  double max_angular_velocity_;

  // Subscribers and publishers
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;

  // Command storage
  geometry_msgs::msg::Twist latest_cmd_vel_;
  std::chrono::steady_clock::time_point cmd_vel_timeout_;

  void publish_odometry(const rclcpp::Time & time);
};

}  // namespace semubot_velocity_controller

#endif  // semubot_velocity_controller__semubot_velocity_controller_HPP_
