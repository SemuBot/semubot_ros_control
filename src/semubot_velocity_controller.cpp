#include "semubot_velocity_controller.hpp"
#include <algorithm>
#include <cmath>
#include <string>

namespace semubot_velocity_controller
{

SemubotVelocityController::SemubotVelocityController()
: controller_interface::ControllerInterface()
{
}

controller_interface::CallbackReturn SemubotVelocityController::on_init()
{
  try
  {
    auto_declare<double>("wheel_radius", 0.05);
    auto_declare<double>("base_radius", 0.15);
    auto_declare<std::vector<std::string>>("wheel_names", std::vector<std::string>());
    auto_declare<double>("max_linear_velocity", 1.0);
    auto_declare<double>("max_angular_velocity", 2.0);
    auto_declare<double>("kp", 0.1);
    auto_declare<double>("ki", 0.0);
    auto_declare<double>("kd", 0.0);
  }
  catch (const std::exception & e)
  {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Exception thrown during init: %s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
SemubotVelocityController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (const auto & wheel_name : wheel_names_)
  {
    config.names.push_back(wheel_name + "/velocity");
  }

  return config;
}

controller_interface::InterfaceConfiguration
SemubotVelocityController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (const auto & wheel_name : wheel_names_)
  {
    config.names.push_back(wheel_name + "/position");
    config.names.push_back(wheel_name + "/velocity");
  }

  return config;
}

controller_interface::CallbackReturn SemubotVelocityController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  wheel_radius_ = get_node()->get_parameter("wheel_radius").as_double();
  base_radius_ = get_node()->get_parameter("base_radius").as_double();
  wheel_names_ = get_node()->get_parameter("wheel_names").as_string_array();
  max_linear_velocity_ = get_node()->get_parameter("max_linear_velocity").as_double();
  max_angular_velocity_ = get_node()->get_parameter("max_angular_velocity").as_double();
  kp_ = get_node()->get_parameter("kp").as_double();
  ki_ = get_node()->get_parameter("ki").as_double();
  kd_ = get_node()->get_parameter("kd").as_double();

  if (wheel_names_.size() != 3)
  {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Expected 3 wheels, got %zu", wheel_names_.size());
    return controller_interface::CallbackReturn::ERROR;
  }

  // Only used for odometry right now.
  // Command control below uses manual calibrated mixing.
  wheel_angles_ = {
    M_PI / 2.0,
    4.0 * M_PI / 3.0,
    11.0 * M_PI / 6.0
  };

  cmd_vel_sub_ = get_node()->create_subscription<geometry_msgs::msg::Twist>(
    "~/cmd_vel",
    rclcpp::SystemDefaultsQoS(),
    [this](const geometry_msgs::msg::Twist::SharedPtr msg)
    {
      latest_cmd_vel_ = *msg;
      cmd_vel_timeout_ = std::chrono::steady_clock::now();
    });

  odom_pub_ = get_node()->create_publisher<nav_msgs::msg::Odometry>(
    "~/odom",
    rclcpp::SystemDefaultsQoS());

  RCLCPP_INFO(get_node()->get_logger(), "Semubot velocity controller configured");

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn SemubotVelocityController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  latest_cmd_vel_ = geometry_msgs::msg::Twist();
  cmd_vel_timeout_ = std::chrono::steady_clock::now();

  for (auto & s : pid_states_)
  {
    s.integral = 0.0;
    s.prev_error = 0.0;
  }

  for (size_t i = 0; i < command_interfaces_.size(); i++)
  {
    RCLCPP_INFO(
      get_node()->get_logger(),
      "command_interface[%zu] = %s",
      i,
      command_interfaces_[i].get_name().c_str());
  }

  RCLCPP_INFO(get_node()->get_logger(), "Semubot velocity controller activated");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn SemubotVelocityController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  for (size_t i = 0; i < command_interfaces_.size(); i++)
  {
    bool success = command_interfaces_[i].set_value(0.0);

    if (!success)
    {
      RCLCPP_WARN(
        get_node()->get_logger(),
        "Failed to zero command interface %zu during deactivate",
        i);
    }
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type SemubotVelocityController::update(
  const rclcpp::Time & time,
  const rclcpp::Duration & /*period*/)
{
  auto now = std::chrono::steady_clock::now();
  auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
    now - cmd_vel_timeout_).count();

  geometry_msgs::msg::Twist cmd_vel;

  if (elapsed > 500)
  {
    cmd_vel.linear.x = 0.0;
    cmd_vel.linear.y = 0.0;
    cmd_vel.angular.z = 0.0;
  }
  else
  {
    cmd_vel = latest_cmd_vel_;

    cmd_vel.linear.x = std::clamp(
      cmd_vel.linear.x,
      -max_linear_velocity_,
      max_linear_velocity_);

    cmd_vel.linear.y = std::clamp(
      cmd_vel.linear.y,
      -max_linear_velocity_,
      max_linear_velocity_);

    cmd_vel.angular.z = std::clamp(
      cmd_vel.angular.z,
      -max_angular_velocity_,
      max_angular_velocity_);
  }

  const bool stopped =
    std::abs(cmd_vel.linear.x) < 1e-6 &&
    std::abs(cmd_vel.linear.y) < 1e-6 &&
    std::abs(cmd_vel.angular.z) < 1e-6;

  std::array<double, 3> output = {0.0, 0.0, 0.0};

  if (!stopped)
  {
    if (cmd_vel.linear.x >= 0.0) {
      output[0] =  0.533 * cmd_vel.linear.x;  // M1
      output[1] =  0.533 * cmd_vel.linear.x;  // M2
      output[2] = -1.133 * cmd_vel.linear.x;  // M3
    } else {
      // Backward = [-0.15, -0.15, 0.225] at x = -0.3
      output[0] =  0.500 * cmd_vel.linear.x;  // M1
      output[1] =  0.500 * cmd_vel.linear.x;  // M2
      output[2] = -0.750 * cmd_vel.linear.x;  // M3
    }

    // Right / left
    if (cmd_vel.linear.y >= 0.0) {
      // Right
      // [M1, M2, M3] = [0.20, -0.20, -0.10]
      output[0] += -0.667 * cmd_vel.linear.y;  // M1
      output[1] +=  0.667 * cmd_vel.linear.y;  // M2
      output[2] +=  0.167 * cmd_vel.linear.y;  // M3
    } else {
      // Left
      // [M1, M2, M3] = [-0.20, 0.20, 0.05]
      output[0] += -0.667 * cmd_vel.linear.y;  // M1
      output[1] +=  0.667 * cmd_vel.linear.y;  // M2
      output[2] +=  0.333 * cmd_vel.linear.y;  // M3
    }

    // Rotation.
    output[0] += 0.50 * cmd_vel.angular.z;
    output[1] += 0.50 * cmd_vel.angular.z;
    output[2] += 0.50 * cmd_vel.angular.z;

    for (size_t i = 0; i < 3; i++)
    {
      output[i] = std::clamp(output[i], -0.40, 0.40);
    }
  }
  if (elapsed > 500)
  {
    for (auto & s : pid_states_)
    {
      s.integral = 0.0;
      s.prev_error = 0.0;
    }
  }

  for (size_t i = 0; i < command_interfaces_.size(); i++)
  {
    bool success = command_interfaces_[i].set_value(output[i]);

    if (!success)
    {
      RCLCPP_WARN_THROTTLE(
        get_node()->get_logger(),
        *get_node()->get_clock(),
        1000,
        "Failed to set command interface %zu",
        i);
    }
  }

  publish_odometry(time);
  return controller_interface::return_type::OK;
}

void SemubotVelocityController::publish_odometry(const rclcpp::Time & time)
{
  std::vector<double> actual_velocities(3);
  for (size_t i = 0; i < 3; i++)
  {
    auto opt_value = state_interfaces_[i * 2 + 1].get_optional();
    actual_velocities[i] = opt_value.has_value() ? opt_value.value() : 0.0;
  }

  Eigen::Matrix3d K;
  for (size_t i = 0; i < 3; i++)
  {
    K(i, 0) = -std::sin(wheel_angles_[i]);
    K(i, 1) =  std::cos(wheel_angles_[i]);
    K(i, 2) =  base_radius_;
  }

  Eigen::Vector3d wheel_vel(
    actual_velocities[0],
    actual_velocities[1],
    actual_velocities[2]);

  Eigen::Vector3d robot_vel = K.colPivHouseholderQr().solve(wheel_vel);

  static double x = 0.0;
  static double y = 0.0;
  static double theta = 0.0;
  static rclcpp::Time last_time = time;

  double dt = (time - last_time).seconds();

  if (dt > 0.0 && dt < 1.0)
  {
    x += robot_vel(0) * dt;
    y += robot_vel(1) * dt;
    theta += robot_vel(2) * dt;
  }

  last_time = time;

  auto odom_msg = nav_msgs::msg::Odometry();
  odom_msg.header.stamp = time;
  odom_msg.header.frame_id = "odom";
  odom_msg.child_frame_id = "base_link";

  odom_msg.pose.pose.position.x = x;
  odom_msg.pose.pose.position.y = y;
  odom_msg.pose.pose.position.z = 0.0;

  tf2::Quaternion q;
  q.setRPY(0, 0, theta);

  odom_msg.pose.pose.orientation.x = q.x();
  odom_msg.pose.pose.orientation.y = q.y();
  odom_msg.pose.pose.orientation.z = q.z();
  odom_msg.pose.pose.orientation.w = q.w();

  odom_msg.twist.twist.linear.x = robot_vel(0);
  odom_msg.twist.twist.linear.y = robot_vel(1);
  odom_msg.twist.twist.angular.z = robot_vel(2);

  odom_pub_->publish(odom_msg);
}

}  // namespace semubot_velocity_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  semubot_velocity_controller::SemubotVelocityController,
  controller_interface::ControllerInterface)