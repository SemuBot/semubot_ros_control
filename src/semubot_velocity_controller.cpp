#include "semubot_velocity_controller.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <string>
#include <vector>

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
    auto_declare<double>("kp", 0.005);
    auto_declare<double>("ki", 0.0);
    auto_declare<double>("kd", 0.0);
    auto_declare<double>("max_duty", 0.80);

    // Integral safety clamp.
    auto_declare<double>("integral_limit", 5.0);

    // If no cmd_vel arrives for this long, stop.
    auto_declare<double>("cmd_timeout_sec", 0.5);
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

  max_duty_ = get_node()->get_parameter("max_duty").as_double();
  integral_limit_ = get_node()->get_parameter("integral_limit").as_double();
  cmd_timeout_sec_ = get_node()->get_parameter("cmd_timeout_sec").as_double();

  if (wheel_names_.size() != 3)
  {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Expected 3 wheels, got %zu", wheel_names_.size());
    return controller_interface::CallbackReturn::ERROR;
  }

  if (wheel_radius_ <= 0.0)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "wheel_radius must be > 0");
    return controller_interface::CallbackReturn::ERROR;
  }

  if (max_duty_ <= 0.0)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "max_duty must be > 0");
    return controller_interface::CallbackReturn::ERROR;
  }

  /*
    Wheel angle convention.

      wheel_angles_[0] = M1 = omni_ball_1_joint
      wheel_angles_[1] = M2 = omni_ball_2_joint
      wheel_angles_[2] = M3 = omni_ball_3_joint

    Kinematic equation used:

      wheel_velocity[i] =
        (-sin(theta_i) * vx + cos(theta_i) * vy + base_radius * wz) / wheel_radius

    Units:
      vx, vy: m/s
      wz: rad/s
      wheel_radius: m
      base_radius: m
      wheel_velocity: rad/s
  */
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
  RCLCPP_INFO(
    get_node()->get_logger(),
    "PID gains: kp=%.6f ki=%.6f kd=%.6f max_duty=%.3f",
    kp_, ki_, kd_, max_duty_);

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
    command_interfaces_[i].set_value(0.0);

    RCLCPP_INFO(
      get_node()->get_logger(),
      "command_interface[%zu] = %s",
      i,
      command_interfaces_[i].get_name().c_str());
  }

  for (size_t i = 0; i < state_interfaces_.size(); i++)
  {
    RCLCPP_INFO(
      get_node()->get_logger(),
      "state_interface[%zu] = %s",
      i,
      state_interfaces_[i].get_name().c_str());
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

  for (auto & s : pid_states_)
  {
    s.integral = 0.0;
    s.prev_error = 0.0;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type SemubotVelocityController::update(
  const rclcpp::Time & time,
  const rclcpp::Duration & period)
{
  auto now = std::chrono::steady_clock::now();

  const double elapsed_sec =
    std::chrono::duration<double>(now - cmd_vel_timeout_).count();

  geometry_msgs::msg::Twist cmd_vel;

  if (elapsed_sec > cmd_timeout_sec_)
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

  const double dt = std::max(period.seconds(), 1e-3);

  std::array<double, 3> target_wheel_vel = {0.0, 0.0, 0.0};
  std::array<double, 3> actual_wheel_vel = {0.0, 0.0, 0.0};
  std::array<double, 3> feedforward_duty = {0.0, 0.0, 0.0};
  std::array<double, 3> output_duty = {0.0, 0.0, 0.0};

  for (size_t i = 0; i < 3; i++)
  {
    const size_t velocity_index = i * 2 + 1;

    if (velocity_index < state_interfaces_.size())
    {
      auto opt_value = state_interfaces_[velocity_index].get_optional();
      actual_wheel_vel[i] = opt_value.has_value() ? opt_value.value() : 0.0;
    }
    else
    {
      actual_wheel_vel[i] = 0.0;
    }
  }

  if (!stopped)
  {
    // Forward / backward feedforward
    if (cmd_vel.linear.x >= 0.0) {
      const double vx = cmd_vel.linear.x;

      const std::array<double, 3> low_coeff = {
        0.533,
        0.533,
      -1.333
      };

      // Tuned at vx = 0.40:
      // [0.28, 0.06, -0.52]
      const std::array<double, 3> mid_coeff = {
        0.700,
        0.150,
      -1.300
      };

      // Tuned at vx = 0.60:
      // [0.50, 0.00, -0.67]
      const std::array<double, 3> high_coeff = {
        0.733,
        0.067,
      -1.033
      };

      if (vx <= 0.40) {
        // vx <= 0.20 uses low-speed tuning
        // vx >= 0.40 uses mid-speed tuning
        const double t = std::clamp((vx - 0.20) / 0.20, 0.0, 1.0);

        for (size_t i = 0; i < 3; i++) {
          const double coeff =
            low_coeff[i] + (mid_coeff[i] - low_coeff[i]) * t;

          feedforward_duty[i] = coeff * vx;
        }
      } else {
        // vx >= 0.40 blends from mid-speed to high-speed tuning
        // vx >= 0.60 uses high-speed tuning
        const double t = std::clamp((vx - 0.40) / 0.20, 0.0, 1.0);

        for (size_t i = 0; i < 3; i++) {
          const double coeff =
            mid_coeff[i] + (high_coeff[i] - mid_coeff[i]) * t;

          feedforward_duty[i] = coeff * vx;
        }
      }
    } else {
      feedforward_duty[0] =  0.500 * cmd_vel.linear.x;
      feedforward_duty[1] =  0.500 * cmd_vel.linear.x;
      feedforward_duty[2] = -0.750 * cmd_vel.linear.x;
    }
    // Left / right feedforward
    if (cmd_vel.linear.y >= 0.0) {
      // left
      feedforward_duty[0] += -0.667 * cmd_vel.linear.y;
      feedforward_duty[1] +=  0.667 * cmd_vel.linear.y;
      feedforward_duty[2] +=  0.167 * cmd_vel.linear.y;
    } else {
      // right
      feedforward_duty[0] += -0.667 * cmd_vel.linear.y;
      feedforward_duty[1] +=  0.667 * cmd_vel.linear.y;
      feedforward_duty[2] +=  0.333 * cmd_vel.linear.y;
    }
    // Rotation feedforward
    feedforward_duty[0] += 0.50 * cmd_vel.angular.z;
    feedforward_duty[1] += 0.50 * cmd_vel.angular.z;
    feedforward_duty[2] += 0.50 * cmd_vel.angular.z;

    for (size_t i = 0; i < 3; i++) {
      feedforward_duty[i] = std::clamp(feedforward_duty[i], -max_duty_, max_duty_);
    }
    for (size_t i = 0; i < 3; i++)
    {
      target_wheel_vel[i] =
        (
          -std::sin(wheel_angles_[i]) * cmd_vel.linear.x +
           std::cos(wheel_angles_[i]) * cmd_vel.linear.y +
           base_radius_ * cmd_vel.angular.z
        ) / wheel_radius_;
    }

    /*
      PID:
        error = target rad/s - measured rad/s
        output = duty command
    */
    for (size_t i = 0; i < 3; i++)
    {
      const double error = target_wheel_vel[i] - actual_wheel_vel[i];

      pid_states_[i].integral += error * dt;
      pid_states_[i].integral = std::clamp(
        pid_states_[i].integral,
        -integral_limit_,
        integral_limit_);

      const double derivative =
        (error - pid_states_[i].prev_error) / dt;

      const double correction =
        kp_ * error +
        ki_ * pid_states_[i].integral +
        kd_ * derivative;

      output_duty[i] = feedforward_duty[i] + correction;
      output_duty[i] = std::clamp(output_duty[i], -max_duty_, max_duty_);
      pid_states_[i].prev_error = error;

    }
  }
  else
  {

    for (auto & s : pid_states_)
    {
      s.integral = 0.0;
      s.prev_error = 0.0;
    }

    output_duty = {0.0, 0.0, 0.0};
  }

  if (elapsed_sec > cmd_timeout_sec_)
  {
    for (auto & s : pid_states_)
    {
      s.integral = 0.0;
      s.prev_error = 0.0;
    }

    output_duty = {0.0, 0.0, 0.0};
  }

  for (size_t i = 0; i < command_interfaces_.size() && i < 3; i++)
  {
    bool success = command_interfaces_[i].set_value(output_duty[i]);

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

  RCLCPP_DEBUG_THROTTLE(
    get_node()->get_logger(),
    *get_node()->get_clock(),
    500,
    "cmd vx=%.3f vy=%.3f wz=%.3f | target=[%.2f %.2f %.2f] actual=[%.2f %.2f %.2f] duty=[%.3f %.3f %.3f]",
    cmd_vel.linear.x,
    cmd_vel.linear.y,
    cmd_vel.angular.z,
    target_wheel_vel[0],
    target_wheel_vel[1],
    target_wheel_vel[2],
    actual_wheel_vel[0],
    actual_wheel_vel[1],
    actual_wheel_vel[2],
    output_duty[0],
    output_duty[1],
    output_duty[2]);

  publish_odometry(time);
  return controller_interface::return_type::OK;
}

void SemubotVelocityController::publish_odometry(const rclcpp::Time & time)
{
  std::vector<double> actual_velocities(3, 0.0);

  for (size_t i = 0; i < 3; i++)
  {
    const size_t velocity_index = i * 2 + 1;

    if (velocity_index < state_interfaces_.size())
    {
      auto opt_value = state_interfaces_[velocity_index].get_optional();
      actual_velocities[i] = opt_value.has_value() ? opt_value.value() : 0.0;
    }
  }

  Eigen::Matrix3d K;

  for (size_t i = 0; i < 3; i++)
  {
    K(i, 0) = -std::sin(wheel_angles_[i]) / wheel_radius_;
    K(i, 1) =  std::cos(wheel_angles_[i]) / wheel_radius_;
    K(i, 2) =  base_radius_ / wheel_radius_;
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
    const double vx = robot_vel(0);
    const double vy = robot_vel(1);
    const double wz = robot_vel(2);

    const double cos_theta = std::cos(theta);
    const double sin_theta = std::sin(theta);

    x += (vx * cos_theta - vy * sin_theta) * dt;
    y += (vx * sin_theta + vy * cos_theta) * dt;
    theta += wz * dt;
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
  q.setRPY(0.0, 0.0, theta);

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