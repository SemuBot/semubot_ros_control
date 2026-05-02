#include "semubot_hardware_interface.hpp"
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <limits>
#include <vector>

namespace semubot_hardware_interface
{

hardware_interface::CallbackReturn SemubotHardwareInterface::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Get parameters
  wheel_radius_ = std::stod(info_.hardware_parameters["wheel_radius"]);
  base_radius_ = std::stod(info_.hardware_parameters["base_radius"]);
  
  // Initialize wheel angles (30°, 150°, 270°)
  wheel_angles_ = {M_PI / 6.0, 5.0 * M_PI / 6.0, 3.0 * M_PI / 2.0};

  hw_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_efforts_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

  // Verify we have exactly 3 joints
  if (info_.joints.size() != 3)
  {
    RCLCPP_ERROR(
      rclcpp::get_logger("SemubotHardwareInterface"),
      "Expected 3 joints, got %zu", info_.joints.size());
    return hardware_interface::CallbackReturn::ERROR;
  }

  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    // Check for required state interfaces
    if (joint.state_interfaces.size() != 3)
    {
      RCLCPP_ERROR(
        rclcpp::get_logger("SemubotHardwareInterface"),
        "Joint '%s' has %zu state interfaces. Expected 3 (position, velocity, effort).",
        joint.name.c_str(), joint.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.command_interfaces.size() != 1)
    {
      RCLCPP_ERROR(
        rclcpp::get_logger("SemubotHardwareInterface"),
        "Joint '%s' has %zu command interfaces. Expected 1 (velocity).",
        joint.name.c_str(), joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  RCLCPP_INFO(
    rclcpp::get_logger("SemubotHardwareInterface"),
    "Successfully initialized SemubotHardwareInterface with %zu joints", info_.joints.size());

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn SemubotHardwareInterface::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("SemubotHardwareInterface"), "Configuring...");

  for (size_t i = 0; i < hw_positions_.size(); i++)
  {
    hw_positions_[i] = 0.0;
    hw_velocities_[i] = 0.0;
    hw_efforts_[i] = 0.0;
    hw_commands_[i] = 0.0;
  }

  node_ = rclcpp::Node::make_shared("semubot_hardware_interface_node");

  // Subscribe to motor states from STM32
  motor_state_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
    "/motor_states", 50,
    std::bind(&SemubotHardwareInterface::motor_state_callback, this, std::placeholders::_1));

  // Publisher for velocity commands to STM32
  velocity_cmd_pub_ = node_->create_publisher<std_msgs::msg::Float32MultiArray>(
    "/hardware_interface/velocity_cmd", 1);

  RCLCPP_INFO(rclcpp::get_logger("SemubotHardwareInterface"), "Configuration successful");
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
SemubotHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  
  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]));
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]));
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &hw_efforts_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
SemubotHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  
  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    command_interfaces.emplace_back(
      hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_[i]));
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn SemubotHardwareInterface::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("SemubotHardwareInterface"), "Activating...");

  // Set initial commands to zero
  for (size_t i = 0; i < hw_commands_.size(); i++)
  {
    hw_commands_[i] = 0.0;
  }

  RCLCPP_INFO(rclcpp::get_logger("SemubotHardwareInterface"), "Successfully activated");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn SemubotHardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("SemubotHardwareInterface"), "Deactivating...");
  
  // Stop all motors
  for (size_t i = 0; i < hw_commands_.size(); i++)
  {
    hw_commands_[i] = 0.0;
  }
  
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type SemubotHardwareInterface::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
    rclcpp::spin_some(node_);
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type SemubotHardwareInterface::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // Send velocity commands to STM32
  auto msg = std_msgs::msg::Float32MultiArray();
  msg.data.resize(3);
  
  auto get_cmd = [&](const std::string & name) -> float {
    for (size_t i = 0; i < info_.joints.size(); i++) {
      if (info_.joints[i].name == name) {
        return static_cast<float>(hw_commands_[i]);
      }
    }
    return 0.0f;
  };

  msg.data[0] = get_cmd("omni_ball_1_joint");
  msg.data[1] = get_cmd("omni_ball_2_joint");
  msg.data[2] = get_cmd("omni_ball_3_joint");

  velocity_cmd_pub_->publish(msg);

  return hardware_interface::return_type::OK;
}

void SemubotHardwareInterface::motor_state_callback(
  const sensor_msgs::msg::JointState::SharedPtr msg)
{
  last_msg_time_ = node_->get_clock()->now();
  // Update hardware states from STM32 feedback
  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    // Find the joint in the message
    auto it = std::find(msg->name.begin(), msg->name.end(), info_.joints[i].name);
    if (it != msg->name.end())
    {
      size_t index = std::distance(msg->name.begin(), it);
      
      if (index < msg->position.size())
        hw_positions_[i] = msg->position[index];
      
      if (index < msg->velocity.size())
        hw_velocities_[i] = msg->velocity[index];
      
      if (index < msg->effort.size())
        hw_efforts_[i] = msg->effort[index];
            RCLCPP_INFO_THROTTLE(
        rclcpp::get_logger("SemubotHardwareInterface"),
        *node_->get_clock(), 1000,
        "joint[%zu] %s → msg index %zu → vel %.3f",
        i, info_.joints[i].name.c_str(), index, hw_velocities_[i]);
    }
        else
    {
      RCLCPP_WARN_THROTTLE(
        rclcpp::get_logger("SemubotHardwareInterface"),
        *node_->get_clock(), 1000,
        "joint[%zu] %s NOT FOUND in motor_states msg: msg names: %s %s %s",
        i, info_.joints[i].name.c_str(), msg->name[0].c_str(), msg->name[1].c_str(), msg->name[2].c_str());
    }
  }
}

}  // namespace semubot_hardware_interface

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  semubot_hardware_interface::SemubotHardwareInterface, hardware_interface::SystemInterface)
