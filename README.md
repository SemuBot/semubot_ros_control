# semubot_ros_control

ROS 2 `ros2_control` package for the Semubot 3-motor omni/ball wheelbase.

This package contains:

- a custom `ros2_control` hardware interface
- a custom Semubot velocity controller
- support for ROS-side PID experiments
- micro-ROS and serial transport variants

## Command convention

Main robot command topic:

    /cmd_vel

Message type:

    geometry_msgs/msg/Twist

Meaning:

    linear.x  = forward/backward
    linear.y  = left/right
    angular.z = rotation

Low-level motor command topic used by the ros2_control hardware interface:

    /hardware_interface/velocity_cmd

Message type:

    std_msgs/msg/Float32MultiArray

Current meaning:

    data = [M1, M2, M3]

For the current `ros2ctrl-microros` stack, this topic carries PWM duty commands, not wheel velocity targets.

## Current architecture: ros2ctrl-microros

The current stack is:

    /cmd_vel
    -> semubot_velocity_controller
    -> ros2_control command interfaces
    -> semubot_hardware_interface
    -> /hardware_interface/velocity_cmd
    -> STM32 micro-ROS
    -> PWM duty

Encoder feedback returns as:

    STM32 encoders
    -> /motor_states
    -> semubot_hardware_interface
    -> ros2_control state interfaces
    -> semubot_velocity_controller
