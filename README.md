# semubot_ros_control

> `ros2_control` hardware interface and velocity controller for the SemuBot 3-motor omnidirectional wheelbase.

![ROS 2](https://img.shields.io/badge/ROS%202-Jazzy-blue)
![License](https://img.shields.io/badge/license-MIT-green)

---

## Overview

`semubot_ros_control` integrates the SemuBot omnidirectional wheelbase into the standard `ros2_control` framework. It provides:

- A custom `ros2_control` hardware interface ([`SemuBotHardwareInterface`](src/semubot_hardware_interface.cpp))
- A custom omnidirectional velocity controller ([`SemuBotVelocityController`](src/semubot_velocity_controller.cpp))
- Transport variants: USB CDC serial and micro-ROS over UART/USB
- Optional ROS-side PID experiments

The package is part of the broader [SemuBot](https://github.com/SemuBot) project — a humanoid robot platform.

---

## Architecture

The active stack is **`ros2ctrl-microros`**: `ros2_control` running on the PC with micro-ROS handling low-level control on the STM32.

```
/cmd_vel  (geometry_msgs/Twist)
    │
    ▼
SemuBotVelocityController
    │
    ▼
ros2_control command interfaces
    │
    ▼
SemuBotHardwareInterface
    │
    ▼
/hardware_interface/velocity_cmd  (std_msgs/Float32MultiArray)
    │  data = [M1, M2, M3]  — PWM duty commands
    ▼
STM32 micro-ROS agent
    │
    ▼
PWM → motors
```

Encoder feedback flows in the opposite direction:

```
STM32 encoders
    │
    ▼
/motor_states  (std_msgs/Float32MultiArray)
    │
    ▼
SemuBotHardwareInterface
    │
    ▼
ros2_control state interfaces
    │
    ▼
SemuBotVelocityController
```


---

## Topics

### Subscribed

| Topic | Type | Description |
|---|---|---|
| `/cmd_vel` | `geometry_msgs/Twist` | High-level velocity command |
| `/motor_states` | `std_msgs/Float32MultiArray` | Per-motor encoder feedback from STM32 |

### Published

| Topic | Type | Description |
|---|---|---|
| `/hardware_interface/velocity_cmd` | `std_msgs/Float32MultiArray` | Low-level motor commands `[M1, M2, M3]` |

### `/cmd_vel` field mapping

| Field | Meaning |
|---|---|
| `linear.x` | Forward / backward |
| `linear.y` | Left / right (strafe) |
| `angular.z` | Rotation |

> **Note:** In the current `ros2ctrl-microros` stack, `/hardware_interface/velocity_cmd` carries **PWM duty cycle values**, not wheel velocity targets.

---

## Dependencies

- ROS 2 Jazzy (or later)
- `ros2_control` / `ros2_controllers`
- `micro_ros_agent` (for micro-ROS transport variant)
- `geometry_msgs`, `std_msgs`
- STM32 firmware: [`SemuBot-Firmware`](https://github.com/SemuBot/semubot-firmware/tree/fw/ros2ctrl-microros)

---

## Building

```bash
cd ~/ros2_ws/src
git clone https://github.com/SemuBot/semubot_ros_control.git
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select semubot_ros_control
source install/setup.bash
```

---

## Usage

### Launch the full stack

```bash
ros2 launch semubot_ros_control semubot_control.launch.py
```

### Send a velocity command manually

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {z: 0.0}}"
```

### Monitor motor commands

```bash
ros2 topic echo /hardware_interface/velocity_cmd
```

---

## Integration Variants

| Variant | Transport | PID location | Status |
|---|---|---|---|
| 1 | Serial CDC | STM32 | Working |
| 2 *(active)* | micro-ROS | STM32 | Working — **recommended** |
| 3 | Serial CDC | ROS 2 | Working |
| 4 | micro-ROS | ROS 2 | Working |


---

## Repository Structure

```
semubot_ros_control/
├── include/semubot_ros_control/
│   ├── semubot_hardware_interface.hpp
│   └── semubot_velocity_controller.hpp
├── src/
│   ├── semubot_hardware_interface.cpp
│   └── semubot_velocity_controller.cpp
├── launch/
│   └── semubot_control.launch.py
├── config/
│   └── joy_config.yaml
│   └── semubot_controllers.yaml
├── CMakeLists.txt
├── package.xml
└── README.md
```

---

## Related Packages

| Package | Description |
|---|---|
| [`semubot_description`](https://github.com/SemuBot/semubot_description) | URDF and mesh assets |
| [`semubot_bringup`](https://github.com/SemuBot/semubot_bringup) | Top-level launch files |
| [`SemuBot-Firmware`](https://github.com/SemuBot/semubot-firmware/tree/fw/ros2ctrl-microros) | STM32 firmware (FreeRTOS, micro-ROS) |
| [`semubot_driver`](https://github.com/SemuBot/semubot_driver) | Low-level serial driver utilities |

---

## **License**

This project is licensed under the Apache 2.0 license - see the [LICENSE](LICENSE) file for more information.