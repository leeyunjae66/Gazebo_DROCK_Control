# gazebo_control

ROS 2 control package for the **DROK_CK tracked robot** running in **Gazebo Classic**.

This package provides:

* A bridge from ROS 2 `/cmd_vel` commands to the Gazebo `SimpleTrackedVehiclePlugin` transport topic.
* Joystick-based position control for four flippers through a `JointTrajectoryController`.
* Controller configuration for wheel-joint velocity interfaces and flipper effort-based trajectory control.

> The robot model, URDF, Gazebo world, collision geometry, and tracked-vehicle plugin configuration are maintained in the `drok_gazebo` package. This package focuses on ROS 2 control nodes and controller settings.

## Features

### `/cmd_vel` to Gazebo tracked-vehicle bridge

`cmd_vel_track_control_node` subscribes to ROS 2 `geometry_msgs/msg/Twist` commands and publishes Gazebo Transport `gazebo::msgs::Twist` messages.

```text
/cmd_vel
  ↓
cmd_vel_track_control_node
  ↓
/gazebo/drok_gazebo/cmd_vel_twist
  ↓
SimpleTrackedVehiclePlugin
```

* `linear.x`: forward and reverse velocity
* `angular.z`: yaw-rate command
* The yaw command sign is inverted to match the Gazebo model convention.
* A watchdog sends a stop command when `/cmd_vel` is not received for the configured timeout period.
* Default watchdog timeout: `0.30 s`

### Joystick-based flipper control

`flipper_joy_control_node` receives `/joy` and `/joint_states`, then sends a `trajectory_msgs/msg/JointTrajectory` command to the flipper controller.

```text
FRF_joint  FLF_joint  BRF_joint  BLF_joint
```

* Front flippers: `FRF_joint`, `FLF_joint`
* Rear flippers: `BRF_joint`, `BLF_joint`
* Initial targets are loaded from `/joint_states` to preserve the spawned flipper pose.
* Per-joint sign parameters compensate for opposite URDF joint axes.
* Target angles are limited to the configured range.

## Package Layout

```text
gazebo_control/
├── CMakeLists.txt
├── package.xml
├── config/
│   └── controllers.yaml
└── src/
    ├── cmd_vel_track_control_node.cpp
    └── flipper_joy_control_node.cpp
```

## Build

```bash
cd ~/yunjae/ros2_ws

colcon build --packages-select gazebo_control

source install/setup.bash
```

## Running the System

Run the following commands in separate terminals.

### 1. Launch Gazebo and the robot

```bash
source /opt/ros/humble/setup.bash
source ~/yunjae/ros2_ws/install/setup.bash
source ~/yunjae/Gazebo/DROK_CK/install/setup.bash

ros2 launch drok_gazebo gazebo.launch.py
```

### 2. Start the `/cmd_vel` bridge

```bash
ros2 run gazebo_control cmd_vel_track_control_node
```

Default settings:

```text
ROS input topic       : /cmd_vel
Gazebo output topic   : /gazebo/drok_gazebo/cmd_vel_twist
Watchdog timeout      : 0.30 s
```

### 3. Start the joystick driver

```bash
ros2 run joy joy_node
```

Check joystick messages:

```bash
ros2 topic echo /joy
```

### 4. Start drive teleoperation

```bash
ros2 run teleop_twist_joy teleop_node --ros-args \
  -p require_enable_button:=false \
  -p axis_linear.x:=1 \
  -p axis_angular.yaw:=3 \
  -p scale_linear.x:=1.5 \
  -p scale_angular.yaw:=2.0
```

### 5. Start flipper control

```bash
ros2 run gazebo_control flipper_joy_control_node
```

## Joystick Mapping

| Input                        | Function                                        |
| ---------------------------- | ----------------------------------------------- |
| Left stick forward/back      | Drive forward/reverse                           |
| Left stick left/right        | Turn left/right                                 |
| D-pad up/down (`axes[7]`)    | Raise/lower both front flippers                 |
| D-pad left/right (`axes[6]`) | Raise/lower both rear flippers                  |
| A button (`button[0]`)       | Store current flipper positions as hold targets |
| B button (`button[1]`)       | Move all flippers gradually toward `0 rad`      |

## Controllers

`config/controllers.yaml` defines:

```text
joint_state_broadcaster
track_velocity_controller
flipper_position_controller
```

Flipper controller configuration:

```text
type: joint_trajectory_controller/JointTrajectoryController
command interface: effort
state interfaces: position, velocity
```

Default PID gains:

```yaml
p: 50.0
i: 0.0
d: 3.0
i_clamp: 1.0
```

Check controller status:

```bash
ros2 control list_controllers
```

## Drive-Speed Tuning

The drive speed is limited by both teleoperation output and the Gazebo tracked-vehicle plugin.

```text
teleop_twist_joy scale_linear.x
             ↓
/cmd_vel.linear.x
             ↓
cmd_vel_track_control_node
             ↓
SimpleTrackedVehiclePlugin max_linear_speed
```

Increase teleop output:

```bash
-p scale_linear.x:=3.0
```

In the `drok_gazebo` URDF, ensure the plugin limit is high enough:

```xml
<max_linear_speed>4.0</max_linear_speed>
<max_angular_speed>8.0</max_angular_speed>
```

Verify command output:

```bash
ros2 topic echo /cmd_vel
```

## Troubleshooting

### `/cmd_vel` is published but the robot is slow

1. Check `/cmd_vel.linear.x`.
2. Increase `scale_linear.x`.
3. Check `<max_linear_speed>` in the URDF plugin.
4. Inspect track collision geometry and friction parameters.
5. If the tracks slip, the cause is usually traction or collision geometry rather than a ROS command limit.

### Flippers do not move

```bash
ros2 control list_controllers
ros2 topic echo /joint_states
ros2 topic echo /joy
```

Confirm that:

* `flipper_position_controller` is active.
* All four flipper joints expose effort command interfaces.
* Joint names match between the URDF, YAML, and flipper node.
* `/joint_states` contains all flipper joints.

## License

This package declares the Apache-2.0 license in `package.xml`.
