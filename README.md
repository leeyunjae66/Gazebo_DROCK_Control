# gazebo_control

ROS 2 control package for the **DROK_CK tracked robot** in **Gazebo Classic**.

This package provides:

* A `/cmd_vel` bridge to the Gazebo tracked-vehicle plugin.
* Joystick-based control for four flippers.
* Controller configuration for joint state broadcasting and trajectory-controlled flipper motion.

> This package focuses on ROS 2 control nodes and controller settings. The robot model, URDF, Gazebo world, and tracked-vehicle plugin configuration are maintained in the separate `drok_gazebo` package.

## Features

### `/cmd_vel` to Gazebo tracked-vehicle bridge

`cmd_vel_track_control_node` converts ROS 2 `geometry_msgs/msg/Twist` commands into Gazebo Transport `gazebo::msgs::Twist` messages.

```text
/cmd_vel
  ↓
cmd_vel_track_control_node
  ↓
/gazebo/drok_gazebo/cmd_vel_twist
  ↓
SimpleTrackedVehiclePlugin
```

* `linear.x` controls forward and reverse motion.
* `angular.z` controls yaw rate.
* The yaw sign is inverted to match the Gazebo model convention.
* A watchdog publishes a stop command when `/cmd_vel` is not received within the timeout interval.
* Default watchdog timeout: `0.30 s`

### Flipper control via joystick

`flipper_joy_control_node` listens to `/joy` and `/joint_states`, then sends a `trajectory_msgs/msg/JointTrajectory` command to the flipper controller.

* Front flippers: `FRF_joint`, `FLF_joint`
* Rear flippers: `BRF_joint`, `BLF_joint`
* Initial targets are read from `/joint_states` so flippers keep their spawned pose.
* Per-joint sign parameters compensate for inverted URDF joint axes.
* Target angles are clamped to the configured limits.

## Package Layout

```text
gazebo_control/
├── CMakeLists.txt
├── package.xml
├── config/
│   └── controllers.yaml
├── launch/
│   └── control_nodes.launch.py
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

## Launching the Package

This package provides `launch/control_nodes.launch.py` to start both control nodes together.

```bash
source /opt/ros/humble/setup.bash
source ~/yunjae/ros2_ws/install/setup.bash
ros2 launch gazebo_control control_nodes.launch.py
```

### Available launch parameters

* `linear_scale` (default: `3.0`)
* `angular_scale` (default: `8.0`)
* `watchdog_timeout_sec` (default: `0.30`)
* `angle_rate` (default: `1.0`)
* `min_angle` (default: `-1.45`)
* `max_angle` (default: `1.45`)
* `trajectory_duration` (default: `0.25`)
* `command_period` (default: `0.20`)

## Usage

### Start Gazebo and the robot

```bash
source /opt/ros/humble/setup.bash
source ~/yunjae/ros2_ws/install/setup.bash
source ~/yunjae/Gazebo/DROK_CK/install/setup.bash
ros2 launch drok_gazebo gazebo.launch.py
```

### Start the command bridge

```bash
ros2 run gazebo_control cmd_vel_track_control_node
```

### Start the joystick driver

```bash
ros2 run joy joy_node
```

### Start drive teleoperation

```bash
ros2 run teleop_twist_joy teleop_node --ros-args \
  -p require_enable_button:=false \
  -p axis_linear.x:=1 \
  -p axis_angular.yaw:=3 \
  -p scale_linear.x:=1.5 \
  -p scale_angular.yaw:=2.0
```

### Start flipper control

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

## Controller Configuration

The `config/controllers.yaml` file defines:

* `joint_state_broadcaster`
* `flipper_position_controller`

### Flipper controller settings

* Controller type: `joint_trajectory_controller/JointTrajectoryController`
* Command interface: `effort`
* State interfaces: `position`, `velocity`
* `open_loop_control`: `false`
* `allow_partial_joints_goal`: `false`
* `allow_nonzero_velocity_at_trajectory_end`: `false`

### PID gains

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

Drive speed depends on teleoperation scaling and the Gazebo tracked-vehicle plugin limits.

```text
teleop_twist_joy scale_linear.x
             ↓
/cmd_vel.linear.x
             ↓
cmd_vel_track_control_node
             ↓
SimpleTrackedVehiclePlugin max_linear_speed
```

Increase teleop output with:

```bash
-p scale_linear.x:=3.0
```

Ensure the tracked-vehicle plugin limits are high enough in the `drok_gazebo` URDF.

## Notes

* This package assumes a compatible `drok_gazebo` robot package is installed and sourced.
* The flipper control node expects `/joy` and `/joint_states` topics to be available.
* Adjust launch parameters to tune drive scaling, flipper range, and command timing.


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
