# Handover between human and flying robots

## simulation mode

### Description

VR in gazebo system with mocap system.

### Command

```bash
 roslaunch aerial_handover task.launch rm:=false sim:=true headless:=false robot_name:=mini_quadrotor
```

- `robot_name`: can be any types of robot supported by `jsk_aerial_robot`, like `dragon`
- two topics (`/eye/mocap/pose` and `/wrist/mocap/pose`) subscribed from mocap system.

### Example: DRAGON handover

```
roslaunch aerial_handover handover_main.launch rm:=false sim:=true headless:=false hand_source:=marker
```

Use joystick or keyboard to make the robot take off.

When `hand_source:=marker`, the hand pose in Gazebo is given by the interactive marker.

In RViz, set the hand pose using the interactive marker. The hand pose in Gazebo will be updated accordingly.

To trigger a handover task, in a separate terminal, execute:

```
rostopic pub /dragon/trigger_handover std_msgs/Empty "{}"
```

The robot will start to approach the hand using the latest hand pose.
