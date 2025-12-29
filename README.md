# This repository is for the aerial interaction using flying robots.

## Dependency

- `jsk_aerial_robot`: https://github.com/jsk-ros-pkg/jsk_aerial_robot

## Packages

- `aerial_handover`

## Usage

### Aerial handover

```
roslaunch aerial_handover handover_main.launch rm:=false sim:=true headless:=false hand_source:=marker
```

Use joystick or keyboard to make the robot take off.

When `hand_source:=marker`, the hand pose in Gazebo is given by the interactive marker.

In RViz, set the hand pose using the interactive marker. The hand pose in Gazebo will be updated accordingly. Meanwhile, the robot head's desired pose is published, and the robot starts to approach the hand.