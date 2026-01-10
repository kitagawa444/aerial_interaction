# Dragon Handover Navigation System

## Overview

This document describes the navigation logic of the `dragon_handover_nav.py` node, which implements a safe, human-aware navigation system for aerial handover tasks using the DRAGON platform. The system ensures safe approach and interaction with humans during object handover by maintaining a configurable safety distance and executing smooth, collision-free trajectories.

## Key Features

- **Human-Centered Safety**: Maintains a minimum safety distance from the human center during all phases of navigation
- **Three-Stage Navigation**: Employs a phased approach for safe and predictable robot behavior
- **Arc Motion Planning**: Generates circular arc trajectories around the safety perimeter
- **Joint Configuration**: Automatically adjusts robot joint angles to match target pose orientation
- **RViz Visualization**: Provides real-time visual feedback of safety zones and waypoints

## Navigation Logic

### Phase 1: Task Initialization (Trigger)

When a trigger message is received on `/dragon/trigger_handover`:

1. **Target Pose Capture**: Captures the latest `robot_head_pose` as the final handover target
2. **Human Center Calculation**: 
   - Point H is calculated 180° from the robot head orientation at `safe_distance`
   - Represents the estimated human body center
   - Formula: `H = target_head_pose.position + (-cos(yaw), -sin(yaw)) * safe_distance`
3. **Safety Circle Definition**: 
   - Circle O with center H and radius `safe_distance`
   - Robot head must remain outside this circle during approach
4. **Visualization**: Immediately publishes markers for:
   - **Human center** (blue sphere): Point H location
   - **Safety circle** (red circle): Circle O perimeter
   - **Final target pose** (yellow arrow): Desired handover position
5. **Joint Configuration**: Commands joints to match target pitch angle while maintaining predefined yaw angles

### Phase 2: Safe Approach Navigation

After joint configuration is complete, the system calculates and executes a three-stage navigation:

#### Stage 1: Intermediate Position
- **Calculation**: 
  - Determines line from current CoG to human center H
  - Point M is at the intersection of this line with safety circle O
  - Robot head orientation at M points away from H (along radius)
- **Purpose**: Initial safe approach position on the perimeter
- **Visualization**: Green arrow marker shows intermediate head pose
- **Navigation**: Robot navigates to the calculated intermediate CoG position

#### Stage 2: Arc Motion
- **Trajectory Generation**: 
  - Creates `num_arc_waypoints` (default: 10) waypoints along circular arc
  - Arc spans from intermediate position (M) to final target position
  - All waypoints lie on circle O (distance = `safe_distance` from H)
- **Orientation Control**: 
  - At each waypoint, robot head x-axis points radially outward from H
  - This ensures the robot never intrudes into the safety zone
- **Safety Guarantee**: Robot head maintains exactly `safe_distance` from H throughout arc motion
- **Navigation**: Sequential waypoint following with position and yaw control

#### Stage 3: Final Handover Position
- **Target**: Moves from last arc waypoint to final target robot head pose
- **Completion**: Navigation complete when position and yaw tolerances are satisfied

## Safety Strategy

### Human-Centered Safety Circle

The system defines a safety circle (Circle O) centered at the estimated human position (Point H):

```
Direction from head to human: 180° rotation of head orientation
H_x = target_head_pose.x - cos(target_yaw) * safe_distance
H_y = target_head_pose.y - sin(target_yaw) * safe_distance
Circle O: All points at distance safe_distance from H
```

### Key Safety Features

1. **Minimum Distance Guarantee**: Robot head maintains at least `safe_distance` from H throughout motion
2. **Predictable Circular Trajectories**: Arc motion follows circle O, providing predictable robot behavior
3. **Radial Orientation**: Robot head always faces away from human center during approach
4. **No Safety Zone Intrusion**: Mathematical guarantee that robot never enters circle O
5. **Configurable Safety**: `safe_distance` parameter adjustable for different scenarios and risk levels

## ROS Interface

### Subscribed Topics

| Topic | Message Type | Description |
|-------|-------------|-------------|
| `/robot_head_pose` | geometry_msgs/PoseStamped | Target pose for robot head during handover |
| `/dragon/trigger_handover` | std_msgs/Empty | Trigger to start handover task |
| `/dragon/uav/cog/odom` | nav_msgs/Odometry | Robot center of gravity odometry feedback |
| `/dragon/joint_states` | sensor_msgs/JointState | Current joint positions |

### Published Topics

| Topic | Message Type | Description |
|-------|-------------|-------------|
| `/dragon/uav/nav` | aerial_robot_msgs/FlightNav | UAV navigation commands (CoG target) |
| `/dragon/joints_ctrl` | sensor_msgs/JointState | Joint position commands |
| `/dragon/intermediate_pose_marker` | visualization_msgs/Marker | Visualization markers (multiple namespaces) |

### Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `~safe_distance` | float | 1.0 | Safety radius around human center (meters) |
| `~num_arc_waypoints` | int | 10 | Number of waypoints for arc motion trajectory |
| `~link_length` | float | 0.44 | Approximate distance from CoG to robot head (meters) |
| `~nav_position_tolerance` | float | 0.1 | Position tolerance for waypoint achievement (meters) |
| `~nav_yaw_tolerance` | float | 0.1 | Yaw tolerance for waypoint achievement (radians) |
| `~position_tolerance` | float | 0.05 | Joint position tolerance for configuration completion (radians) |

## Visualization Markers

The system publishes multiple markers for RViz visualization:

| Marker | Type | Color | ID | Description |
|--------|------|-------|----|-------------|
| Human Center (H) | Sphere | Blue | 2 | Estimated human body center |
| Safety Circle (O) | Line Strip | Red | 1 | Safety perimeter (radius = safe_distance) |
| Intermediate Pose (M) | Arrow | Green | 0 | Initial approach position on circle |
| Final Target Pose | Arrow | Yellow | 3 | Desired final handover position |

All markers use the `world` frame and have permanent lifetime until replaced.

## Coordinate Frames

- **world**: Global reference frame for all navigation commands
- **dragon/root**: Robot head link frame (x-axis points from head toward body)
- **dragon/cog**: Robot center of gravity frame (control point for UAV navigation)

## Transform Chain

The system uses TF transformations to convert between head pose and CoG pose:

```
Target Head Pose (world frame)
    ↓
Target Root Transform (dragon/root in world)
    ↓ (apply root→cog transform)
Target CoG Pose (world frame)
    ↓
Navigation Command (to /dragon/uav/nav)
```

## Joint Configuration Strategy

The system controls 6 joints with the following configuration:

| Joint | Control Strategy | Purpose |
|-------|------------------|---------|
| joint1_pitch | Matches target head pose pitch (negated) | Align head pitch with target |
| joint1_yaw | Fixed at -45° (-0.785 rad) | Predefined handover configuration |
| joint2_pitch | Fixed at 0° | Extended arm configuration |
| joint2_yaw | Fixed at 90° (1.57 rad) | Lateral extension |
| joint3_pitch | Fixed at 0° | Straight segment |
| joint3_yaw | Fixed at 90° (1.57 rad) | Final orientation |

## Navigation State Machine

```
┌──────────────────────────────────────────────────────────┐
│                         IDLE                              │
│                    (waiting for trigger)                  │
└───────────────────────┬──────────────────────────────────┘
                        │ trigger_handover received
                        ↓
┌──────────────────────────────────────────────────────────┐
│               JOINT_TRANSFORMATION                        │
│  - Calculate human center H and safety circle O          │
│  - Publish visualization markers (H, O, target)           │
│  - Send joint commands                                    │
│  - Monitor joint states                                   │
└───────────────────────┬──────────────────────────────────┘
                        │ joints reach target
                        ↓
┌──────────────────────────────────────────────────────────┐
│        STAGE 1: INTERMEDIATE NAVIGATION                   │
│  - Calculate intermediate point M on circle O             │
│  - Publish intermediate pose marker (green)               │
│  - Navigate to intermediate CoG position                  │
└───────────────────────┬──────────────────────────────────┘
                        │ intermediate position reached
                        ↓
┌──────────────────────────────────────────────────────────┐
│           STAGE 2: ARC MOTION                             │
│  - Generate arc waypoints along circle O                  │
│  - Navigate sequentially through waypoints                │
│  - Maintain safe_distance from H                          │
└───────────────────────┬──────────────────────────────────┘
                        │ all arc waypoints completed
                        ↓
┌──────────────────────────────────────────────────────────┐
│           STAGE 3: FINAL NAVIGATION                       │
│  - Navigate to final target CoG pose                      │
└───────────────────────┬──────────────────────────────────┘
                        │ final position reached
                        ↓
┌──────────────────────────────────────────────────────────┐
│                       COMPLETE                            │
└──────────────────────────────────────────────────────────┘
```

## Arc Motion Geometry

The arc motion follows these geometric principles:

1. **Intermediate Angle**: `θ_M = atan2(M_y - H_y, M_x - H_x)`
2. **Final Angle**: `θ_final = atan2(target_y - H_y, target_x - H_x)`
3. **Angular Sweep**: `Δθ = θ_final - θ_M` (normalized to [-π, π])
4. **Waypoint Interpolation**: 
   ```
   For i = 1 to num_arc_waypoints:
       t = i / num_arc_waypoints
       θ_i = θ_M + Δθ * t
       x_i = H_x + safe_distance * cos(θ_i)
       y_i = H_y + safe_distance * sin(θ_i)
       yaw_i = θ_i (pointing radially outward from H)
   ```

## Error Handling

The system includes robust error handling:

- **Missing Target Pose**: Warns and ignores trigger if no robot_head_pose received
- **Concurrent Tasks**: Prevents starting new handover if task already in progress
- **Robot Too Close**: Warns if current position is within safety distance but proceeds
- **TF Lookup Failures**: Catches TF exceptions and aborts navigation safely
- **Zero Distance**: Handles edge case of robot at exact human center position

## Implementation Notes

### Coordinate System Convention
- Robot head x-axis points from head to body (important for orientation calculations)
- Yaw angles: 0° = +X axis, increases counter-clockwise (ROS convention)
- All safety calculations performed in 2D (XY plane)
- Z-coordinate maintained from target pose

### Transform Calculations
- Uses TF library to get `dragon/root` → `dragon/cog` transform
- Applies transform chain: `world` → `target_root` → `cog`
- Ensures CoG navigation commands achieve desired head pose

### Marker Publishing
- Human center, safety circle, and final target markers published immediately upon trigger
- Intermediate pose marker published when navigation stage starts
- All markers use world frame and permanent lifetime for persistent visualization

## Usage Example

1. Launch the system:
   ```bash
   roslaunch aerial_handover handover_main.launch safe_distance:=1.0
   ```

2. Set target hand pose using interactive marker in RViz

3. Trigger handover:
   ```bash
   rostopic pub /dragon/trigger_handover std_msgs/Empty "{}"
   ```

4. Observe in RViz:
   - Blue sphere: Human center
   - Red circle: Safety perimeter
   - Yellow arrow: Final target
   - Green arrow: Intermediate waypoint (appears after joint config)
   - Robot follows: Intermediate → Arc → Final
