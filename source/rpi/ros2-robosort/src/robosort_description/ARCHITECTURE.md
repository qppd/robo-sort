# RoboSort Robot Structure

## Physical Layout (Top View)

```
                    FRONT
                      ↑
        ┌─────────────────────────┐
        │                         │
        │     ▲ LiDAR (LD06)      │
        │     │                   │
    ●   │     │                   │   ●  ← Caster wheels
        │                         │
        │    ┌─────────────┐      │
        │    │   Robot     │      │
        │    │   Body      │      │
        │    │  (Arduino)  │      │
        │    └─────────────┘      │
        │                         │
     ◉  │                         │  ◉   ← Drive wheels (12")
        │                         │
        └─────────────────────────┘
    
    Legend:
    ◉ = Drive wheel (12" diameter, differential drive)
    ● = Caster wheel (omni-directional)
    ▲ = LiDAR sensor (360° scanning)
```

## Physical Layout (Side View)

```
        ┌─┐  ← LiDAR (~6cm tall)
        └┬┘
    ┌────┴────────────────┐
    │   Robot Body        │  ← 4" height
    │  (Main Electronics) │
    └─────────────────────┘
           │   │
           ◉   ◉  ← 12" diameter wheels
```

## TF Frame Tree

```
odom (world frame, published by diff_drive plugin)
 │
 └── base_footprint (on ground, projection of base_link)
      │
      └── base_link (robot center of mass)
           │
           ├── left_wheel_link (left drive wheel)
           │    └── [continuous joint, rotates around Y-axis]
           │
           ├── right_wheel_link (right drive wheel)
           │    └── [continuous joint, rotates around Y-axis]
           │
           ├── front_left_caster_link (front-left support)
           │    └── [fixed joint]
           │
           ├── front_right_caster_link (front-right support)
           │    └── [fixed joint]
           │
           └── lidar_link (LiDAR sensor)
                └── [fixed joint, offset to front-top]
```

## Coordinate System (REP-103)

```
     Z (up)
     ↑
     │    X (forward)
     │   ↗
     │  /
     │ /
     │/
     └─────────→ Y (left)
```

- **X-axis**: Points forward (direction of travel)
- **Y-axis**: Points left (perpendicular to motion)
- **Z-axis**: Points up (perpendicular to ground)

## Dimensions

### Body
- Length (X): 35" (0.889 m)
- Width (Y): 20" (0.508 m)
- Height (Z): 4" (0.1016 m)
- Mass: 15 kg

### Wheels
- Diameter: 12" (0.3048 m)
- Width: 2" (0.05 m)
- Separation: 22" (0.558 m)
- Mass per wheel: 0.5 kg

### LiDAR
- Position: Front-center, top of robot
- Offset from center: +0.35m (X), 0.0m (Y), +0.11m (Z)
- Field of view: 360°
- Max range: 12 m

## Control Flow

```
┌──────────────────────────────────────────────────────────────┐
│                        User Input                             │
└───────────────────┬──────────────────────────────────────────┘
                    │
        ┌───────────┴───────────┐
        │                       │
    [Teleop]              [Autonomous]
        │                       │
        │        /cmd_vel       │
        └───────────┬───────────┘
                    │
                    ↓
         ┌──────────────────────┐
         │ Gazebo Diff Drive    │
         │    Controller        │
         └──────────┬───────────┘
                    │
         ┌──────────┴──────────┐
         │                     │
    [Left Wheel]          [Right Wheel]
         │                     │
         └──────────┬──────────┘
                    │
                    ↓
            Robot Movement
                    │
                    ↓
         ┌──────────────────────┐
         │   Odometry (/odom)   │
         │   TF (odom→base)     │
         └──────────────────────┘
```

## Sensor Data Flow

```
┌──────────────┐
│   LiDAR      │
│  (360° scan) │
└──────┬───────┘
       │
       │ /scan (LaserScan)
       │
       ↓
┌──────────────────┐
│ Obstacle         │
│ Avoidance        │
└──────┬───────────┘
       │
       │ /cmd_vel (if obstacle)
       │
       ↓
┌──────────────────┐
│ Diff Drive       │
│ Controller       │
└──────────────────┘
```

## Hardware vs Simulation

### Simulation Mode (Gazebo)
```
Gazebo World
    │
    ├─→ Differential Drive Plugin → /cmd_vel subscriber
    ├─→ LiDAR Ray Sensor → /scan publisher
    ├─→ Joint State Publisher → /joint_states
    └─→ Odometry Publisher → /odom, /tf
```

### Hardware Mode (Real Robot)
```
Real Robot
    │
    ├─→ motor_controller → Serial → Arduino → DC Motors
    ├─→ ldlidar_stl_ros2 → /scan publisher
    ├─→ tf_broadcaster → /tf (dead reckoning)
    └─→ encoder feedback (future) → /odom
```

## ROS2 Topics Architecture

```
                    RoboSort Node Graph
                    
/cmd_vel ──────────┐
                   │
/scan ─────────────┤
                   │
/odom ─────────────┼──→ [motor_controller]
                   │         │
/joint_states ─────┤         ├──→ Serial ──→ Arduino
                   │         │
/tf ───────────────┤         └──→ /robosort/motor_status
                   │
/robosort/* ───────┼──→ [obstacle_avoidance]
                   │         │
                   │         └──→ /cmd_vel (safety override)
                   │
                   └──→ [tf_broadcaster]
                            │
                            └──→ /tf (odom→base_footprint)
```

## Launch Configurations

### 1. View Robot (RViz Only)
```
view_robot.launch.py
├── robot_state_publisher
├── joint_state_publisher
└── rviz2
```

### 2. Gazebo Simulation
```
gazebo.launch.py
├── gzserver (physics)
├── gzclient (GUI)
├── robot_state_publisher
└── spawn_entity.py
```

### 3. Complete Simulation
```
simulation.launch.py
├── gazebo.launch.py
│   ├── gzserver
│   ├── gzclient
│   ├── robot_state_publisher
│   └── spawn_entity.py
├── rviz2
└── obstacle_avoidance
```

## File Organization

```
robosort_description/
├── urdf/
│   ├── robosort.urdf.xacro          # Main robot model
│   └── robosort_arm.urdf.xacro      # Arm model (separate)
├── launch/
│   ├── view_robot.launch.py         # RViz visualization
│   ├── gazebo.launch.py             # Gazebo simulation
│   └── simulation.launch.py         # Complete system
├── config/
│   └── robot_params.yaml            # Configuration parameters
├── rviz/
│   └── view_robot.rviz              # RViz settings
├── meshes/
│   └── [STL/DAE files if needed]
├── README.md                         # Full documentation
├── QUICKSTART.md                     # Quick reference
└── IMPLEMENTATION_SUMMARY.md         # This summary
```
