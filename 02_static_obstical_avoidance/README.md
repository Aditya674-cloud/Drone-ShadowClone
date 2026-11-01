# Static Obstacle Avoidance

## 1. Objective

The first autonomous obstacle-avoidance stage of the Drone-ShadowClone project is focused on detecting a static obstacle using a simulated 2D LiDAR and using the detected distance to influence the drone's motion.

The objective of this stage is to progress from:

**Manual flight → autonomous flight → obstacle perception → obstacle response → obstacle avoidance**

At this stage, the drone is being tested in Gazebo with a single static obstacle.

---

## 2. Simulation Environment

### Hardware

- HP Omen 16
- NVIDIA RTX 4060 Max-Q 8 GB
- Intel i7 14th Gen HX
- 16 GB RAM

### Software

- Ubuntu 24.04.4 LTS
- ROS 2 Jazzy
- Gazebo Harmonic
- PX4 SITL
- Micro XRCE-DDS Agent
- QGroundControl
- Python
- `px4_msgs`
- `ros_gz_bridge`

---

## 3. Static Obstacle

A static red box was added to the Gazebo world.

### Obstacle configuration

- Position: approximately `(3, 0, 1)`
- Size: `1 × 1 × 2 m`
- Static: `true`

The obstacle was added to:

```text
Tools/simulation/gz/worlds/shadowclone_obstacle.sdf
```

The custom world was then launched using:

```bash
PX4_GZ_WORLD=shadowclone_obstacle make px4_sitl gz_x500_lidar_2d
```

---

## 4. LiDAR Integration

A 2D LiDAR-equipped X500 model was used.

The LiDAR provides:

- 1080 scan samples
- 270° field of view
- 0.1–30 m measurement range
- approximately 30 Hz update rate

The Gazebo LiDAR topic is:

```text
/world/shadowclone_obstacle/model/x500_lidar_2d_0/link/link/sensor/lidar_2d_v2/scan
```

The Gazebo LaserScan message was bridged into ROS 2 using `ros_gz_bridge`.

---

## 5. ROS 2 Obstacle Detection

A ROS 2 package called:

```text
drone_obstacle_perception
```

was created.

The obstacle detector subscribes to the LiDAR `LaserScan` topic and analyses the forward sector.

The forward sector was defined approximately as:

```text
-15° to +15°
```

Only valid finite LiDAR measurements are considered.

The closest valid measurement is reported as the estimated forward obstacle distance.

Example detection:

```text
FRONT OBSTACLE DETECTED: 2.38 m
```

This confirmed that the LiDAR was capable of detecting the static obstacle.

---

## 6. PX4 Obstacle Distance Investigation

PX4 provides the following input topic:

```text
/fmu/in/obstacle_distance
```

with message type:

```text
px4_msgs/msg/ObstacleDistance
```

The message contains:

- 72 distance sectors
- distance values in centimetres
- minimum and maximum measurement limits
- angular increment
- angular offset
- sensor type
- coordinate frame

A separate test publisher was developed to convert the ROS `LaserScan` data into the PX4 `ObstacleDistance` format.

For safety, this was initially published to a test topic:

```text
/shadowclone/obstacle_distance_test
```

rather than directly into PX4.

The test output successfully contained obstacle measurements around:

```text
238–242 cm
```

which was consistent with the approximately 2.38 m LiDAR detection.

---

## 7. Autonomous Obstacle-Response Controller

A separate controller was created:

```text
obstacle_avoidance_controller.py
```

This controller was intentionally kept separate from the existing waypoint controller.

The existing waypoint controller was not modified.

The new controller uses:

- PX4 Offboard control
- `TrajectorySetpoint`
- velocity commands
- vehicle local position
- vehicle status
- LiDAR `LaserScan`
- forward obstacle distance

The intended control sequence is:

```text
Takeoff
   ↓
Reach approximately 2 m altitude
   ↓
Move forward
   ↓
Monitor LiDAR
   ↓
Obstacle detected
   ↓
Stop
   ↓
Hold position
   ↓
Land
```

The initial test was deliberately designed as an obstacle-response test rather than full lateral avoidance.

---

# 8. First Autonomous Flight Test

The controller was successfully launched through ROS 2:

```bash
ros2 run px4_offboard_control obstacle_avoidance_controller
```

PX4 successfully passed its preflight checks after connecting QGroundControl.

PX4 reported:

```text
INFO [commander] Ready for takeoff!
```

The drone then successfully:

- armed
- took off
- maintained altitude

This confirmed that the new Offboard controller was capable of commanding the simulated drone into autonomous flight.

---

# 9. Problem Encountered — Drone Takes Off but Does Not Move Forward

During the first test, the drone successfully took off and hovered.

However, it did not proceed toward the obstacle.

The controller reported:

```text
Forward movement stopped.
```

The controller's safety logic stopped forward movement because the LiDAR data was considered stale.

Therefore, the drone remained hovering instead of continuing forward without reliable obstacle information.

This behaviour is currently considered a **sensor/data-pipeline issue under investigation**, not a successful obstacle-avoidance result.

---

# 10. Why This Is Important

The test demonstrated an important principle in autonomous robotics:

> The controller should not continue moving when its perception data cannot be trusted.

Instead of blindly commanding forward motion, the controller entered a safe hold condition when fresh LiDAR data was not available.

The next development step is therefore to diagnose the ROS 2 ↔ Gazebo LiDAR data pipeline and ensure that fresh `LaserScan` messages reach the controller reliably.

---

# 11. Current Status

### Completed

- [x] PX4 SITL configured
- [x] Gazebo environment configured
- [x] Custom static obstacle created
- [x] LiDAR-equipped drone launched
- [x] Gazebo LiDAR topic identified
- [x] Gazebo → ROS 2 LaserScan bridge configured
- [x] ROS 2 obstacle detector created
- [x] Static obstacle detected using LiDAR
- [x] PX4 `ObstacleDistance` interface investigated
- [x] Test obstacle-distance publisher created
- [x] Autonomous Offboard controller created
- [x] PX4 preflight checks passed
- [x] Drone autonomously took off
- [x] Drone successfully maintained altitude
- [x] Safety response triggered when LiDAR data was considered stale

### Not completed yet

- [ ] Reliable real-time LiDAR data into the autonomous controller
- [ ] Forward approach toward the obstacle
- [ ] Reliable obstacle stopping
- [ ] Left/right obstacle avoidance
- [ ] Path replanning
- [ ] Dynamic obstacle avoidance
- [ ] Multi-drone coordination

---

# 12. Development Philosophy

This project is being developed incrementally.

Each stage is tested independently before adding another layer of autonomy.

Current development progression:

```text
PX4 Manual Flight
       ↓
Waypoint Flight
       ↓
Gazebo Environment
       ↓
Static Obstacle
       ↓
LiDAR Perception
       ↓
Obstacle Distance Estimation
       ↓
Autonomous Obstacle Response   ← CURRENT STAGE
       ↓
Reactive Avoidance
       ↓
Dynamic Obstacles
       ↓
Path Planning
       ↓
Multi-Drone / Swarm
```

The objective is not only to produce a final demonstration, but to document the engineering problems, experiments, failures, fixes and design decisions encountered throughout development.
