# Test Results — 5 m Waypoint Navigation

## 1. Objective

Validate the autonomous waypoint-navigation controller using PX4 SITL, Gazebo, ROS 2 Jazzy, and Python. The objective was to command the simulated drone through a sequence of waypoints and verify successful mission completion and landing.

## 2. Test Environment

- **Operating system:** Ubuntu 24.04 LTS
- **Autopilot:** PX4 SITL
- **Simulator:** Gazebo
- **Robotics middleware:** ROS 2 Jazzy
- **Communication:** Micro XRCE-DDS Agent
- **Ground control station:** QGroundControl
- **Controller:** Python waypoint controller using PX4 Offboard control

## 3. Test Procedure

1. Started the PX4 SITL simulation in Gazebo.
2. Started the Micro XRCE-DDS Agent.
3. Connected QGroundControl and verified the preflight status.
4. Launched the waypoint controller using ROS 2.
5. Tested the modified waypoint route with a 5 m movement.
6. Repeated the mission to check consistency.
7. Observed the drone's landing behaviour after completing the waypoints.

## 4. Results

| Test | Observed result |
|---|---|
| PX4 simulation startup | Successful |
| Preflight check | Passed |
| Waypoint navigation | Successful |
| 5 m route test | Successful, as observed in simulation |
| Repeated mission runs | Successful |
| Automatic landing after waypoint completion | Successful |

## 5. Conclusion

The simulated drone successfully completed the tested waypoint-navigation mission and landed automatically. Repeated runs demonstrated consistent behaviour under the tested conditions.

This milestone establishes a baseline for extending the controller with static obstacle detection and collision-free path planning.

## 6. Limitations

- Testing was performed in simulation, not on a physical drone.
- Obstacle detection and avoidance are not yet implemented in this milestone.
- The results apply to the tested route and simulation conditions; they do not establish general collision avoidance or safety.

## 7. Evidence

Add screenshots of the Gazebo simulation during waypoint navigation and after landing to the `evidence/` directory.

- `evidence/5m_route.png`
- `evidence/successful_landing.png`
