# 01 — Autonomous Waypoint Navigation (5 m Test)

## Overview

This milestone documents the development and testing of a waypoint-based autonomous drone controller using PX4 SITL, Gazebo, ROS 2 Jazzy, and Python.

The goal was to progress from manually controlling a simulated drone to executing a predefined waypoint mission and landing automatically after completing the route.

This is the first autonomy milestone in the **Drone ShadowClone** project.

## Objectives

- Understand PX4 Offboard control through ROS 2.
- Publish trajectory setpoints to control the simulated drone.
- Implement waypoint-based navigation using Python.
- Test a route involving 5 m movement.
- Verify repeatability across multiple simulation runs.
- Verify automatic landing after waypoint completion.

## Technologies Used

| Technology | Purpose |
|---|---|
| Ubuntu 24.04 LTS | Development environment |
| PX4 SITL | Software-in-the-loop flight controller |
| Gazebo | Drone simulation |
| ROS 2 Jazzy | Communication and control framework |
| Python | Waypoint controller implementation |
| Micro XRCE-DDS Agent | Communication between PX4 and ROS 2 |
| QGroundControl | Ground control and preflight verification |

## System Architecture

The controller communicates with PX4 through ROS 2 topics using the PX4 message definitions.

1. The Python controller publishes Offboard control-mode information.
2. It publishes trajectory setpoints representing the desired drone positions.
3. It sends vehicle commands for arming, Offboard mode and landing.
4. PX4 executes the commands in the simulation.
5. The controller uses vehicle status and local-position feedback to track mission progress.

## Mission Workflow

1. Start PX4 SITL and Gazebo.
2. Start the Micro XRCE-DDS Agent.
3. Connect QGroundControl and verify preflight readiness.
4. Launch the ROS 2 waypoint controller.
5. Execute the predefined waypoint sequence.
6. Complete the route and request automatic landing.
7. Repeat the test to check consistent behaviour.

## Results

The tested 5 m waypoint mission completed successfully in simulation. The drone also landed automatically after completing the waypoints, and repeated runs behaved consistently.

See [`test_results.md`](test_results.md) for the test procedure, observations and limitations.

## Project Structure

```text
01_waypoint_navigation_5m/
├── README.md
├── waypoint_controller.py
├── test_results.md
└── evidence/
    ├── 5m_route.png
    └── successful_landing.png
```

## Limitations

- The mission has been tested in simulation only.
- This version follows predefined waypoints; it does not yet implement obstacle detection or obstacle avoidance.
- Testing on this route does not establish general flight safety or collision avoidance.

## Next Milestone

**02 — Static Obstacle Avoidance**

The next stage will introduce a static obstacle in Gazebo and develop the sensing and path-planning logic needed to navigate around it.

## Learning Outcomes

- Understanding PX4 Offboard control.
- Working with ROS 2 publishers and subscribers.
- Using trajectory setpoints for waypoint navigation.
- Integrating a Python controller with PX4 SITL.
- Testing autonomous mission completion and landing.

---

Part of the **Drone ShadowClone** project: a step-by-step exploration of autonomous navigation, obstacle avoidance, and eventually multi-drone coordination.

