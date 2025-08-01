# Drone Control Algorithms

<p align="center">
    <img src="images/irik.png" alt="icon" width="300">
</p>

ROS2 Python package for autonomous drone control algorithms, designed to work with PX4 and Gazebo simulation.

## Project Timeline
20-day development sprint for complete autonomous drone control system.

## Features
- ✅ Basic flight control (takeoff, landing, hover)
- ✅ PID controllers for position/attitude control
- ✅ Safety systems and failsafe handling
- ✅ Waypoint navigation
- ✅ Path planning algorithms (A*, RRT)
- ✅ Obstacle avoidance
- ✅ MAVLink-ROS2 integration
- ✅ Telemetry and monitoring

## Quick Start

### Prerequisites
- Ubuntu 22.04
- ROS2 Humble
- Python 3.10
- PX4-Autopilot (for simulation interface)
- Micro-XRCE-DDS-Agent (for link ros with px4)
- rviz2 (for simulate movement of drone)

### Installation
```bash
# Clone repository
git clone https://github.com/alinaserinia6/irik.git
cd irik

# Install dependencies
./script -d

# Build ROS2 package
./script -b

# launch drone
./script -l
```

## Team Members
- **Member 1**: Zahra Ayed
- **Member 2**: Reyhaneh Fathollahi
- **Member 3**: Mohammad javad Shadpoor
- **Member 4**: Mohammad hosseyn Lotfi
- **Member 5**: Mahdi Gholami
- **Member 6**: Arshia Yarmohammadi
- **Member 7**: Ali Naserinia
- **Member 8**: Mohammad bagher Mohsenian
- **Member 9**: Leyla Shekari
<p align="center">
    <img src="images/team.jpg" alt="team members" width="700">
</p>