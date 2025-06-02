# Drone Control Algorithms

ROS2 Python package for autonomous drone control algorithms, designed to work with PX4 and Gazebo simulation.

## Team Members
- **Member 1**: Flight Control & Safety Systems
- **Member 2**: Navigation & Path Planning  
- **Member 3**: Communication & Integration

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
- ROS2 Humble/Iron
- Python 3.8+
- PX4-Autopilot (for simulation interface)

### Installation
```bash
# Clone repository
git clone https://github.com/your-username/drone-control-algorithms.git
cd drone-control-algorithms

# Install dependencies
pip install -r requirements.txt

# Build ROS2 package
colcon build
source install/setup.bash
