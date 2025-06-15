# GhostSwarm: Resilient Multi-Agent System for Adversarial Environments

## Overview
This project implements a heterogeneous multi-robot system using ROS 2 for operation in GPS-denied and communication-constrained environments. The system consists of a UAV (Unmanned Aerial Vehicle) and a UGV (Unmanned Ground Vehicle) that collaborate to explore and map unknown areas while being resilient to communication disruptions.

## Features
- **Decentralized Coordination**: Agents make autonomous decisions while maintaining swarm cohesion
- **Resilient Communication**: Implements message passing that adapts to intermittent connectivity
- **Multi-Modal Perception**: Combines LIDAR and camera data for robust environment understanding
- **Dynamic Task Allocation**: Agents dynamically reassign tasks based on changing conditions

## System Architecture
![Architecture Diagram](./docs/architecture.png)

### Components
1. **UAV Agent**: Handles aerial exploration and target detection
2. **UGV Agent**: Performs ground-based navigation and object manipulation
3. **Coordinator**: Manages communication and task allocation between agents

## Prerequisites
- Ubuntu 22.04 (recommended)
- ROS 2 Humble
- Gazebo Fortress
- Python 3.8+
- OpenCV
- NumPy

## Installation

### 1. Install ROS 2 Humble
Follow the official ROS 2 Humble installation guide: [ROS 2 Humble Installation](https://docs.ros.org/en/humble/Installation.html)

### 2. Install Dependencies
```bash
sudo apt update
sudo apt install -y \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-cv-bridge \
    ros-humble-image-transport \
    ros-humble-tf2-ros \
    python3-pip

pip3 install --user numpy opencv-python
```

### 3. Clone and Build the Workspace
```bash
mkdir -p ~/ghostswarm_ws/src
cd ~/ghostswarm_ws/src
git clone https://github.com/yourusername/ghostswarm_robots.git
cd ~/ghostswarm_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

## Usage

### Launch the Simulation
```bash
# In terminal 1: Start the complete system
ros2 launch ghostswarm_robots complete_system.launch.py

# In terminal 2: Manually control the UAV (optional)
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/uav1/cmd_vel
```

### Manual Control
- Use the teleop terminal to control the UAV
- The UGV will autonomously follow the UAV
- The system will automatically detect and track blue objects

## Project Structure
```
ghostswarm_robots/
├── config/           # Configuration files
├── launch/           # Launch files
│   ├── complete_system.launch.py  # Main launch file
│   └── swarm_simulation.launch.py # Simulation launch file
├── maps/             # Map files
├── models/           # 3D models for simulation
│   ├── uav/          # UAV model
│   └── ugv/          # UGV model
├── rviz/             # RViz configurations
│   └── default.rviz  # Default RViz config
├── scripts/          # Python scripts
│   ├── uav_agent.py  # UAV agent implementation
│   ├── ugv_agent.py  # UGV agent implementation
│   └── coordinator.py # Coordination logic
├── src/              # C++ source files (if any)
├── urdf/             # Robot descriptions
├── worlds/           # Simulation worlds
│   └── adversarial_environment.world  # Main world file
├── CMakeLists.txt
└── package.xml
```

## Demo Scenario
1. The UAV takes off and begins exploring the environment
2. The UGV follows the UAV's path on the ground
3. When the UAV detects a target (blue object), it hovers above it
4. The UGV navigates to the target location
5. The system marks the target as processed and continues exploration

## Troubleshooting
- **Gazebo not starting**: Make sure to source ROS 2 and the workspace first
- **No models visible**: Check if the model paths are correctly set in the launch file
- **Communication issues**: Verify that the topics are correctly remapped

## Future Work
- Implement SLAM for better mapping
- Add more sophisticated path planning
- Improve target detection and classification
- Add more agents to the swarm

## License
This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.
