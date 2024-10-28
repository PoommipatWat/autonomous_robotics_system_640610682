# Autonomous Robot System Project

Part of **Autonomous Robot System (271411)** Course at Chiang Mai University

## Student Info
- **Name:** Poommipat Wattanaprasit
- **Student ID:** 640610682
- **Faculty:** Faculty of Engineering, Chiang Mai University

## Installation Guide

### Clone and Setup Workspace
```bash
# Clone the repository
git clone https://github.com/PoommipatWat/autonomous_robotics_system_640610682.git

# Navigate to project directory
cd autonomous_robotics_system_640610682

# Build the workspace
colcon build

# Source the setup file
source install/setup.bash
```

## Usage Instructions

### Map Creation
To create a new map:
```bash
ros2 launch ai_robot keep_map.launch.py
```

### Load Saved Map with RRT*
To load your saved map and use RRT*:
```bash
ros2 launch ai_robot bringup.launch.py
```

### Load Example Map with RRT*
To load the example map and use RRT*:
```bash
ros2 launch ai_robot example.launch.py
```

---
*Project repository: [autonomous_robotics_system_640610682](https://github.com/PoommipatWat/autonomous_robotics_system_640610682)*