# ROS 2 Turtle Hunter Simulation

**Repository:** `ros2_turtlesim_catch_them_all`

## Description

An autonomous turtle hunter simulation using ROS 2 and `turtlesim`, featuring target spawning, PID control, and custom interfaces.

This project simulates a hunter turtle that autonomously tracks and eliminates randomly spawning target turtles using a simple PID controller. It is built using ROS 2 and demonstrates modular node design, custom message/service interfaces, and coordinated launch configuration.

## Project Overview

The workspace includes three ROS 2 Python packages:

### `turtlesim_hunter_target_routine`

Contains the core logic of the simulation, including:

- Nodes for spawning target turtles at random positions
- PID-based movement control of the hunter turtle
- Logic to detect and eliminate targets

### `custom_turtle_interfaces`

Defines custom ROS 2 interfaces required by the simulation, including:

- Custom messages and services for turtle state and control

### `turtlesim_bringup`

Provides the launch configuration for the simulation:

- Main launch file: `turtlesim_catch_them_all.launch.py`
- Launches all required nodes, including turtlesim, the hunter, and the spawner

## Requirements

- ROS 2 Humble Hawksbill (or compatible distribution)
- `turtlesim` package (usually included in ROS 2 installations)

## Building the Workspace

Clone the repository and build the workspace:

```bash
git clone https://github.com/your_username/ros2_turtle_sim_project_only.git
cd ros2_turtle_sim_project_only

source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

## Running the Simulation

To start the full simulation:

```bash
ros2 launch turtlesim_bringup turtlesim_catch_them_all.launch.py
```

## Directory Structure

```text
ros2_turtle_sim_project_only/
├── src/
│   ├── custom_turtle_interfaces/         # Custom interfaces (msg, srv)
│   ├── turtlesim_bringup/                # Launch files
│   └── turtlesim_hunter_target_routine/  # Nodes for hunting and spawning logic
├── build/                                # (Excluded from version control)
├── install/                              # (Excluded from version control)
├── log/                                  # (Excluded from version control)
```

Only the `src/` directory and relevant configuration files should be tracked in version control.

## License

This project is licensed under the MIT License. See the `LICENSE` file for more information.

## Contributions

Contributions are welcome. Please submit issues or pull requests to propose improvements or report bugs.

# 
