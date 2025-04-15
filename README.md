A PyBullet-based simulation environment for Robile robots with multi-robot formation control capabilities.

## Overview

RobilePyBullet provides a comprehensive simulation framework for Robile robotic platforms using the PyBullet physics engine. This package allows for the simulation of individual Robile robots as well as multiple robots in formation configurations.

The package is designed to be used within a ROS 2 workspace and serves as a lightweight alternative to Gazebo for testing control algorithms and robot behaviors.

## Features

- **Single Robot Simulation**: Control and simulate individual Robile robots in a PyBullet environment
- **Multi-Robot Formation Control**: Simulate leader-follower formations with multiple Robile robots
- **Omnidirectional Movement**: Control robots with omnidirectional motion capabilities
- **Realistic Physics**: Integration with PyBullet's physics engine for realistic simulation
- **ROS 2 Integration**: Designed to work within a ROS 2 ecosystem

## Installation

### Prerequisites

- ROS 2 (tested on Humble)
- Python 3.6+
- PyBullet

### Setup

1. Clone this repository into your ROS 2 workspace:
   ```bash
   cd ~/ros2_ws/src
   git clone https://github.com/SuneshSundarasami/MultiRobot-Simulation-PyBullet.git
   ```

2. Install the required dependencies:
   ```bash
   pip install pybullet numpy
   ```

3. Build the package:
   ```bash
   cd ~/ros2_ws
   colcon build --packages-select robile_pybullet
   ```

4. Source the workspace:
   ```bash
   source ~/ros2_ws/install/setup.bash
   ```

## Usage

### Single Robot Simulation

To run a basic simulation with a single Robile robot:

```bash
ros2 run robile_pybullet robile_sim
```

### Multi-Robot Formation Control

To run a simulation with multiple Robile robots in a formation:

```bash
python3 ~/ros2_ws/src/robile_pybullet/robile_pybullet/multi_robot_sim.py
```

This will start a simulation with one leader robot and four follower robots in a V-formation.

## Key Components

### MultiRobileController

The `MultiRobileController` class handles the coordination of multiple Robile robots in formation. It provides methods for:

- Initializing robots in formation
- Controlling robot movement
- Maintaining formation using PID control
- Computing appropriate steering angles and wheel velocities

### SimpleDistanceController

The `SimpleDistanceController` implements a PID controller for maintaining positions in formation.

### Formation Configuration

Formations are defined as relative positions to a leader robot. For example, a V-formation is configured as:

```python
formation_config = [
    (-1.0, -1.0),  # First follower position
    (-1.0, 1.0),   # Second follower position
    (-2.0, -2.0),  # Third follower position
    (-2.0, 2.0),   # Fourth follower position 
]
```

## Robot Control

### Omnidirectional Movement

The robots can be controlled with three parameters:
- `vx`: Velocity in the x-direction (forward/backward)
- `vy`: Velocity in the y-direction (left/right)
- `omega`: Angular velocity for rotation

Example:
```python
controller.move_omnidirectional(0.5, 0, 0, robot_id)  # Move forward
controller.move_omnidirectional(0, 0.5, 0, robot_id)  # Move sideways
controller.move_omnidirectional(0, 0, 0.5, robot_id)  # Rotate in place
```

## File Structure

- multi_robot_sim.py: Implementation of multi-robot formation control
- `robile_sim.py`: Single robot simulation interface
- `robot_controller.py`: Base robot controller implementation
- `laser_scanner.py`: Simulated laser scanner for the robots
- `test_movement.py`, `test_movement_2.py`: Example movement test scripts
- robile.urdf, robile3_config.urdf: URDF model files for the Robile robot

## License

This package contains components with various licenses:
- The Robile URDF models are provided under a dual-license: GNU Lesser General Public License LGPL 2.1 and BSD license
- The included PyBullet object models are under the LGPL 2.1 license

For more details, see the license information in the respective files.

## Contributing

Contributions to improve RobilePyBullet are welcome. Please feel free to submit pull requests or report issues on the project's GitHub page.

## Contact

For questions or feedback, please contact the maintainer at sunesh@outlook.de.
