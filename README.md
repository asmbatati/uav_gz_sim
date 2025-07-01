# uav_gz_sim

Simulation environment that can be used for UAV frameworks.

## Dependencies

* ROS 2 jazzy + Gazebo `harmonic`
* PX4 Atuopilot

## Installation

A Docker image for the simulation development environment is available at [aaa](aaa). It includes Ubuntu 24.04, ROS 2 Jazzy + Gazebo Harmonic, and PX4 Autopilot.

## Run

* Combile the workspace using `colcon build`

* Source the workspace source `install/setup.bash`

* Launch Simulation: In your first terminal, initiate the simulation by running:

```bash
ros2 launch uav_gz_sim sim.launch.py
```
Upon execution, Gazebo should display a `quadcopter` and `world`.