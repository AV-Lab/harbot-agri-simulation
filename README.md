# HARBOT Agri Simulation

A ROS Noetic and Gazebo Classic workspace for multi-robot navigation and orange
harvesting in a greenhouse, using RB-Kairos mobile bases with robotic arms.

![Simulation system walkthrough preview](media/harbot_simulation_features.gif)

**[Watch the full simulation system overview](https://www.dropbox.com/scl/fi/a84vjmc8g44aoa231ryps/harbot_simulation_features.mp4?rlkey=xneqs1onafdokjcb1fav9ost0&raw=1)**

## System overview

- **Navigation:** recorded greenhouse routes, path and speed planners, and a holonomic controller.
- **Localization:** Gazebo ground-truth odometry.
- **Perception:** HSV orange detection from the Franka wrist D435 camera.
- **Manipulation:** MoveIt planning for the Franka arm; oranges hang on the plant stems and break off when grasped.
- **Task planning:** PROST/RDDL multi-robot planning through `rbkairos_etf_services`.

## Packages

- `harbot_description`: robot, arm and gripper models
- `harbot_control`: controller configuration and the attachment service
- `harbot_simulation`: Gazebo world, models and simulation launch files
- `harbot_navigation`: recorded routes, planners, controller and navigation GUI
- `harbot_moveit`: Franka MoveIt configuration
- `harbot_harvest`: perception, harvesting workflow and the PROST action server
- `simulation_plugins/`: Gazebo plugins (fruit detachment, link attacher, mimic joints, D435 camera)

## Getting started

- **[Run with Docker](docs/docker.md)**: build and start the simulation image (recommended).
- **[Build from source](docs/build-from-source.md)**: build in your own ROS Noetic workspace.
- **[Use the simulation](docs/usage.md)**: navigation, harvesting and PROST planning.

The simulation uses **Ubuntu 20.04, ROS Noetic, Python 3.8 and Gazebo 11**, which
are end-of-life software.

## Current limitations

- Orange positions are camera-detected, but fruit identity, waypoint membership and ripeness are configured.
