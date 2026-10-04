# HARBOT Agri Simulation

A ROS Noetic and Gazebo Classic workspace for multi-robot navigation and orange
harvesting in a greenhouse, using RB-Kairos mobile bases with robotic arms.

![Simulation system walkthrough preview](media/harbot_simulation_features.gif)

**[Watch the full simulation system overview](https://www.dropbox.com/scl/fi/a84vjmc8g44aoa231ryps/harbot_simulation_features.mp4?rlkey=xneqs1onafdokjcb1fav9ost0&raw=1)**

## System overview

- **Navigation:** recorded greenhouse routes, separate path/speed planners and a holonomic tracking controller.
- **Localization:** Gazebo ground-truth odometry.
- **Perception:** the Franka wrist D435 supplies RGB/depth images for HSV-based orange detection.
- **Manipulation:** MoveIt plans arm motion; a Gazebo attachment plugin simulates grasping.

## Package layout

- `harbot_description`: robot, arm and gripper models
- `harbot_control`: controller configuration and attachment service definition
- `harbot_simulation`: Gazebo worlds, models and simulation launch files
- `harbot_navigation`: recorded routes, planners, controller and navigation GUI
- `harbot_moveit`: Franka and UR5 MoveIt configurations
- `harbot_harvest`: perception, harvesting workflow and PROST integration

The independent Gazebo plugin packages live together in
[`simulation_plugins/`](simulation_plugins/): link attachment
(`gazebo_ros_link_attacher`), mimic joints (`roboticsgroup_gazebo_plugins`) and
D435 cameras (`realsense_d435_gazebo`). Source and license notices are kept with
the relevant packages. Images/videos are in `media/`; instructions are in `docs/`.

## Getting started

- **[Run with Docker](docs/docker.md)** — build and run the simulation image, with optional rosbridge.
- **[Build from source](docs/build-from-source.md)** — install and build in a ROS Noetic workspace.
- **[Use the simulation](docs/usage.md)** — run navigation, harvesting and optional PROST integration.

The simulation uses **Ubuntu 20.04, ROS Noetic, Python 3.8 and Gazebo 11**.

## Current limitations

- Fruit support and attachment are simulated; stems do not physically break during picking.
- Orange positions are camera-detected, but fruit identity, waypoint membership and ripeness are configured.
- End-to-end harvesting targets the Franka wrist camera/gripper. UR5 simulation and basic MoveIt execution work, but its harvesting workflow still needs adaptation.

PROST/RDDL requires [external packages](docs/build-from-source.md#optional-integrations)
and is not included in the default Docker image. ROS Noetic and Gazebo Classic
are end-of-life software.
