# HARBOT Agri Simulation

A ROS workspace for experiments with multiple robots in a greenhouse environment for the HARBOT project.

This repository provides a ROS Noetic and Gazebo Classic simulation setup based on the RBKairos mobile base integrated with a robotic manipulator, adapted for greenhouse-based research experiments.

This project is under active development.

---

## Requirements

The simulation has been developed and tested with:

- **Ubuntu 20.04**
- [**ROS Noetic**]
- **Gazebo Classic (Gazebo 11)**

Recommended hardware (tested configuration):

- **GPU:** NVIDIA Quadro RTX 4000 (8GB VRAM)
- **CPU:** Intel i7-class processor
- **RAM:** 16 GB or higher recommended

---

## Included Base Packages

This workspace includes the following upstream repositories as the base simulation framework. Some packages may be modified or extended for the HARBOT use case.

- [rbkairos_sim](https://github.com/RobotnikAutomation/rbkairos_sim)
- [egh_gripper_common](https://github.com/RobotnikAutomation/egh_gripper_common)
- [rbkairos_common](https://github.com/RobotnikAutomation/rbkairos_common)
- [roboticsgroup_gazebo_plugins](https://github.com/RobotnikAutomation/roboticsgroup_gazebo_plugins)
- [robotnik_sensors](https://github.com/RobotnikAutomation/robotnik_sensors)
- [summit_xl_common](https://github.com/RobotnikAutomation/summit_xl_common)
- [universal_robot](https://github.com/fmauch/universal_robot)
- [wsg50-ros-pkg](https://github.com/RobotnikAutomation/wsg50-ros-pkg)
- [robotnik_msgs](https://github.com/RobotnikAutomation/robotnik_msgs)

---

## Build

Clone the repository into your catkin workspace:

```bash
cd ~/catkin_ws/src
git clone https://github.com/AV-Lab/harbot-agri-simulation.git
```

Install dependencies:

```bash
cd ~/catkin_ws
rosdep install --from-paths src --ignore-src -r -y
```

Build the workspace:

```bash
catkin build
```

Source the workspace:

```bash
source devel/setup.bash
```

---

## Usage

Before starting the simulation, from the repository root set the Gazebo model path:

```bash
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$PWD/rbkairos_sim/rbkairos_gazebo/models
```

Launch the complete RB-Kairos simulation:

```bash
roslaunch rbkairos_sim_bringup rbkairos_complete.launch
```

This starts Gazebo along with the required ROS controllers.
By default, the full simulation stack is loaded (navigation, localization, and manipulation) using the **RB-Kairos + UR5 + EGH gripper** configuration with **differential drive**.

To enable omni/holonomic drive:

```bash
roslaunch rbkairos_sim_bringup rbkairos_complete.launch omni_drive:=true
```

To enable MoveIt:

```bash
roslaunch rbkairos_sim_bringup rbkairos_complete.launch moveit_movegroup_a:=true
```

For initial testing of orange picking with the UR5 arm, first run:

```bash
rosrun rbkairos_sim_bringup moveit_target_reach.py
```

This moves the arm to the target position of an orange. Then run to grape the orange:

```bash
rosrun rbkairos_sim_bringup grasp_orange.py
```

> **Note:** This is an initial simulation test flow.
