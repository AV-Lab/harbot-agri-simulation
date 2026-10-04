# Build from source

[Back to README](../README.md) · [Docker setup](docker.md) · [Usage](usage.md)

Use this guide for an existing ROS Noetic environment. Docker users can follow
the [Docker guide](docker.md) instead; its workspace is already built.

## Requirements

- Ubuntu 20.04, ROS Noetic and Python 3.8
- Git LFS, rosdep and `python3-catkin-tools` (`catkin build`)
- A working graphical desktop/GPU for Gazebo 11

## Clone and build

Clone into your catkin workspace and download the mesh assets:

```bash
source /opt/ros/noetic/setup.bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone https://github.com/AV-Lab/harbot-agri-simulation.git
cd harbot-agri-simulation
git lfs pull
```

Initialize rosdep once with `sudo rosdep init` if it has not already been
initialized. Install the core simulation dependencies and build:

```bash
cd ~/catkin_ws
rosdep update --rosdistro noetic --include-eol-distros
rosdep install --from-paths src --ignore-src --rosdistro noetic -y \
  --skip-keys="stomp_moveit rbkairos_etf_services"
catkin build
source devel/setup.bash
```

The skipped packages are optional, not needed for the default simulation.
Required Gazebo plugins are included in [`simulation_plugins/`](../simulation_plugins/).

In each new terminal, source the workspace before running ROS commands:

```bash
source ~/catkin_ws/devel/setup.bash
```

After updating code or models, run `git lfs pull` from the repository root,
rebuild with `catkin build` from `~/catkin_ws`, and source the workspace again.
Continue with [starting the simulation](usage.md#start-the-simulation).

## Optional integrations

For PROST, add [`rbkairos_etf_services` on `SIM-integration`](https://github.com/Syed-Abbas-Hussain/rbkairos_etf_services/tree/SIM-integration)
and its `prost_ros` dependency to `~/catkin_ws/src`, install their dependencies
and rebuild. They are external packages with no rosdep definitions.

See [PROST usage](usage.md#prost-integration). The external RDDL planner must be
started separately. Docker users must add these packages to the image build,
not only to a host workspace.
