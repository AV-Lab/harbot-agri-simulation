# Build from source

[Back to README](../README.md) · [Usage](usage.md)

Use this for your own ROS Noetic workspace. Docker users can skip it.

## Requirements

- Ubuntu 20.04, ROS Noetic, Python 3.8 and Gazebo 11
- Git, Git LFS, rosdep and `python3-catkin-tools`
- A graphical desktop with a GPU

## 1. Get the code

Clone this repository and download its mesh files:

```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone https://github.com/AV-Lab/harbot-agri-simulation.git
git -C harbot-agri-simulation lfs pull
```

Clone the PROST planning packages next to it:

```bash
git clone https://github.com/etfrobotics/prost_ros.git
git -C prost_ros checkout 72028b15f452f9f1fd39d07f7eee85807fa3f038
git clone -b SIM-integration https://github.com/etfrobotics/rbkairos_etf_services.git
```

## 2. Build the PROST planner

Install its build tools and the Python RDDL library, then compile the planner:

```bash
sudo apt install bison flex libbdd-dev libz3-dev python3-pip
pip3 install pyRDDLGym==2.7
cd ~/catkin_ws/src/prost_ros/prost
./build.py
```

## 3. Build the workspace

Install the ROS dependencies and build. Run `sudo rosdep init` first if rosdep
has never been set up.

```bash
cd ~/catkin_ws
rosdep update --rosdistro noetic --include-eol-distros
rosdep install --from-paths src --ignore-src --rosdistro noetic -y --skip-keys="stomp_moveit"
catkin build
```

## 4. Source the workspace

Run this in every new terminal:

```bash
source ~/catkin_ws/devel/setup.bash
```

Continue with the [usage guide](usage.md).
