# Using the simulation

[Back to README](../README.md) · [Docker setup](docker.md) · [Source build](build-from-source.md)

- [Start the simulation](#start-the-simulation)
- [Recorded-path navigation](#recorded-path-navigation)
- [Harvesting workflow](#harvesting-workflow)
- [PROST integration](#prost-integration)

Run ROS commands in a sourced Noetic terminal. With Docker, open a container shell
from the repository root on the host:

```bash
docker compose exec simulation /harbot_entrypoint.sh bash
```

For a source build, use `source ~/catkin_ws/devel/setup.bash` in each terminal.
Use only one navigation or harvesting workflow to command each robot at a time.

## Start the simulation

**Docker:** the startup script already launches the simulation. Skip to navigation
or harvesting below.

**Source build:** start the complete simulation:

```bash
roslaunch harbot_simulation rbkairos_complete.launch
```

To choose spawn waypoints, use this command **instead**:

```bash
roslaunch harbot_simulation rbkairos_complete.launch waypoint_robot_a:=a1 waypoint_robot_b:=a40
```

Robot A uses waypoints `a1`–`a20`; Robot B uses `a21`–`a40`.
Use this map to choose spawn locations and recorded route endpoints:

![Greenhouse waypoint map](../media/greenhouse_waypoint_map.jpg)

## Recorded-path navigation

The navigation components have separate roles:

- **Global planner:** loads a recorded JSON route and publishes the full path.
- **Local planner:** publishes the remaining path, current point index and permission to move.
- **Speed planner:** selects the matching recorded speed, or zero when movement is blocked.
- **Controller:** uses the local path, speed and odometry to publish `cmd_vel`.

Select components in [`navigation.yaml`](../harbot_navigation/config/navigation.yaml):

```yaml
global_planner: recorded_path
local_planner: path_follower
speed_planner: recorded_speed
controller: holonomic_controller
```

Tune settings in each component's `config.yaml` under
[`harbot_navigation/scripts/`](../harbot_navigation/scripts/). Keep `path_folder`
the same for the recorded-path and recorded-speed components. Docker users must
rebuild the image after changing these files on the host.

### Run a route

With robots at `a1` and `a40`, run these in separate sourced terminals:

```bash
# Robot A
roslaunch harbot_navigation path_nav.launch robot_ns:=robot route:=a1-a2 gui:=true
```

```bash
# Robot B
roslaunch harbot_navigation path_nav.launch robot_ns:=robot_b route:=a40-a39 gui:=true
```

Choose a route starting at the robot's current location. The short name
`start-destination` resolves the folder and `.json` extension automatically.
Restart that robot's navigation launch with a new `route` for its next trip.
Negative recorded speeds select reverse motion.

For automatic start selection, use only a destination, e.g. `route:=a20` or
`route:=ul2_2`. Odometry selects the nearest recorded start. This loads an
existing route; it does not create a new path. If the selected start is already
the destination, the robot stays stopped.

Launch while stopped near one clear waypoint, with odometry in the recorded
path's coordinate frame. The path and speed planners select their starts
independently, so wait for both to load before pressing **Start**. There is no
distance limit or automatic path from an arbitrary position to the waypoint.

### Start and stop

Press **Start** or **Stop** in the GUI. With `gui:=false`, use these instead
(replace `/robot` with `/robot_b` for Robot B):

```bash
# Start
rostopic pub -1 /robot/path_nav/proceed_flag std_msgs/Bool "data: true"
# Stop
rostopic pub -1 /robot/path_nav/proceed_flag std_msgs/Bool "data: false"
```

## Harvesting workflow

Start the simulation first, then start the detector for the robot you will use:

```bash
rosrun harbot_harvest hsv_object_detector.py _robot_ns:=/robot
```

For Robot B, use `_robot_ns:=/robot_b`. Run one detector per robot being used;
do not start another if the PROST launch below already started it.

In another sourced terminal:

```bash
rosrun harbot_harvest harvest_workflow.py
```

This workflow does not need PROST. It starts navigation itself, so stop any other
navigation or external control for the same robot first.

The prompts select robot `a` (`/robot`) or `b` (`/robot_b`), whether to show the
path GUI, and manual or automatic operation:

- **Manual:** detects the current waypoint, lists reachable destinations, navigates
  to the selected destination and harvests the selected fruits one by one.
- **Automatic:** harvests configured ripe fruits, skips configured unripe fruits,
  follows the robot's waypoint order, then unloads by opening the container bottom.

Automatic waypoint order:

- Robot A: `a1` → `a8`, then `a16` → `a9`, then `a17` → `a20`
- Robot B: `a40` → `a33`, then `a25` → `a32`, then `a24` → `a21`

End-to-end harvesting currently targets the Franka configuration; see the
[limitations](../README.md#current-limitations).

## PROST integration

This requires the [external PROST packages](build-from-source.md#optional-integrations)
and is **not available in the default Docker image**.

Start the detector and simulation action server for each robot in separate terminals:

```bash
roslaunch harbot_harvest prost_harvest.launch robot_ns:=/robot
```

```bash
roslaunch harbot_harvest prost_harvest.launch robot_ns:=/robot_b
```

These launch files do **not** start the external PROST planner or load an RDDL
task. Those must be configured and started separately from that project's setup.
Use this as an alternative to the standalone harvesting workflow, not alongside
another workflow controlling the same robot.
