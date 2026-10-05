# Run with Docker

[Back to README](../README.md) · [Usage](usage.md)

The image contains ROS Noetic, Gazebo, MoveIt, the HARBOT packages and the PROST
planning stack, already built.

## Requirements

- Linux X11/XWayland desktop with an NVIDIA GPU and working drivers
- Docker with Docker Compose **2.30+**
- [NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html)
- Git, Git LFS and `xauth`

## Build

From the repository root:

```bash
git lfs pull
docker build -t harbot-simulation:noetic .
```

The first build compiles the PROST planner and takes a while. Rebuild after any
code change, since the code is copied into the image.

## Start

```bash
./docker/run.sh
```

Gazebo opens on your desktop with robots A and B at waypoints `a1` and `a40`.
Keep this terminal open; press **Ctrl+C** to stop.

## Open a container shell

For each command in the [usage guide](usage.md), open a new shell from the
repository root:

```bash
docker compose exec simulation /harbot_entrypoint.sh bash
```

ROS is already sourced. Do not start the simulation again inside the container.

## Clean up

```bash
docker compose down
```
