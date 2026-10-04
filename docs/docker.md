# Run with Docker

[Back to README](../README.md)

The image includes ROS Noetic, Gazebo, MoveIt and the HARBOT packages.
The workspace is built into the image, not rebuilt at startup. Rosbridge is optional.

## Requirements

- A Linux X11/XWayland desktop with an NVIDIA GPU and working drivers.
- Docker and Docker Compose **2.30+**.
- [NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html), configured for Docker.
- Git, Git LFS, and `xauth` on the host.

ROS and Gazebo do not need to be installed on the host.

## Build the image

If you have not cloned the repository:

```bash
git clone https://github.com/AV-Lab/harbot-agri-simulation.git
cd harbot-agri-simulation
```

From the repository root:

```bash
git lfs pull
docker build -t harbot-simulation:noetic .
```

Git LFS downloads the mesh assets. Building the image does not start the simulation.

### Optional rosbridge

By default, rosbridge is **not installed or started**. To include it and start it
automatically, use this build command instead:

```bash
docker build --build-arg ENABLE_ROSBRIDGE=true -t harbot-simulation:noetic .
```

Both builds use the same image name; choose the one you need. External clients
that connect through rosbridge require the enabled build.

## Start the simulation

In a terminal on your desktop, from the repository root:

```bash
./docker/run.sh
```

This starts Gazebo, robots A/B at `a1`/`a40`, their controllers and MoveIt.
The script handles display access; Gazebo opens on your desktop.
Navigation and harvesting are started separately below.

If built with `ENABLE_ROSBRIDGE=true`, rosbridge also starts at
`ws://127.0.0.1:9090`. Compose reserves host port `9090` in either case, so keep
it free of other services.

## Run navigation or harvesting

In another host terminal, from the repository root:

```bash
docker compose exec simulation /harbot_entrypoint.sh bash
```

ROS is already sourced in this shell. Follow the [usage guide](usage.md) for
navigation or harvesting; **do not launch the simulation again**. Open another
container shell for each command that needs its own terminal.

## Stop or update

Press **Ctrl+C** in the startup terminal to stop. To remove the stopped container:

```bash
docker compose down
```

Code is copied into the image. After changing code or configuration, rebuild:

```bash
docker build -t harbot-simulation:noetic .
./docker/run.sh
```

Include `--build-arg ENABLE_ROSBRIDGE=true` again when rebuilding if you need
rosbridge. The next startup recreates the Compose container when the image changes.

PROST/RDDL is not included. Add its
[external packages](build-from-source.md#optional-integrations) to the image build
if needed.
