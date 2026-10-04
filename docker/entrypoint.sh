#!/bin/bash
set -e

# The workspace is already built into the image.
source /opt/ros/noetic/setup.bash
source /opt/harbot_ws/devel/setup.bash
exec "$@"
