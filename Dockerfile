# Ubuntu 20.04, ROS Noetic and Python 3.8; pin the base image rather than a moving tag.
FROM ros:noetic-ros-base-focal@sha256:72b8bc59035dc0a5b8e07aae28c16caa84192971d72d207c72ed734fb1d5e97d

SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ARG DEBIAN_FRONTEND=noninteractive
ARG ROSDISTRO_REVISION=bee34a7e7b843546acffd6b40ffa230472d0f946
ARG GAZEBO_MODELS_REVISION=8163eb4b5e7e21985c6591d1c0bfb56468c0093f
ARG PROST_ROS_REVISION=72028b15f452f9f1fd39d07f7eee85807fa3f038
ARG ETF_SERVICES_REPO=https://github.com/etfrobotics/rbkairos_etf_services.git
ARG ETF_SERVICES_REVISION=SIM-integration
ARG BUILD_JOBS=4

ENV ROSDISTRO_INDEX_URL=https://raw.githubusercontent.com/ros/rosdistro/${ROSDISTRO_REVISION}/index-v4.yaml
ENV GAZEBO_MODEL_PATH=/opt/gazebo_models
ENV GAZEBO_MODEL_DATABASE_URI=""
WORKDIR /opt/harbot_ws

RUN apt-get update && apt-get install -y --no-install-recommends \
        curl git python3-catkin-tools python3-pip \
        bison flex libbdd-dev libz3-dev \
    && rm -rf /var/lib/apt/lists/*

# PROST planning stack: the PROST ROS bridge with its planner, and the RDDL action services.
RUN git clone https://github.com/etfrobotics/prost_ros.git src/prost_ros \
    && git -C src/prost_ros checkout "${PROST_ROS_REVISION}" \
    && git clone "${ETF_SERVICES_REPO}" src/rbkairos_etf_services \
    && git -C src/rbkairos_etf_services checkout "${ETF_SERVICES_REVISION}" \
    && cd src/prost_ros/prost && ./build.py \
    && pip3 install --no-cache-dir pyRDDLGym==2.7

# Cache dependency installation separately from changes to scripts and models.
COPY harbot_control/package.xml src/harbot-agri-simulation/harbot_control/package.xml
COPY harbot_description/package.xml src/harbot-agri-simulation/harbot_description/package.xml
COPY harbot_harvest/package.xml src/harbot-agri-simulation/harbot_harvest/package.xml
COPY harbot_moveit/package.xml src/harbot-agri-simulation/harbot_moveit/package.xml
COPY harbot_navigation/package.xml src/harbot-agri-simulation/harbot_navigation/package.xml
COPY harbot_simulation/package.xml src/harbot-agri-simulation/harbot_simulation/package.xml
COPY simulation_plugins/gazebo_ros_link_attacher/package.xml src/harbot-agri-simulation/simulation_plugins/gazebo_ros_link_attacher/package.xml
COPY simulation_plugins/greenhouse_fruit_detachment/package.xml src/harbot-agri-simulation/simulation_plugins/greenhouse_fruit_detachment/package.xml
COPY simulation_plugins/realsense_d435_gazebo/package.xml src/harbot-agri-simulation/simulation_plugins/realsense_d435_gazebo/package.xml
COPY simulation_plugins/roboticsgroup_gazebo_plugins/package.xml src/harbot-agri-simulation/simulation_plugins/roboticsgroup_gazebo_plugins/package.xml

# STOMP is an optional MoveIt planner and is not part of this image.
RUN sed -i "s|ros/rosdistro/master|ros/rosdistro/${ROSDISTRO_REVISION}|g" \
        /etc/ros/rosdep/sources.list.d/20-default.list \
    && rosdep update --rosdistro noetic --include-eol-distros \
    && apt-get update \
    && rosdep install --from-paths src --ignore-src --rosdistro noetic -y \
        --skip-keys="stomp_moveit" \
    && rm -rf /var/lib/apt/lists/*

# The world uses Gazebo's sun model. Include it now; no model download at startup.
RUN mkdir -p /opt/gazebo_models/sun \
    && for model_file in model.config model.sdf model-1_2.sdf model-1_3.sdf model-1_4.sdf; do \
        curl -fsSL --retry 3 \
            "https://raw.githubusercontent.com/osrf/gazebo_models/${GAZEBO_MODELS_REVISION}/sun/${model_file}" \
            -o "/opt/gazebo_models/sun/${model_file}" || exit 1; \
    done

COPY . src/harbot-agri-simulation/
RUN if grep -RIl --include='*.obj' '^version https://git-lfs.github.com/spec/v1$' src; then \
        echo "Missing mesh assets: run git lfs pull before building." >&2; exit 1; \
    fi \
    && source /opt/ros/noetic/setup.bash \
    && catkin config --extend /opt/ros/noetic --cmake-args -DCMAKE_BUILD_TYPE=Release \
    && catkin build --no-status --jobs "${BUILD_JOBS}" --parallel-packages 2

COPY docker/entrypoint.sh /harbot_entrypoint.sh
RUN chmod +x /harbot_entrypoint.sh

ENTRYPOINT ["/harbot_entrypoint.sh"]
CMD ["roslaunch", "/opt/harbot_ws/src/harbot-agri-simulation/docker/simulation.launch"]
