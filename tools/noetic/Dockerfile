FROM osrf/ros:noetic-desktop-full@sha256:7dbfb9576d8e6d226c31e06129a82aaab8702695f38eca2116918cb9b9308797

ARG DEBIAN_FRONTEND=noninteractive
ARG ROS_DISTRO=noetic
SHELL ["/bin/bash", "-o", "pipefail", "-c"]

# This image is a CI/SIL validation environment, never a NUC runtime artifact.
# The ROS/Gazebo closure is pinned by the base digest; test-only additions use
# exact Focal package versions.
RUN apt-get update && apt-get install -y --no-install-recommends \
      python3-jsonschema=3.2.0-0ubuntu2 \
      python3-pytest=4.6.9-1 \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ws
COPY . /ws
RUN apt-get update \
    && rosdep update --rosdistro=${ROS_DISTRO} \
    && rosdep install --from-paths src --ignore-src --rosdistro=${ROS_DISTRO} -y \
    && rm -rf /var/lib/apt/lists/*

ENV ROS_DISTRO=noetic
ENV GAZEBO_MODEL_DATABASE_URI=file:///usr/share/gazebo-11/models
ENV GAZEBO_MODEL_PATH=/usr/share/gazebo-11/models
CMD ["bash"]
