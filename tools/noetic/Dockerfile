FROM osrf/ros:noetic-desktop-full@sha256:7dbfb9576d8e6d226c31e06129a82aaab8702695f38eca2116918cb9b9308797

ARG DEBIAN_FRONTEND=noninteractive
ARG ROS_DISTRO=noetic
SHELL ["/bin/bash", "-o", "pipefail", "-c"]

# This image is a CI/SIL validation environment, never a NUC runtime artifact.
# The ROS/Gazebo closure is pinned by the base digest; test-only additions use
# exact Focal package versions.
# git is not test-only: scripts/generate_release_manifest.py shells out to it in
# source_identity() to tell a clean commit from a dirty worktree, so the release
# tooling cannot run in this image without it.
#
# liburdfdom-tools supplies check_urdf, which the URDF validation step invokes.
# noetic-desktop-full brings the urdf libraries but not that CLI.
#
# git is deliberately NOT pinned, unlike everything else here. It comes from
# focal-updates, where each security patch supersedes and removes the previous
# version, so pinning the observed 1:2.25.1-1ubuntu3.14 would build today and
# fail the moment the next patch lands. The other three are frozen Focal
# release packages, so pinning them is stable. This image is a CI/SIL validation
# environment and never a NUC runtime artifact, so tracking git security patches
# is the right trade; the ROS/Gazebo closure stays reproducible via the base
# digest.
RUN apt-get update && apt-get install -y --no-install-recommends \
      git \
      liburdfdom-tools=1.0.4+ds-2 \
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
