#!/usr/bin/env bash
set -euo pipefail

if [[ "${ROS_DISTRO:-}" != "noetic" ]]; then
  echo "source /opt/ros/noetic/setup.bash before running this script"
  exit 2
fi
for command in vcs git catkin_make catkin_test_results grep realpath python3; do
  if ! command -v "$command" >/dev/null 2>&1; then
    echo "required command is missing: $command"
    exit 2
  fi
done

if [[ $# -ne 1 ]]; then
  echo "usage: $0 /absolute/path/to/new_localization_catkin_ws"
  exit 2
fi

workspace="$(realpath "$1")"
script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
project="$(realpath "$script_dir/..")"
repository="$(realpath "$project/../..")"
if [[ ! -d "$workspace/src" || ! -f "$project/external/localization_noetic.repos" ||
      ! -d "$repository/src/wheelchair_navigation" ||
      ! -d "$repository/src/wheelchair_interfaces" ]]; then
  echo "workspace/src, project assets, or repository source packages are missing"
  exit 2
fi

vcs import --recursive "$workspace/src" \
  < "$project/external/localization_noetic.repos"

driver="$workspace/src/livox_ros_driver2"
fast_lio="$workspace/src/FAST_LIO"
global_localizer="$workspace/src/hdl_global_localization"

cp "$driver/package_ROS1.xml" "$driver/package.xml"

if ! grep -q "livox_ros_driver2/CustomMsg.h" "$fast_lio/src/laserMapping.cpp"; then
  git -C "$fast_lio" apply --check \
    "$project/external/patches/fast_lio_ros1_driver2.patch"
  git -C "$fast_lio" apply \
    "$project/external/patches/fast_lio_ros1_driver2.patch"
fi

if ! grep -q "d87b310eba15caa992d8f3fa9084f976d0f6a907" \
    "$global_localizer/CMakeLists.txt"; then
  git -C "$global_localizer" apply --check \
    "$project/external/patches/hdl_global_localization_pin_teaser.patch"
  git -C "$global_localizer" apply \
    "$project/external/patches/hdl_global_localization_pin_teaser.patch"
fi

link_project_package() {
  local package="$1"
  local source="$repository/src/$package"
  local target="$workspace/src/$package"
  if [[ "$package" == "wheelchair_lidar_localization" ]]; then
    source="$project/src/$package"
  fi
  if [[ ! -d "$source" ]]; then
    echo "project package is missing: $source"
    exit 2
  fi
  if [[ -e "$target" && ! -L "$target" ]]; then
    if [[ "$(realpath "$target")" != "$(realpath "$source")" ]]; then
      echo "refusing to replace existing $target"
      exit 2
    fi
  elif [[ -L "$target" ]]; then
    ln -sfn "$source" "$target"
  else
    ln -s "$source" "$target"
  fi
}

# Build the native localizer with its exact adapter and message ABI.  This
# keeps the deployed workspace untouched while preventing an old adapter from
# accepting a pose whose diagnostic reset epoch belongs to another sample.
for package in wheelchair_interfaces wheelchair_navigation wheelchair_lidar_localization; do
  link_project_package "$package"
done

catkin_make -C "$workspace" \
  -DROS_EDITION=ROS1 \
  -DENABLE_TEASER=ON \
  -DCMAKE_BUILD_TYPE=Release

catkin_make -C "$workspace" run_tests_wheelchair_lidar_localization
catkin_test_results "$workspace/build/test_results/wheelchair_lidar_localization"
python3 "$repository/src/wheelchair_navigation/tests/test_fast_lio_icp_unittest.py"

echo "Built and tested localization workspace. Source $workspace/devel/setup.bash"
