#!/usr/bin/env bash
set -u

shorten() {
  local text="$1"
  text="${text//$'\n'/ }"
  printf '%s' "${text:0:120}"
}

check_cmd() {
  local label="$1"
  local cmd="$2"
  if command -v "$cmd" >/dev/null 2>&1; then
    local path
    path="$(command -v "$cmd")"
    local raw version
    case "$cmd" in
      roscore|roslaunch|rosparam)
        raw="$($cmd --help 2>&1)"
        version="$(shorten "$raw")"
        ;;
      gazebo)
        raw="$($cmd --version 2>&1)"
        version="$(shorten "$raw")"
        ;;
      catkin_make)
        raw="$($cmd --version 2>&1)"
        version="$(shorten "$raw")"
        ;;
      xacro)
        raw="$($cmd --version 2>&1)"
        version="$(shorten "$raw")"
        ;;
      *)
        version="available"
        ;;
    esac
    printf 'OK      %-18s %s :: %s\n' "$label" "$path" "$version"
    return 0
  fi
  printf 'MISSING %-18s command not found: %s\n' "$label" "$cmd"
  return 1
}

missing=0
printf 'ROS1 Noetic / Gazebo Classic environment check\n'
printf 'Target OS: Ubuntu 20.04, ROS noetic, Gazebo Classic 11\n'
printf 'Current ROS_DISTRO: %s\n' "${ROS_DISTRO:-<unset>}"
printf '\n'

check_cmd roscore roscore || missing=$((missing + 1))
check_cmd roslaunch roslaunch || missing=$((missing + 1))
check_cmd rosparam rosparam || missing=$((missing + 1))
check_cmd catkin_make catkin_make || missing=$((missing + 1))
check_cmd xacro xacro || missing=$((missing + 1))
check_cmd gazebo gazebo || missing=$((missing + 1))

printf '\n'
if [ "$missing" -eq 0 ]; then
  printf 'Summary: all required ROS1/Gazebo commands were found.\n'
else
  printf 'Summary: %s command(s) missing. Install ROS1 Noetic/Gazebo Classic 11 before simulator runtime validation.\n' "$missing"
fi

exit 0
