#!/usr/bin/env bash

set -u

usage() {
  cat <<'EOH'
Usage: cleanup_ros2_env.sh [options]

Clean up common ROS 2 / Gazebo / FastDDS leftovers for local simulation runs.

Options:
  --dry-run        Show what would be cleaned without changing anything
  --no-daemon      Do not stop ros2 daemon
  --no-shm         Do not clean FastDDS shared-memory files in /dev/shm
  --no-processes   Do not kill ROS/Gazebo related processes
  -h, --help       Show this help
EOH
}

DRY_RUN=0
CLEAN_DAEMON=1
CLEAN_SHM=1
CLEAN_PROCESSES=1

while [[ $# -gt 0 ]]; do
  case "$1" in
    --dry-run)
      DRY_RUN=1
      ;;
    --no-daemon)
      CLEAN_DAEMON=0
      ;;
    --no-shm)
      CLEAN_SHM=0
      ;;
    --no-processes)
      CLEAN_PROCESSES=0
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "Unknown option: $1" >&2
      usage >&2
      exit 1
      ;;
  esac
  shift
done

log() {
  printf '[cleanup] %s\n' "$*"
}

run_cmd() {
  if [[ $DRY_RUN -eq 1 ]]; then
    printf '[dry-run] '
    printf '%q ' "$@"
    printf '\n'
    return 0
  fi

  "$@"
}

PATTERNS=(
  'ros2 launch'
  'ros2 topic'
  'ros2 bag'
  'gz sim'
  'ign gazebo'
  'gazebo'
  'nav2_container'
  'component_container'
  'component_container_isolated'
  'lifecycle_manager'
  'pointlio_mapping'
  'loam_interface'
  'sensor_scan_generation'
  'sync_slam_toolbox'
  'slam_toolbox'
  'map_saver_server'
  'pointcloud_to_laserscan'
  'fake_vel_transform'
  'ign_sim_pointcloud_tool_node'
  'simple_competition_1v1.py'
  'rmua19_robot_base'
  'rmu_gazebo_simulator'
  'pb2025_nav_bringup'
)

list_target_processes() {
  local ps_out
  ps_out="$(ps -eo pid=,ppid=,cmd=)"

  while IFS= read -r line; do
    [[ -z "$line" ]] && continue
    [[ "$line" == *"cleanup_ros2_env.sh"* ]] && continue

    for pattern in "${PATTERNS[@]}"; do
      if [[ "$line" == *"$pattern"* ]]; then
        printf '%s\n' "$line"
        break
      fi
    done
  done <<< "$ps_out"
}

kill_target_processes() {
  local matches pids
  matches="$(list_target_processes)"

  if [[ -z "$matches" ]]; then
    log "No matching ROS/Gazebo processes found."
    return 0
  fi

  log "Matching processes:"
  printf '%s\n' "$matches"

  pids="$(printf '%s\n' "$matches" | awk '{print $1}' | tr '\n' ' ')"

  if [[ -z "${pids// }" ]]; then
    return 0
  fi

  log "Sending SIGTERM to matching processes."
  run_cmd kill $pids

  if [[ $DRY_RUN -eq 0 ]]; then
    sleep 2
    local survivors
    survivors="$(list_target_processes)"
    if [[ -n "$survivors" ]]; then
      log "Processes still alive after SIGTERM, sending SIGKILL."
      printf '%s\n' "$survivors"
      # shellcheck disable=SC2086
      run_cmd kill -9 $(printf '%s\n' "$survivors" | awk '{print $1}' | tr '\n' ' ')
    fi
  fi
}

stop_ros2_daemon() {
  if ! command -v ros2 >/dev/null 2>&1; then
    log "ros2 command not found, skipping ros2 daemon stop."
    return 0
  fi

  log "Stopping ros2 daemon."
  run_cmd ros2 daemon stop
}

clean_fastdds_shm() {
  local entries=()

  while IFS= read -r path; do
    [[ -n "$path" ]] && entries+=("$path")
  done < <(find /dev/shm -maxdepth 1 \( -name 'fastrtps*' -o -name 'sem.fastrtps*' \) -print 2>/dev/null)

  if [[ ${#entries[@]} -eq 0 ]]; then
    log "No FastDDS shared-memory files found in /dev/shm."
    return 0
  fi

  log "FastDDS shared-memory files:"
  printf '%s\n' "${entries[@]}"

  if [[ $DRY_RUN -eq 1 ]]; then
    return 0
  fi

  local failed=0
  local path
  for path in "${entries[@]}"; do
    if ! unlink "$path"; then
      log "Failed to remove $path"
      failed=1
    fi
  done

  if [[ $failed -eq 0 ]]; then
    log "FastDDS shared-memory files removed."
  else
    log "Some FastDDS files could not be removed. Check ownership or active processes."
    return 1
  fi
}

main() {
  log "Start cleanup."

  if [[ $CLEAN_PROCESSES -eq 1 ]]; then
    kill_target_processes
  else
    log "Skipping process cleanup."
  fi

  if [[ $CLEAN_DAEMON -eq 1 ]]; then
    stop_ros2_daemon
  else
    log "Skipping ros2 daemon cleanup."
  fi

  if [[ $CLEAN_SHM -eq 1 ]]; then
    clean_fastdds_shm
  else
    log "Skipping FastDDS shared-memory cleanup."
  fi

  log "Cleanup finished."
}

main
