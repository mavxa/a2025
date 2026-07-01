#!/usr/bin/env bash
set -euo pipefail

MAP_DIR="${1:-$HOME/catkin_ws/src/clover/aruco_pose/map}"
MAP_FILE="$MAP_DIR/cmit.txt"
BACKUP_FILE="$MAP_FILE.before_a2025_10x10.bak"

if [[ ! -f "$BACKUP_FILE" ]]; then
  echo "Backup not found: $BACKUP_FILE" >&2
  exit 1
fi

cp "$BACKUP_FILE" "$MAP_FILE"
echo "Restored ArUco map: $MAP_FILE"
echo "Restart Clover/Gazebo after this."
