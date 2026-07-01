#!/usr/bin/env bash
set -euo pipefail

find_clover_simulation() {
  if command -v rospack >/dev/null 2>&1; then
    local package_path
    package_path="$(rospack find clover_simulation 2>/dev/null || true)"
    if [[ -n "$package_path" && -d "$package_path" ]]; then
      echo "$package_path"
      return 0
    fi
  fi

  for candidate in \
    "$HOME/catkin_ws/src/clover/clover_simulation" \
    "$HOME/catkin_ws/install/share/clover_simulation"; do
    if [[ -d "$candidate" ]]; then
      echo "$candidate"
      return 0
    fi
  done
  return 1
}

CLOVER_SIMULATION_PATH="$(find_clover_simulation || true)"
if [[ -z "$CLOVER_SIMULATION_PATH" ]]; then
  echo "clover_simulation package not found." >&2
  exit 1
fi

target_world="$CLOVER_SIMULATION_PATH/resources/worlds/clover_aruco.world"
backup_world="$target_world.before_a2025.bak"
if [[ ! -f "$backup_world" ]]; then
  echo "Backup not found: $backup_world" >&2
  exit 1
fi

cp "$backup_world" "$target_world"
echo "Restored Clover world: $target_world"

MAP_DIR="$HOME/catkin_ws/src/clover/aruco_pose/map"
for map_name in map.txt cmit.txt; do
  target_map="$MAP_DIR/$map_name"
  backup_map="$target_map.before_a2025.bak"
  if [[ -f "$backup_map" ]]; then
    cp "$backup_map" "$target_map"
    echo "Restored ArUco map: $target_map"
  fi
done

for model_name in red green aruco_cmit_txt parquet_plane; do
  target="$HOME/.gazebo/models/$model_name"
  if [[ -L "$target" ]]; then
    link_target="$(readlink "$target")"
    case "$link_target" in
      *"/a2025/world/models/"*)
        rm "$target"
        echo "Removed A2025 model symlink: $target -> $link_target"
        ;;
    esac
  fi
done

solar_model="$HOME/.gazebo/models/solar_panel"
if [[ -L "$solar_model" ]]; then
  rm "$solar_model"
  echo "Removed old solar_panel symlink: $solar_model"
fi
