#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
WORLD_DIR="$PROJECT_ROOT/Мир"

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

if [[ ! -d "$WORLD_DIR/models" || ! -f "$WORLD_DIR/resources/worlds/clover_aruco.world" ]]; then
  echo "Extracted world not found: $WORLD_DIR" >&2
  echo "Run: unrar x 'Мир.rar'" >&2
  exit 1
fi

mkdir -p "$HOME/.gazebo/models"

# Делаем модели доступными через model://... для Gazebo.
for model_dir in "$WORLD_DIR/models"/*; do
  [[ -d "$model_dir" && -f "$model_dir/model.config" ]] || continue
  model_name="$(basename "$model_dir")"
  target="$HOME/.gazebo/models/$model_name"
  if [[ -L "$target" ]]; then
    rm "$target"
  elif [[ -e "$target" ]]; then
    backup="$target.backup.$(date +%Y%m%d_%H%M%S)"
    mv "$target" "$backup"
    echo "Existing model moved to backup: $backup"
  fi
  ln -s "$model_dir" "$target"
done

CLOVER_SIMULATION_PATH="$(find_clover_simulation || true)"
if [[ -z "$CLOVER_SIMULATION_PATH" ]]; then
  echo "clover_simulation package not found; models were installed only." >&2
  exit 0
fi

target_world="$CLOVER_SIMULATION_PATH/resources/worlds/clover_aruco.world"
backup_world="$target_world.before_a2025.bak"
if [[ -f "$target_world" && ! -f "$backup_world" ]]; then
  cp "$target_world" "$backup_world"
  echo "Backup created: $backup_world"
fi

cp "$WORLD_DIR/resources/worlds/clover_aruco.world" "$target_world"
echo "Installed A2025 world: $target_world"
echo "Gazebo models installed into: $HOME/.gazebo/models"
