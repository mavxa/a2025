#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
WORLD_DIR="$PROJECT_ROOT/world"

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

if [[ ! -d "$WORLD_DIR/models" ]]; then
  echo "Extracted world not found: $WORLD_DIR" >&2
  echo "Expected directory: $PROJECT_ROOT/world" >&2
  exit 1
fi

mkdir -p "$HOME/.gazebo/models"

# Делаем доступными только модели станций. ArUco/parquet из архива не ставим:
# они могут не совпадать со штатной CMIT-картой 10x10 из образа Clover.
for model_name in red green; do
  model_dir="$WORLD_DIR/models/$model_name"
  [[ -d "$model_dir" && -f "$model_dir/model.config" ]] || continue
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

# Если раньше этим скриптом был поставлен неправильный aruco_cmit_txt из архива,
# убираем symlink, чтобы Gazebo снова использовал штатную модель 10x10 из образа.
for model_name in aruco_cmit_txt parquet_plane; do
  target="$HOME/.gazebo/models/$model_name"
  if [[ -L "$target" ]]; then
    link_target="$(readlink "$target")"
    case "$link_target" in
      "$WORLD_DIR"/*)
        rm "$target"
        echo "Removed archive model symlink: $target -> $link_target"
        ;;
    esac
  fi
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

if [[ -f "$backup_world" ]]; then
  cp "$backup_world" "$target_world"
fi

python3 - "$target_world" <<'PY'
from pathlib import Path
import sys

path = Path(sys.argv[1])
content = path.read_text(encoding="utf-8")
start = "    <!-- A2025 stations start -->"
end = "    <!-- A2025 stations end -->"
if start in content and end in content:
    before, rest = content.split(start, 1)
    _, after = rest.split(end, 1)
    content = before + after

block = f"""
{start}
    <include>
      <uri>model://red</uri>
      <name>a2025_red_station</name>
      <pose>1 5 0 0 0 0</pose>
    </include>
    <include>
      <uri>model://green</uri>
      <name>a2025_green_station</name>
      <pose>5 2 0 0 0 0</pose>
    </include>
{end}
"""
marker = "</world>"
if marker not in content:
    raise SystemExit(f"Cannot find {marker} in {path}")
content = content.replace(marker, block + "  " + marker, 1)
path.write_text(content, encoding="utf-8")
PY

echo "Installed A2025 stations into existing Clover 10x10 world: $target_world"
echo "Gazebo models installed into: $HOME/.gazebo/models"
