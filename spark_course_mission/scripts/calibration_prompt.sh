#!/usr/bin/env bash
set -euo pipefail
script_dir="$(rospack find spark_course_mission)/scripts"
exec gnome-terminal -- bash "$script_dir/calibration_confirm.sh"
