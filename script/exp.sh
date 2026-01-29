#!/bin/bash

if [ "$#" -lt 1 ]; then
  echo "Usage: $0 <input_file> [max_tasks default=4]"
  exit 1
fi

input_file="$1"
max_tasks=${2:-4}

sh_dir=$(dirname $(readlink -f "$0"))

py_script="$sh_dir/bin/BV-Parti.py"
partitioner="$sh_dir/bin/binaries/partitioner-bin"
base_solver="$sh_dir/bin/binaries/bitwuzla-0.8.0-bin"
output_dir="$sh_dir/test-instances/test-output-dir/"

if [ ! -d "$output_dir" ]; then
  mkdir -p "$output_dir"
fi

solver_params=(
  "--file" "$input_file"
  "--output-dir" "$output_dir"
  "--partitioner" "$partitioner"
  "--solver" "$base_solver"
  "--max-running-tasks" "$max_tasks"
  "--time-limit" "1200"
)

# echo "python3 $py_script ${solver_params[@]}"

python3 $py_script "${solver_params[@]}"
