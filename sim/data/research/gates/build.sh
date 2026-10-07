#!/usr/bin/env bash
# Build the plugins the gates load, into $GATES_SCRATCH (default <tmp>/rover_gates), with the pixi env's
# compilers and Gazebo: the drivetrain prototype with load_source (patch_proto.py), the fly camera
# prototype and HmCheck (check_heightmap.py). Usage: sim/data/research/gates/build.sh
set -euo pipefail
here="$(cd "$(dirname "$0")" && pwd)"
repo="$(cd "$here/../../../.." && pwd)"
scratch="${GATES_SCRATCH:-$(python -c 'import tempfile; print(tempfile.gettempdir())')/rover_gates}"
prefix="${CONDA_PREFIX:-$(dirname "$(dirname "$(command -v python)")")}"

build() {  # name, source file, target, extra link libraries, extra include directory
  local dir="$scratch/$1"
  mkdir -p "$dir"
  cat > "$dir/CMakeLists.txt" <<EOF
cmake_minimum_required(VERSION 3.20)
project($3 CXX)
set(CMAKE_CXX_STANDARD 17)
find_package(gz-sim8 REQUIRED)
find_package(gz-plugin2 REQUIRED COMPONENTS register)
find_package(gz-transport13 REQUIRED)
find_package(gz-rendering8 REQUIRED)
find_package(gz-common5 REQUIRED COMPONENTS geospatial)
add_library($3 SHARED $2)
target_include_directories($3 PRIVATE $5)
target_link_libraries($3 PRIVATE gz-sim8::gz-sim8 gz-plugin2::register $4)
target_compile_options($3 PRIVATE -Wall -Wextra)
EOF
  cmake -S "$dir" -B "$dir/build" -DCMAKE_BUILD_TYPE=Release -DCMAKE_PREFIX_PATH="$prefix" > "$dir/cmake.log"
  cmake --build "$dir/build" -j
}

mkdir -p "$scratch/proto" "$scratch/flycam" "$scratch/hmcheck"
python "$here/patch_proto.py" "$repo/sim/data/research/drive/prototype/cpp/rover_drive.cpp" "$scratch/proto/rover_drive.cpp"
build proto rover_drive.cpp RoverDrive gz-transport13::gz-transport13 "$here"
cp "$repo/sim/data/research/flycam/prototype/flycam_proto.cpp" "$scratch/flycam/"
build flycam flycam_proto.cpp FlyCamProto "gz-rendering8::gz-rendering8 gz-common5::geospatial" "$here"
cp "$here/hm_check.cpp" "$scratch/hmcheck/"
build hmcheck hm_check.cpp HmCheck gz-common5::geospatial "$repo/sim/plugins"
echo "built into $scratch"
