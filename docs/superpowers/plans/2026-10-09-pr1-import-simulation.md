# PR 1: Import the simulation into Harvard-HURC/Rover-Software Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** The branch `import/simulation` of the team's repository holds this repository's whole history (without `papers/`) merged with the team's `main`, restructured into the packages `rover_sim` and `rover_control` beside the team's `rover_description`, built by `colcon build`, with one pixi environment for three platforms, `pixi run fetch-data`, CI and the documentation split, and with no change in behaviour.

**Architecture:** Design spec `docs/superpowers/specs/2026-10-09-rover-software-integration-design.md`, sections 9.1 and 9.2 (row 1). The history import is a `git filter-branch` copy of this repository merged with the team's `main` (`--allow-unrelated-histories`). The restructuring is pure `git mv` renames in one commit, then small commits that adapt build files, paths, pixi, tests and documents. The simulation keeps running from the source tree as today; only its plugin build directory moves to colcon's `<workspace>/build/rover_sim`, which `gzenv.py`, the one place that sets Gazebo's paths, points at.

**Tech Stack:** git, pixi 0.81 (RoboStack Jazzy channel), colcon and ament_cmake, CMake, Gazebo Harmonic (gz-sim 8.10, gz-rendering 8.2.2), Python 3.12 unittest, GoogleTest, GitHub Actions (prefix-dev/setup-pixi), actionlint.

---

## Places and rules

| Name | Path |
|---|---|
| SOURCE | `/Users/alarion239/Desktop/Rover` (this repository; `main`, no remote) |
| WS | `/Users/alarion239/Desktop/hurc_ws` (the colcon workspace) |
| SRC | `/Users/alarion239/Desktop/hurc_ws/src` (the team's clone: `origin` = `https://github.com/Harvard-HURC/Rover-Software`, `main` at `408bc99`) |
| T | `/private/tmp/claude-502/rover-pr1` (temporary clones and scripts; deleted in Task 19). If the implementing session lists a scratchpad directory, use `<scratchpad>/rover-pr1` as T in every command instead: the sandbox may refuse writes elsewhere |
| pixi | `/Users/alarion239/.pixi/bin/pixi` (0.81) |

- **Never push, never create anything on GitHub, never open a pull request.** `git fetch` and read-only `gh` calls are fine. The user approves each push separately.
- **SOURCE is read-only** for this plan: no commit, no file change there (the data rasters are symlinked from it). Commands that use SOURCE's environment run `pixi run --frozen --manifest-path /Users/alarion239/Desktop/Rover/pixi.toml ...`: without `--frozen`, pixi may rewrite SOURCE's `pixi.lock`.
- Every commit message ends with the line `Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>`; the author is the configured `Alexander Belotserkovtsev <amb30239@gmail.com>`.
- Every Gazebo run outside the test suite gets `GZ_IP=127.0.0.1` and its own `GZ_PARTITION` (the commands below set them). The suite sets its own partitions. Stop only processes you started; ports 8765 and 8766 belong to an unrelated app.
- Shell state does not persist between tool calls: every command block sets the variables it uses and starts with `cd`.
- The Bash tool stops foreground commands after 10 minutes. Run `pixi install`, the full test suite and other long commands in the background (`run_in_background`) with their output in a log under `$T`, and wait for them.
- Disk: about 20 GB free. Do not copy the rasters (symlink them), build no Docker image, keep one pixi environment in SRC (pixi hardlinks from its shared cache), delete `$T` at the end.
- Style: match SOURCE's `sim/README.md` and the spec: short declarative sentences, units in brackets, no marketing words; code at 120 columns as in `rover_sim/`.

## Decisions this plan makes

The spec leaves these open; each is final for PR 1 and stated in the commit or document that implements it.

1. **Research split.** `sim/data/research` has 543 tracked files. Four are read or written by code and stay at `rover_sim/data/research/` (so no code path changes): `terrain_targets.json` (urc/realism.py, tools/terrain_targets.py, test_landscape, test_urc_terrain, the sim-worlds inputs), `mdrs_terrain_measurements.json` (tools/make_relief_swatches.py), `realism_report.json` and `realism_contact_sheet.jpg` (tools/realism_report.py writes them; it stores the sheet's path relative to `rover_sim/`, so they must stay under it). The other 539 move to `docs/research/`. That includes `gates/` and `gates.json`: no code or test reads them (grep: only comments cite them); they are research scripts that themselves locate the repository by `parents[4] / "sim"`, so they run only on the tree before the move wherever they live. `docs/research/README.md` says how to run them (check out the commit before the move).
2. **Plugin directory.** colcon builds `rover_sim` in `<workspace>/build/rover_sim`; the plugins land there. `gzenv.BUILD_DIR` becomes that directory (the patched media stay in it too, as before in `sim/build`). The plugins are also installed to `install/rover_sim/lib/rover_sim` with an environment hook that adds them to `GZ_SIM_SYSTEM_PLUGIN_PATH` in a sourced workspace, for ROS launch files.
3. **Linters.** The team's skeleton runs `ament_lint_auto` (flake8, pep257, uncrustify, cpplint...) under `colcon test`. The imported code keeps its own style (120 columns, its docstrings), which those linters reject file by file, so `rover_sim` drops the lint block and `rover_control` does not get one. `rover_description` keeps the team's block.
4. **fetch-data.** No script in SOURCE rebuilds most rasters: only `tools/fetch_dem.py` exists, for one 3DEP request, and it refuses without `--force` on a fresh clone because the tracked `.json` exists. The 0.5 m lidar DEMs were made from USGS point clouds and tiles by processing that is not in the repository. So `tools/fetch_data.py` with a manifest `data/rasters.json` (SHA-256 of each file and of its content): the six rasters whose provenance lists their ImageServer `exportImage` requests are rebuilt from those official requests and kept only if their content matches; the other four (the two 0.5 m lidar DEMs and the lidar source map, made by processing not in the repository, among them `route_area_lidar_0p5m.tif`, which `urc_autonomy` needs; and the NAIP 2021 route mosaic, whose provenance gives only a URL template) and any official source that fails or changes come from the team's copy, a GitHub release `data-2026-10-06` of Harvard-HURC/Rover-Software. Creating that release needs the user's approval (Task 19 prepares the command); until it exists, fetch-data on a fresh clone cannot get the derived rasters and CI fails at its fetch step.
5. **pixi.** The team's `pixi.lock` is the base: it already holds every package the simulation uses on all three platforms, and on osx-arm64 the physics and rendering packages are the versions SOURCE runs today (gz-sim 8.10.0, gz-physics 7.5.0, dartsim 6.19.4, ogre-next 2.3.3, gz-rendering 8.2.2). The simulation's direct dependencies are listed under `[target.unix.dependencies]` (the team's lock resolves the Gazebo packages on `win-64` too, through `ros2-ros-gz-sim`, and colcon, ament, Eigen, GoogleTest and `vs2022_win-64` are there; the simulation just does not run there); on `win-64` the `build` task builds `rover_description` and `rover_control` only, without tests (the simulation runs in WSL2 there, spec section 7).
6. **Compiler.** The team's `ros-dev-tools` brings conda's clang 21 (`cxx-compiler`, whose activation CMake may follow); SOURCE built the plugins and the driver with Apple's `/usr/bin/c++` (its `CMakeCache.txt`). Task 5 Step 3 records which compiler colcon's CMake took; the full suite on the new build is the check (Task 12).
7. **Paths in prose.** The move commit is pure renames (100 % similarity, so `git log --follow` and blame work). Afterwards only paths that run or that a user sees at run time change (build directory, tasks, help texts, skip reasons, one station message, `link_data.sh`). Comments and docstrings that say `sim/...` stay: changing them would touch `gen_model.py` (PR 2), the tracked generated models and the media caches keyed on `urc/` sources. `rover_sim/README.md` tells readers how to read them.
8. **Rendering tests.** A small helper `tests/gpu.py` decides whether this machine can render (every Mac; Linux with `/dev/dri/renderD*`; WSL2's `/dev/dxg`; `ROVER_RENDERING=1|0` overrides). The 27 rendering tests (`test_render.Render`, `.DigCues`, `.Ruts`, `test_fly_camera.Rendering`, `test_urc_sim.Worlds.test_camera_sees_the_start_post`) skip without a GPU and print why.

## File structure

Created:
- `rover_sim/env-hooks/rover_sim.dsv.in`: adds `lib/rover_sim` to `GZ_SIM_SYSTEM_PLUGIN_PATH` in a sourced install.
- `rover_sim/tools/fetch_data.py`: `pixi run fetch-data`.
- `rover_sim/data/rasters.json`: the raster manifest.
- `rover_sim/tests/gpu.py`: whether rendering tests can run here.
- `rover_sim/tests/test_fetch_data.py`, `rover_sim/tests/test_launch_files.py`: tests.
- `rover_control/package.xml`, `rover_control/README.md`.
- `docs/COLCON_IGNORE`: keeps colcon out of the research prototypes' `CMakeLists.txt`.
- `docs/sim/design-notes.md`, `docs/sim/gazebo-lessons.md`: split from the manual.
- `docs/research/README.md`.
- `.github/workflows/ci.yml`.

Moved (pure renames): `sim/*` to `rover_sim/*` (except `CMakeLists.txt`, folded into the team's file, and `README.md`, which becomes `docs/sim/manual.md`); 539 research files to `docs/research/`; `driver/` to `rover_control/`.

Modified: `pixi.toml`, `pixi.lock`, `.gitignore`, `README.md`, `rover_sim/CMakeLists.txt`, `rover_sim/package.xml`, `rover_sim/README.md`, `rover_sim/launch/gazebo.launch.py`, `rover_control/CMakeLists.txt`, `rover_sim/gzenv.py`, `rover_sim/station/link.py`, `rover_sim/tools/gz_media.py`, `rover_sim/tools/render_map.py`, `rover_sim/tools/link_data.sh`, `rover_sim/tests/test_foundations.py`, `rover_sim/tests/test_landscape.py`, `rover_sim/tests/test_render.py`, `rover_sim/tests/test_fly_camera.py`, `rover_sim/tests/test_urc_sim.py`, `docs/sim/manual.md`, `docs/superpowers/specs/2026-10-06-urc-realism-design.md`.

Unchanged on purpose: `rover_sim/gen_model.py`, the rover model, the drive interface, the station's cmd_vel path (PRs 2 and 3), all C++ and Python code of `rover_control`.

---

## Part A: history import and layout

### Task 1: A copy of SOURCE's history without `papers/`

**Files:** none in SRC; a temporary clone `$T/rover-filtered`.

- [ ] **Step 1: Clone SOURCE's `main` into `$T` and drop the remote**

```bash
T=/private/tmp/claude-502/rover-pr1
rm -rf "$T" && mkdir -p "$T"
git clone --no-local --single-branch --branch main /Users/alarion239/Desktop/Rover "$T/rover-filtered"
cd "$T/rover-filtered" && git remote remove origin && git rev-list --count main
```

Expected: the clone succeeds and prints the commit count of SOURCE (92 on 2026-10-09, with this plan's commit and its critic's; more if the plan was committed again).

- [ ] **Step 2: Remove `papers/` from every commit**

```bash
T=/private/tmp/claude-502/rover-pr1
cd "$T/rover-filtered" && FILTER_BRANCH_SQUELCH_WARNING=1 git filter-branch \
  --index-filter 'git rm -r --cached --ignore-unmatch papers' --prune-empty -- --all
```

Expected: `Ref 'refs/heads/main' was rewritten`. Only the baseline commit (`7e93fd7`) had `papers/`, together with 616 other files, so no commit is pruned.

- [ ] **Step 3: Check the copy**

```bash
T=/private/tmp/claude-502/rover-pr1; S=/Users/alarion239/Desktop/Rover; F="$T/rover-filtered"
git -C "$F" log --oneline main -- papers | wc -l
[ "$(git -C "$S" rev-list --count main)" = "$(git -C "$F" rev-list --count main)" ] && echo "same count"
for d in sim driver docs; do [ "$(git -C "$S" rev-parse main:$d)" = "$(git -C "$F" rev-parse main:$d)" ] && echo "$d same tree"; done
diff <(git -C "$S" log --format='%an%x09%ae%x09%ad%x09%cn%x09%ce%x09%cd%x09%B' main) \
     <(git -C "$F" log --format='%an%x09%ae%x09%ad%x09%cn%x09%ce%x09%cd%x09%B' main) && echo "authors, dates, messages same"
git -C "$F" ls-tree --name-only main
```

Expected:
```
0
same count
sim same tree
driver same tree
docs same tree
authors, dates, messages same
.gitattributes
.gitignore
.pixi
docs
driver
pixi.lock
pixi.toml
sim
```

### Task 2: Merge the team's `main` into that history on `import/simulation`

**Files:** SRC: `pixi.toml`, `pixi.lock`, `.gitignore` (conflict resolutions in the merge commit).

- [ ] **Step 1: Check the team's `main`**

```bash
cd /Users/alarion239/Desktop/hurc_ws/src && git status --short && git fetch origin && git rev-parse --short origin/main
```

Expected: a clean tree and `408bc99`. If `origin/main` moved, stop and tell the user: the conflicts below were worked out for `408bc99`.

- [ ] **Step 2: Bring in the filtered history as the branch and start the merge**

```bash
T=/private/tmp/claude-502/rover-pr1
cd /Users/alarion239/Desktop/hurc_ws/src
git fetch "$T/rover-filtered" main
git checkout -b import/simulation FETCH_HEAD
git merge --allow-unrelated-histories --no-commit origin/main
```

Expected: conflicts in exactly three files, `pixi.toml` and `.gitignore` (add/add) and `pixi.lock` (`warning: Cannot merge binary files: pixi.lock`), then `Automatic merge failed`. `README.md`, `rover_description/` and `rover_sim/` come in without conflict; `.gitattributes` is the same on both sides; `.pixi/config.toml` comes from the simulation's side.

- [ ] **Step 3: Resolve `pixi.lock` with the team's lock**

```bash
cd /Users/alarion239/Desktop/hurc_ws/src && git checkout --theirs pixi.lock && git add pixi.lock
```

- [ ] **Step 4: Resolve `.gitignore`: the team's file plus the simulation's rules (still on `sim/`)**

```bash
cd /Users/alarion239/Desktop/hurc_ws/src && git show origin/main:.gitignore > .gitignore && cat >> .gitignore <<'EOF'

# Python bytecode caches
__pycache__/

# generated URC mission worlds, test courses and their models (pixi run sim-worlds; deterministic), and their
# rendered maps (pixi run sim-maps)
sim/models/urc_*/
sim/worlds/urc_*
sim/worlds/proving_ground.*
sim/worlds/*_map.*
# world copies the tests run (worldfiles.temp_sdf), left behind by a killed run
sim/worlds/tmp*.sdf

# Large source rasters (re-fetchable from their .json provenance; worktrees link them with sim/tools/link_data.sh)
sim/data/**/*.tif
sim/data/research/render/prototype/textures/

# agent worktrees and local Claude state
.claude/
EOF
git add .gitignore
```

The simulation's other rules are covered by the team's template (`build/` anywhere, `.DS_Store`, `.idea/`, `*.pyc`, `.pixi/*`).

- [ ] **Step 5: Resolve `pixi.toml`: the team's workspace and dependencies plus the simulation's tasks (still on `sim/`)**

Write `pixi.toml`:

```toml
[workspace]
authors = ["Beckett O'Brien <44044350+BeckettOBrien@users.noreply.github.com>", "Alexander Belotserkovtsev <amb30239@gmail.com>"]
channels = ["https://prefix.dev/robostack-jazzy"]
name = "src"
platforms = ["osx-arm64", "win-64", "linux-64"]
version = "0.1.0"

[tasks]
driver-build = "cmake -S driver -B driver/build -DCMAKE_BUILD_TYPE=Release && cmake --build driver/build -j"
driver-test = { cmd = "ctest --test-dir driver/build --output-on-failure", depends-on = ["driver-build"] }
driver-viz = { cmd = "python driver/tools/viz.py", depends-on = ["driver-build"] }
# The patched gz-rendering media (sky, haze, terrain roughness) in sim/build. Soft: without it Gazebo uses its stock media.
sim-media = "python sim/tools/gz_media.py || echo 'sim-media: no patched media, Gazebo uses its stock media'"
sim-build = { cmd = "cmake -S sim -B sim/build -DCMAKE_BUILD_TYPE=Release && cmake --build sim/build -j", depends-on = ["sim-media"] }
sim-model = "python sim/gen_model.py"
sim-relief = { cmd = "python sim/tools/make_relief_swatches.py", inputs = ["sim/tools/make_relief_swatches.py", "sim/urc/dem.py", "sim/urc/landscape.py", "sim/urc/terrain.py", "sim/data/dem/*_0p5m*", "sim/data/research/mdrs_terrain_measurements.json"], outputs = ["sim/data/relief/*.npz"] }
sim-worlds = { cmd = "python sim/gen_worlds.py", depends-on = ["sim-model"], inputs = ["sim/gen_worlds.py", "sim/gen_model.py", "sim/urc/**/*.py", "sim/data/dem/*", "sim/data/imagery/*_naip2024.*", "sim/data/imagery/naip2021_far_60km.*", "sim/data/relief/*", "sim/data/soils/*", "sim/data/research/terrain_targets.json"], outputs = ["sim/worlds/urc_*.sdf", "sim/worlds/urc_*.json", "sim/worlds/proving_ground.sdf", "sim/worlds/proving_ground.json", "sim/models/urc_*/**"] }
# Orthophoto maps of the worlds for the station's Map view (needs the GPU, so not part of sim-test).
sim-maps = { cmd = "python sim/tools/render_map.py", depends-on = ["sim-build", "sim-model", "sim-worlds"] }
sim = { cmd = "bash sim/run.sh", depends-on = ["sim-build", "sim-model", "sim-worlds"] }
drive = { cmd = "python sim/station/__main__.py", depends-on = ["sim-build", "sim-model", "sim-worlds"] }
# Every world as a tile on one page: a click starts it with its driver station, Exit stops it.
launcher = { cmd = "python sim/launcher/__main__.py", depends-on = ["sim-build", "sim-model", "sim-worlds"] }
referee = "python sim/referee.py"
sim-bridge = "ros2 run ros_gz_bridge parameter_bridge --ros-args -p config_file:=sim/bridge.yaml"
sim-test = { cmd = "python -m unittest discover -s sim/tests -v", depends-on = ["sim-build", "sim-model", "sim-worlds"] }
# The performance budgets (physics speed, start-up, memory, the drivetrain's cost against DiffDrive): slow, so opt-in.
sim-perf = { cmd = "python -m unittest -v test_perf test_drivetrain.Cost", cwd = "sim/tests", env = { ROVER_PERF = "1" }, depends-on = ["sim-build", "sim-model", "sim-worlds"] }
# The realism report (design spec section 11): sim/data/research/realism_report.json and its contact sheet.
sim-realism = { cmd = "python sim/tools/realism_report.py", depends-on = ["sim-maps"] }
# The mission routes driven by the physical rover (tens of minutes).
sim-slow = { cmd = "python -m unittest -v test_urc_sim.MissionRoutes", cwd = "sim/tests", env = { ROVER_SLOW = "1" }, depends-on = ["sim-build", "sim-model", "sim-worlds"] }

[target.osx-arm64.activation.env]
GZ_TRANSPORT_LOCALHOST_ONLY="1"
GZ_IP="127.0.0.1"

[dependencies]
ros-jazzy-desktop = ">=0.11.0,<0.12"
ros2-ros-gz-sim = ">=1.0.23,<2"
ros2-ros-gz-bridge = ">=1.0.23,<2"
ros-dev-tools = ">=1.0.3,<2"
```

The simulation's three dependencies were the team's first three with the same ranges, so the union is the team's list.

- [ ] **Step 6: Commit the merge**

```bash
cd /Users/alarion239/Desktop/hurc_ws/src && git add pixi.toml && git diff --name-only --diff-filter=U && git commit -F - <<'EOF'
Merge Harvard-HURC/Rover-Software main into the simulation's history

The team's skeleton (rover_description, rover_sim, the pixi workspace for
osx-arm64, win-64 and linux-64) becomes an ancestor of the simulation's
history (papers/ removed from every commit), so GitHub can compare the
import with main. Conflicts:
- pixi.toml: the team's workspace, activation and dependencies (they include
  the simulation's three, with the same ranges) plus the simulation's tasks,
  still on sim/;
- pixi.lock: the team's; it holds every package the simulation uses, with
  the physics and rendering libraries at the versions the simulation runs;
- .gitignore: the team's ROS template plus the simulation's rules.
.pixi/config.toml comes with the simulation (pixi runs the packages'
post-link scripts).

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
EOF
```

Expected: `git diff --name-only --diff-filter=U` prints nothing (no unmerged file), then the commit is made.

- [ ] **Step 7: Check the merge**

```bash
cd /Users/alarion239/Desktop/hurc_ws/src
git merge-base --is-ancestor origin/main HEAD && echo "team main is an ancestor"
git rev-list --count HEAD
git diff --stat origin/main HEAD -- README.md rover_description rover_sim .gitattributes
git log --format='%an <%ae>' origin/main..HEAD | sort | uniq -c
git rev-list --objects --all | grep -c " papers/"
```

Expected: `team main is an ancestor`; count = SOURCE's count + 4 team commits + 1 merge (97 on 2026-10-09); the `diff --stat` prints nothing (the team's files are untouched); all commits on the branch are by `Alexander Belotserkovtsev <amb30239@gmail.com>`; `0`: no object of `papers/` reached this repository (the publishers' PDFs must never be pushed).

- [ ] **Step 8: Delete the temporary clone**

```bash
rm -rf /private/tmp/claude-502/rover-pr1/rover-filtered && ls /private/tmp/claude-502/rover-pr1
```

Expected: an empty listing (the branch holds every object it needs).

### Task 3: Move the trees (renames only)

**Files:** `sim/README.md` to `docs/sim/manual.md`; 35 entries of `sim/data/research/` (539 files) to `docs/research/`; `driver/` to `rover_control/`; every other entry of `sim/` except `sim/CMakeLists.txt` into `rover_sim/`.

- [ ] **Step 1: Move with `git mv`**

```bash
cd /Users/alarion239/Desktop/hurc_ws/src
mkdir -p docs/sim docs/research
git mv sim/README.md docs/sim/manual.md
keep=" terrain_targets.json mdrs_terrain_measurements.json realism_report.json realism_contact_sheet.jpg "
for e in $(git ls-tree --name-only HEAD sim/data/research/); do
  case "$keep" in *" $(basename "$e") "*) continue ;; esac
  git mv "$e" docs/research/
done
git mv driver rover_control
for e in $(git ls-tree --name-only HEAD sim/); do
  case "$e" in sim/CMakeLists.txt|sim/README.md) continue ;; esac
  git mv "$e" rover_sim/
done
git ls-files sim
git ls-files rover_sim/data/research
git diff --cached -M --name-status | cut -f1 | sort | uniq -c
git ls-files docs/research | wc -l
```

Expected:
```
sim/CMakeLists.txt
rover_sim/data/research/mdrs_terrain_measurements.json
rover_sim/data/research/realism_contact_sheet.jpg
rover_sim/data/research/realism_report.json
rover_sim/data/research/terrain_targets.json
 678 R100
     539
```

(662 renames out of `sim/` and 16 out of `driver/`; on 2026-10-09 `sim/` has 663 tracked files and `sim/data/research` 543. If SOURCE changed since, the counts change with it, but every line must still be `R100`.)

- [ ] **Step 2: Commit**

```bash
cd /Users/alarion239/Desktop/hurc_ws/src && git commit -q -F - <<'EOF'
Layout: sim/ becomes rover_sim/, driver/ rover_control/, research to docs/research (moves only)

Pure renames, so history and blame follow every file (design spec 9.1,
9.2 row 1):
- sim/* into the team's rover_sim/ package (its CMakeLists.txt is folded
  into the team's in the next commit);
- sim/README.md to docs/sim/manual.md, split up later in this PR;
- sim/data/research to docs/research/, except the four files the code reads
  or writes (terrain_targets.json, mdrs_terrain_measurements.json,
  realism_report.json, realism_contact_sheet.jpg), which stay in
  rover_sim/data/research/;
- driver/ to rover_control/, code unchanged.

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
EOF
git show --stat --format=%s HEAD | tail -1
```

Expected: `678 files changed, 0 insertions(+), 0 deletions(-)`.

### Task 4: `rover_sim` and `rover_control` as ament_cmake packages; ignore rules

**Files:**
- Modify: `rover_sim/CMakeLists.txt` (the team's, extended with `sim/CMakeLists.txt`); delete `sim/CMakeLists.txt`
- Modify: `rover_sim/package.xml`
- Create: `rover_sim/env-hooks/rover_sim.dsv.in`
- Modify: `rover_control/CMakeLists.txt` (was `driver/CMakeLists.txt`)
- Create: `rover_control/package.xml`
- Create: `docs/COLCON_IGNORE`
- Modify: `.gitignore`

- [ ] **Step 1: Write `rover_sim/CMakeLists.txt` and delete `sim/CMakeLists.txt`**

```cmake
cmake_minimum_required(VERSION 3.20)
project(rover_sim CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
# The installed plugins keep the paths of the libraries they link (the pixi environment's).
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(gz-sim8 REQUIRED)
find_package(gz-plugin2 REQUIRED COMPONENTS register)
find_package(gz-rendering8 REQUIRED)
find_package(gz-common5 REQUIRED COMPONENTS geospatial)

# One Gazebo system plugin per plugins/<name>.cpp, named in CamelCase:
# rocker_differential.cpp -> libRockerDifferential, which a model loads as
# <plugin filename="RockerDifferential">. Gazebo searches
# GZ_SIM_SYSTEM_PLUGIN_PATH, which gzenv.py points at the build directory
# (the workspace's build/rover_sim, where colcon builds this package; any
# `cmake -B` directory works) and a sourced install/ at lib/rover_sim
# (env-hooks/rover_sim.dsv.in). Every plugin may use gz-rendering (camera
# projection) and gz-common's ImageHeightmap (plugins/terrain_heightmap.hh);
# the linker keeps only what a plugin uses.
set(plugin_targets "")
file(GLOB plugin_sources CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/plugins/*.cpp")
foreach(source IN LISTS plugin_sources)
  get_filename_component(stem "${source}" NAME_WE)
  string(REPLACE "_" ";" words "${stem}")
  set(target "")
  foreach(word IN LISTS words)
    string(SUBSTRING "${word}" 0 1 first)
    string(SUBSTRING "${word}" 1 -1 rest)
    string(TOUPPER "${first}" first)
    string(APPEND target "${first}${rest}")
  endforeach()
  add_library(${target} SHARED "${source}")
  target_include_directories(${target} PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/plugins")
  target_link_libraries(${target} PRIVATE gz-sim8::gz-sim8 gz-plugin2::register gz-rendering8::gz-rendering8
                        gz-common5::geospatial)
  target_compile_options(${target} PRIVATE -Wall -Wextra -Wpedantic)
  if(APPLE)
    target_link_options(${target} PRIVATE "LINKER:-dead_strip_dylibs")
  else()
    target_link_options(${target} PRIVATE "LINKER:--as-needed")
  endif()
  list(APPEND plugin_targets ${target})
endforeach()
install(TARGETS ${plugin_targets} LIBRARY DESTINATION lib/${PROJECT_NAME})

# C++ unit tests of the plugins' parts that need no Gazebo running:
# pixi run colcon-test, or ctest --test-dir <build directory>.
enable_testing()
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/plugins/tests/CMakeLists.txt")
  add_subdirectory(plugins/tests)
endif()

# The ament linters (ament_lint_auto) do not run on this package: its code keeps
# the simulation's own style (120 columns, its docstrings), which flake8, pep257
# and uncrustify with ROS settings reject file by file.

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/${PROJECT_NAME}.dsv.in")

ament_package()
```

```bash
cd /Users/alarion239/Desktop/hurc_ws/src && git rm -q sim/CMakeLists.txt && ls sim 2>&1
```

Expected: `ls: sim: No such file or directory`.

- [ ] **Step 2: Write `rover_sim/env-hooks/rover_sim.dsv.in`**

```
prepend-non-duplicate;GZ_SIM_SYSTEM_PLUGIN_PATH;lib/@PROJECT_NAME@
```

- [ ] **Step 3: Write `rover_sim/package.xml`**

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>rover_sim</name>
  <version>0.0.0</version>
  <description>The rover's Gazebo simulation: system plugins (drivetrain and motors, rocker differential, joint monitor, the driver station's cameras), the URC 2027 mission worlds and their generator, the driver station, launcher and referee, and their tests. macOS and Linux; on Windows in WSL2.</description>
  <maintainer email="44044350+BeckettOBrien@users.noreply.github.com">beckett</maintainer>
  <maintainer email="amb30239@gmail.com">Alexander Belotserkovtsev</maintainer>
  <license>TODO: License declaration</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <depend>gz_sim_vendor</depend>
  <depend>gz_plugin_vendor</depend>
  <depend>gz_rendering_vendor</depend>
  <depend>gz_common_vendor</depend>

  <exec_depend>ros_gz_sim</exec_depend>
  <exec_depend>ros_gz_bridge</exec_depend>
  <exec_depend>python3-numpy</exec_depend>
  <exec_depend>python3-opencv</exec_depend>
  <exec_depend>python3-pil</exec_depend>
  <exec_depend>python3-matplotlib</exec_depend>
  <exec_depend>python3-aiohttp</exec_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

(The license stays the team's placeholder until the team picks one.)

- [ ] **Step 4: Write `rover_control/CMakeLists.txt`**

The driver's file with the project renamed, ament_cmake, install rules and MSVC-friendly warnings; the test block is unchanged apart from `${warnings}`.

```cmake
cmake_minimum_required(VERSION 3.20)
project(rover_control CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
# MSVC (the Windows build) gets /W4: its -Wall would be /Wall, every warning of every header.
if(MSVC)
  set(warnings /W4)
else()
  set(warnings -Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)

# rover_driver: the swerve design's kinematics, superseded by tank drive and kept
# until the ros2_control work replaces it (design spec 2026-10-09, section 9.2).
add_library(rover_driver STATIC
  src/kinematics_2d.cpp
  src/kinematics_3d.cpp
  src/steering.cpp
)
target_include_directories(rover_driver PUBLIC include)
target_link_libraries(rover_driver PUBLIC Eigen3::Eigen)
target_compile_options(rover_driver PRIVATE ${warnings})

add_executable(rover_state tools/rover_state.cpp)
target_link_libraries(rover_state PRIVATE rover_driver)
target_compile_options(rover_state PRIVATE ${warnings})

install(DIRECTORY include/ DESTINATION include)
install(TARGETS rover_driver ARCHIVE DESTINATION lib)
install(TARGETS rover_state RUNTIME DESTINATION lib/${PROJECT_NAME})

include(CTest)
if(BUILD_TESTING)
  find_package(GTest REQUIRED)
  add_executable(rover_driver_tests
    tests/test_kinematics_2d.cpp
    tests/test_kinematics_3d.cpp
    tests/test_steering.cpp
  )
  target_link_libraries(rover_driver_tests PRIVATE rover_driver GTest::gtest_main)
  target_compile_options(rover_driver_tests PRIVATE ${warnings})
  include(GoogleTest)
  gtest_discover_tests(rover_driver_tests)
  add_test(NAME rover_state_runs COMMAND rover_state --mode 3d --vx 0.5 --wz 0.2 --ql 0.1 --dqr -0.2)
  add_test(NAME rover_state_rejects_bad_flag COMMAND rover_state --bogus 1)
  # strtod accepts nan, inf and out-of-range values (as inf); JSON has neither.
  add_test(NAME rover_state_rejects_nan COMMAND rover_state --vx nan)
  add_test(NAME rover_state_rejects_out_of_range COMMAND rover_state --wz 1e999)
  set_tests_properties(rover_state_rejects_bad_flag rover_state_rejects_nan
                       rover_state_rejects_out_of_range PROPERTIES WILL_FAIL TRUE)
  # A rejection names its reason (the exit code is checked by the tests above).
  add_test(NAME rover_state_explains_rejection COMMAND rover_state --vx abc)
  set_tests_properties(rover_state_explains_rejection PROPERTIES
                       PASS_REGULAR_EXPRESSION "invalid value 'abc' for --vx")
  # viz.py reads the output with json.loads, so it must parse as JSON in both
  # modes, also when a huge but finite command overflows in the kinematics.
  find_package(Python3 COMPONENTS Interpreter)
  if(Python3_Interpreter_FOUND)
    function(add_rover_state_json_test name)
      string(JOIN " " args ${ARGN})
      set(pipeline "'$<TARGET_FILE:rover_state>' ${args} | '${Python3_EXECUTABLE}' -m json.tool")
      add_test(NAME ${name} COMMAND sh -c "${pipeline}")
    endfunction()
    add_rover_state_json_test(rover_state_json_3d --mode 3d --vx 0.5 --wz 0.2 --ql 0.1 --dqr -0.2)
    add_rover_state_json_test(rover_state_json_2d --mode 2d --vx 0.5 --vy -0.3 --wz 0.2)
    add_rover_state_json_test(rover_state_json_overflow --mode 3d --vx 1e300 --gx 1e300)
    # viz.py drawing tests, where its matplotlib and numpy are installed.
    execute_process(COMMAND "${Python3_EXECUTABLE}" -c "import matplotlib, numpy"
                    RESULT_VARIABLE viz_deps_result OUTPUT_QUIET ERROR_QUIET)
    if(viz_deps_result EQUAL 0)
      add_test(NAME viz_py
               COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/tests/test_viz.py")
      set(viz_env ROVER_STATE_EXE=$<TARGET_FILE:rover_state> MPLBACKEND=Agg
                  PYTHONDONTWRITEBYTECODE=1)
      set_tests_properties(viz_py PROPERTIES ENVIRONMENT "${viz_env}")
    else()
      message(STATUS "matplotlib or numpy missing: skipping the viz_py test")
    endif()
  endif()
endif()

ament_export_include_directories(include)
ament_export_libraries(rover_driver)
ament_export_dependencies(Eigen3)
ament_package()
```

- [ ] **Step 5: Write `rover_control/package.xml`**

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>rover_control</name>
  <version>0.0.0</version>
  <description>The rover's drive code. For now the swerve design's kinematics library rover_driver (2D and 3D inverse and forward kinematics, steering), its tests and the rover_state and viz.py tools; superseded by tank drive and kept until the ros2_control work replaces it.</description>
  <maintainer email="amb30239@gmail.com">Alexander Belotserkovtsev</maintainer>
  <license>TODO: License declaration</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <depend>eigen</depend>

  <test_depend>gtest</test_depend>
  <test_depend>python3-matplotlib</test_depend>
  <test_depend>python3-numpy</test_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

- [ ] **Step 6: Create `docs/COLCON_IGNORE`**

```bash
cd /Users/alarion239/Desktop/hurc_ws/src && : > docs/COLCON_IGNORE
```

colcon would otherwise take `docs/research/drive/prototype/cpp/` and `docs/research/flycam/prototype/` (each has a `CMakeLists.txt`) for packages.

- [ ] **Step 7: Point `.gitignore` at the new paths**

```bash
cd /Users/alarion239/Desktop/hurc_ws/src && git show origin/main:.gitignore > .gitignore && cat >> .gitignore <<'EOF'

# Python bytecode caches
__pycache__/

# generated URC mission worlds, test courses and their models (pixi run sim-worlds; deterministic), and their
# rendered maps (pixi run sim-maps)
rover_sim/models/urc_*/
rover_sim/worlds/urc_*
rover_sim/worlds/proving_ground.*
rover_sim/worlds/*_map.*
# world copies the tests run (worldfiles.temp_sdf), left behind by a killed run
rover_sim/worlds/tmp*.sdf

# the terrain rasters (pixi run fetch-data; rover_sim/data/rasters.json lists them, the .json beside each says
# where it came from), and the textures a research prototype makes
rover_sim/data/**/*.tif
docs/research/render/prototype/textures/

# agent worktrees and local Claude state
.claude/
EOF
git diff --stat -- .gitignore
```

Expected: `.gitignore | 17 +++++++++--------` (seven rules on the new paths, the rasters' comment rewritten).

- [ ] **Step 8: Commit**

```bash
cd /Users/alarion239/Desktop/hurc_ws/src && git add -A rover_sim rover_control docs/COLCON_IGNORE .gitignore && git status --short && git commit -q -F - <<'EOF'
Layout: rover_sim and rover_control as ament_cmake packages

- rover_sim: the team's CMakeLists.txt and package.xml extended with the
  simulation's plugin build (unchanged rules), plugin tests and an install of
  the plugins to lib/rover_sim with an environment hook that puts them on
  GZ_SIM_SYSTEM_PLUGIN_PATH in a sourced workspace. The ament linters are
  off for this package: its code keeps the simulation's style.
- rover_control: the driver's CMake under its package name, with ament,
  install rules and /W4 for MSVC; code and tests unchanged.
- docs/COLCON_IGNORE keeps colcon out of the research prototypes.
- .gitignore rules follow the moves.

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
EOF
```

Expected `git status --short` before the commit: `D  sim/CMakeLists.txt`, `M  .gitignore`, `M  rover_sim/CMakeLists.txt`, `M  rover_sim/package.xml`, `M  rover_control/CMakeLists.txt`, `A  docs/COLCON_IGNORE`, `A  rover_control/package.xml`, `A  rover_sim/env-hooks/rover_sim.dsv.in`.

### Task 5: pixi install and colcon build of the three packages

**Files:** none (verification; fix and amend Task 4's commit only if a step fails).

- [ ] **Step 1: Install the environment (team's lock) in SRC, in the background**

```bash
T=/private/tmp/claude-502/rover-pr1; mkdir -p "$T"
cd /Users/alarion239/Desktop/hurc_ws/src && /Users/alarion239/.pixi/bin/pixi install --locked > "$T/install.log" 2>&1; tail -3 "$T/install.log"; du -sh .pixi
```

Expected: `✔ The default environment has been installed.` The `.pixi` size counts hardlinks from pixi's cache; `df -h /Users/alarion239/Desktop` should drop by far less than that.

- [ ] **Step 2: colcon sees exactly the three packages**

```bash
cd /Users/alarion239/Desktop/hurc_ws && /Users/alarion239/.pixi/bin/pixi run --manifest-path src/pixi.toml colcon list
```

Expected:
```
rover_control	src/rover_control	(ros.ament_cmake)
rover_description	src/rover_description	(ros.ament_cmake)
rover_sim	src/rover_sim	(ros.ament_cmake)
```

(colcon skips hidden folders, so `src/.pixi` is not crawled; `docs/COLCON_IGNORE` hides the prototypes.)

- [ ] **Step 3: Build**

```bash
T=/private/tmp/claude-502/rover-pr1
cd /Users/alarion239/Desktop/hurc_ws && /Users/alarion239/.pixi/bin/pixi run --manifest-path src/pixi.toml colcon build > "$T/build.log" 2>&1; tail -4 "$T/build.log"
ls build/rover_sim/*.dylib install/rover_sim/lib/rover_sim/
grep -h "^CMAKE_CXX_COMPILER:" build/rover_sim/CMakeCache.txt build/rover_control/CMakeCache.txt
```

Expected: `Summary: 3 packages finished` (a line `1 package had stderr output: rover_sim` from compiler warnings is acceptable; read them once). Both listings show `libChaseCamera.dylib libFlyCamera.dylib libJointMonitor.dylib libRockerDifferential.dylib libRoverDrivetrain.dylib`. Write down the compiler the last line names (conda's `clang++` from the environment, or Apple's `/usr/bin/c++` as SOURCE used): Task 12 needs it if a test regresses (decision 6).

- [ ] **Step 4: The installed plugins find their libraries and a sourced workspace finds the plugins**

```bash
cd /Users/alarion239/Desktop/hurc_ws
otool -l install/rover_sim/lib/rover_sim/libRoverDrivetrain.dylib | grep -A2 LC_RPATH | grep path
/Users/alarion239/.pixi/bin/pixi run --manifest-path src/pixi.toml bash -c \
  'source install/local_setup.bash && echo "$GZ_SIM_SYSTEM_PLUGIN_PATH" && ros2 pkg list | grep "^rover_"'
```

Expected: an `LC_RPATH` path `/Users/alarion239/Desktop/hurc_ws/src/.pixi/envs/default/lib`; `GZ_SIM_SYSTEM_PLUGIN_PATH` contains `/Users/alarion239/Desktop/hurc_ws/install/rover_sim/lib/rover_sim`; the packages `rover_control`, `rover_description`, `rover_sim`.

- [ ] **Step 5: rover_control's tests pass under colcon**

```bash
cd /Users/alarion239/Desktop/hurc_ws && /Users/alarion239/.pixi/bin/pixi run --manifest-path src/pixi.toml \
  colcon test --packages-select rover_control rover_sim --return-code-on-test-failure --event-handlers console_cohesion+ 2>&1 | grep -E "tests passed|Summary"
```

Expected: `100% tests passed, 0 tests failed out of 57` (rover_control: 48 GoogleTest cases and 9 CTest tests), `100% tests passed, 0 tests failed out of 3` (rover_sim's plugin tests), `Summary: 2 packages finished`.

If a step fails, fix the file from Task 4, rebuild, and fold the fix into Task 4's commit with `git commit --amend --no-edit` (the commit is local and unpushed).

---

## Part B: pixi for three platforms, paths, data and the suites on this Mac

### Task 6: The pixi manifest for three platforms and the tasks on the new layout

**Files:** Modify `pixi.toml`, `pixi.lock`.

- [ ] **Step 1: Write `pixi.toml`**

```toml
[workspace]
authors = ["Beckett O'Brien <44044350+BeckettOBrien@users.noreply.github.com>", "Alexander Belotserkovtsev <amb30239@gmail.com>"]
channels = ["https://prefix.dev/robostack-jazzy"]
name = "src"
platforms = ["osx-arm64", "win-64", "linux-64"]
version = "0.1.0"

# Run from this folder, the workspace's src/. The colcon tasks run in the workspace root above it (cwd ".."), where
# the README's colcon build runs: build/, install/ and log/ land there.
[tasks]
# Every package (on Windows without rover_sim, see the target below), after the simulation's patched media.
build = { cmd = "colcon build", cwd = "..", depends-on = ["sim-media"] }
# The C++ tests: rover_control's (GoogleTest, rover_state, viz.py) and rover_sim's three plugin test programs.
colcon-test = { cmd = "colcon test --packages-select rover_control rover_sim --return-code-on-test-failure --event-handlers console_cohesion+", cwd = "..", depends-on = ["build"] }
# The C++ tests and the simulation's suite. The rendering tests skip where there is no GPU and say why.
test = { depends-on = ["colcon-test", "sim-test"] }
# The git-ignored terrain rasters the simulation reads, from their official sources or the team's copy
# (--all: every raster; --dry-run; --verify).
fetch-data = "python rover_sim/tools/fetch_data.py"
# rover_control's interactive view of the swerve kinematics.
control-viz = { cmd = "python rover_control/tools/viz.py", env = { ROVER_STATE_EXE = "$PIXI_PROJECT_ROOT/../build/rover_control/rover_state" }, depends-on = ["build"] }
# The patched gz-rendering media (sky, haze, terrain roughness) in the workspace's build/rover_sim. Soft: without it Gazebo uses its stock media.
sim-media = "python rover_sim/tools/gz_media.py || echo 'sim-media: no patched media, Gazebo uses its stock media'"
sim-build = { cmd = "colcon build --packages-select rover_sim", cwd = "..", depends-on = ["sim-media"] }
sim-model = "python rover_sim/gen_model.py"
sim-relief = { cmd = "python rover_sim/tools/make_relief_swatches.py", inputs = ["rover_sim/tools/make_relief_swatches.py", "rover_sim/urc/dem.py", "rover_sim/urc/landscape.py", "rover_sim/urc/terrain.py", "rover_sim/data/dem/*_0p5m*", "rover_sim/data/research/mdrs_terrain_measurements.json"], outputs = ["rover_sim/data/relief/*.npz"] }
sim-worlds = { cmd = "python rover_sim/gen_worlds.py", depends-on = ["sim-model"], inputs = ["rover_sim/gen_worlds.py", "rover_sim/gen_model.py", "rover_sim/urc/**/*.py", "rover_sim/data/dem/*", "rover_sim/data/imagery/*_naip2024.*", "rover_sim/data/imagery/naip2021_far_60km.*", "rover_sim/data/relief/*", "rover_sim/data/soils/*", "rover_sim/data/research/terrain_targets.json"], outputs = ["rover_sim/worlds/urc_*.sdf", "rover_sim/worlds/urc_*.json", "rover_sim/worlds/proving_ground.sdf", "rover_sim/worlds/proving_ground.json", "rover_sim/models/urc_*/**"] }
# Orthophoto maps of the worlds for the station's Map view (needs the GPU, so not part of sim-test).
sim-maps = { cmd = "python rover_sim/tools/render_map.py", depends-on = ["sim-build", "sim-model", "sim-worlds"] }
sim = { cmd = "bash rover_sim/run.sh", depends-on = ["sim-build", "sim-model", "sim-worlds"] }
drive = { cmd = "python rover_sim/station/__main__.py", depends-on = ["sim-build", "sim-model", "sim-worlds"] }
# Every world as a tile on one page: a click starts it with its driver station, Exit stops it.
launcher = { cmd = "python rover_sim/launcher/__main__.py", depends-on = ["sim-build", "sim-model", "sim-worlds"] }
referee = "python rover_sim/referee.py"
sim-bridge = "ros2 run ros_gz_bridge parameter_bridge --ros-args -p config_file:=rover_sim/bridge.yaml"
sim-test = { cmd = "python -m unittest discover -s rover_sim/tests -v", depends-on = ["sim-build", "sim-model", "sim-worlds"] }
# The performance budgets (physics speed, start-up, memory, the drivetrain's cost against DiffDrive): slow, so opt-in.
sim-perf = { cmd = "python -m unittest -v test_perf test_drivetrain.Cost", cwd = "rover_sim/tests", env = { ROVER_PERF = "1" }, depends-on = ["sim-build", "sim-model", "sim-worlds"] }
# The realism report (realism design spec section 11): rover_sim/data/research/realism_report.json and its contact sheet.
sim-realism = { cmd = "python rover_sim/tools/realism_report.py", depends-on = ["sim-maps"] }
# The mission routes driven by the physical rover (tens of minutes).
sim-slow = { cmd = "python -m unittest -v test_urc_sim.MissionRoutes", cwd = "rover_sim/tests", env = { ROVER_SLOW = "1" }, depends-on = ["sim-build", "sim-model", "sim-worlds"] }

[target.win-64.tasks]
# Windows builds the rover's packages without their tests; the simulation and the tests run on macOS and Linux
# (on Windows in WSL2, README, Platforms).
build = { cmd = "colcon build --packages-skip rover_sim --cmake-args -DBUILD_TESTING=OFF", cwd = ".." }

[target.osx-arm64.activation.env]
GZ_TRANSPORT_LOCALHOST_ONLY="1"
GZ_IP="127.0.0.1"

[dependencies]
ros-jazzy-desktop = ">=0.11.0,<0.12"
ros2-ros-gz-sim = ">=1.0.23,<2"
ros2-ros-gz-bridge = ">=1.0.23,<2"
ros-dev-tools = ">=1.0.3,<2"
# rover_control: the kinematics library and its tests
eigen = ">=5.0.1,<6"
gtest = ">=1.18.0,<2"

# rover_sim: Gazebo's libraries for the plugins and the simulation's Python. macOS and Linux only: on Windows the
# simulation runs in WSL2.
[target.unix.dependencies]
gz-sim8 = ">=8.10.0,<9"
gz-sim8-python = ">=8.10.0,<9"
gz-plugin2 = ">=2.0.4,<3"
gz-rendering8 = ">=8.2.2,<9"
gz-common5 = ">=5.7.1,<6"
gz-transport13-python = ">=13.5.0,<14"
gz-msgs10-python = ">=10.3.2,<11"
gz-math7-python = ">=7.7.0,<8"
numpy = ">=2.5.3,<3"
py-opencv = ">=4.13.0,<5"
pillow = ">=12.3.0,<13"
matplotlib-base = ">=3.11.2,<4"
aiohttp = ">=3.14.3,<4"
```

SOURCE's `driver-build`, `driver-test` and `driver-viz` become `build`, `colcon-test` and `control-viz`; `sim-build` builds `rover_sim` with colcon; every other task keeps its name.

- [ ] **Step 2: Lock for the three platforms and install**

```bash
T=/private/tmp/claude-502/rover-pr1
cd /Users/alarion239/Desktop/hurc_ws/src && /Users/alarion239/.pixi/bin/pixi lock > "$T/lock.log" 2>&1; tail -5 "$T/lock.log"
/Users/alarion239/.pixi/bin/pixi lock --check && echo "lock up to date"
grep -c "^- name: " pixi.lock
/Users/alarion239/.pixi/bin/pixi install --locked > "$T/install2.log" 2>&1; tail -1 "$T/install2.log"
```

Expected: the lock solves (every listed package is already locked at a matching version, so few or no package changes; read `$T/lock.log`), `lock up to date`, `3` platforms, and the install finishes.

- [ ] **Step 3: The simulation's physics and rendering stack is SOURCE's; win-64 and linux-64 resolve**

```bash
cd /Users/alarion239/Desktop/hurc_ws/src && P=/Users/alarion239/.pixi/bin/pixi
$P list --platform osx-arm64 '^(libgz-sim8|libgz-physics7|dartsim-cpp|ogre-next|libgz-rendering8|python|numpy)$'
$P list --platform linux-64 '^(libgz-sim8|py-opencv|aiohttp|colcon-core)$'
$P list --platform win-64 '^(eigen|gtest|colcon-core|cxx-compiler)$'
$P task list
```

Expected osx-arm64: libgz-sim8 8.10.0, libgz-physics7 7.5.0, dartsim-cpp 6.19.4, ogre-next 2.3.3, libgz-rendering8 8.2.2, python 3.12.14, numpy 2.5.3 (SOURCE's versions). linux-64 and win-64 list all four packages each. The task list names build, colcon-test, control-viz, drive, fetch-data, launcher, referee, sim, sim-bridge, sim-build, sim-maps, sim-media, sim-model, sim-perf, sim-realism, sim-relief, sim-slow, sim-test, sim-worlds, test.

- [ ] **Step 4: Commit**

```bash
cd /Users/alarion239/Desktop/hurc_ws/src && git add pixi.toml pixi.lock && git commit -q -F - <<'EOF'
pixi: three platforms, the simulation's dependencies, tasks on the new layout

- The team's workspace and lock stay the base (osx-arm64, win-64,
  linux-64; RoboStack Jazzy; macOS activation). Added: Eigen and GoogleTest
  for rover_control, and under target.unix the simulation's direct
  dependencies, which the lock already held through ros_gz at the versions
  the simulation runs. On Windows the simulation runs in WSL2.
- Tasks run from src/: build, colcon-test, test, fetch-data and control-viz
  are new (build and colcon-test run colcon in the workspace root); the
  simulation's tasks keep their names on rover_sim/ paths; sim-build is
  colcon. On win-64, build skips rover_sim and the tests.

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
EOF
```

### Task 7: gzenv takes the plugins from colcon's build directory

**Files:**
- Modify: `rover_sim/gzenv.py:1-30,35,57`
- Test: `rover_sim/tests/test_foundations.py` (class `Environment`)

- [ ] **Step 1: Write the failing tests**

In `rover_sim/tests/test_foundations.py`, class `Environment`, replace

```python
        self.assertEqual(env["GZ_SIM_SYSTEM_PLUGIN_PATH"], str(SIM_DIR / "build"))
```

with

```python
        self.assertEqual(env["GZ_SIM_SYSTEM_PLUGIN_PATH"], str(SIM_DIR.parents[1] / "build" / "rover_sim"))
```

and add after `test_applying_it_twice_changes_nothing`:

```python
    def test_colcon_builds_the_plugins_where_gzenv_looks(self):
        """colcon build (pixi run build) puts the plugins in the workspace's
        build/rover_sim: this repository is the workspace's src/."""
        self.assertEqual(gzenv.BUILD_DIR, SIM_DIR.parents[1] / "build" / "rover_sim")
        for name in ("ChaseCamera", "FlyCamera", "JointMonitor", "RockerDifferential", "RoverDrivetrain"):
            self.assertTrue(list(gzenv.BUILD_DIR.glob(f"lib{name}.*")), f"no lib{name} in {gzenv.BUILD_DIR}")
```

- [ ] **Step 2: Run them to see them fail**

```bash
cd /Users/alarion239/Desktop/hurc_ws/src/rover_sim/tests && /Users/alarion239/.pixi/bin/pixi run python -m unittest -v \
  test_foundations.Environment.test_our_models_and_plugins_come_first test_foundations.Environment.test_colcon_builds_the_plugins_where_gzenv_looks 2>&1 | tail -4
```

Expected: `FAILED (failures=2)`: the plugin path is `.../src/rover_sim/build`, not `.../hurc_ws/build/rover_sim`.

- [ ] **Step 3: Point gzenv at the workspace's build directory**

In `rover_sim/gzenv.py` replace the module docstring's first two bullets and the constants. Replace

```python
"""The Gazebo environment, in one place: sim/run.sh (`eval "$(python
sim/gzenv.py)"`), the driver station, the tests (tests/simulate.py) and the
tools start Gazebo with environment().

- GZ_SIM_RESOURCE_PATH: sim/models first (model://rover, the cameras, urc_*).
- GZ_SIM_SYSTEM_PLUGIN_PATH: the plugin build directory first (sim/build, or
  another CMake build directory).
```

with

```python
"""The Gazebo environment, in one place: rover_sim/run.sh (`eval "$(python
rover_sim/gzenv.py)"`), the driver station, the tests (tests/simulate.py) and
the tools start Gazebo with environment().

- GZ_SIM_RESOURCE_PATH: rover_sim/models first (model://rover, the cameras,
  urc_*).
- GZ_SIM_SYSTEM_PLUGIN_PATH: the plugin build directory first: build/rover_sim
  in the colcon workspace whose src/ this repository is, where colcon build
  puts the plugins (BUILD_DIR), or another CMake build directory.
```

Replace

```python
SIM_DIR = Path(__file__).resolve().parent
BUILD_DIR = SIM_DIR / "build"
```

with

```python
SIM_DIR = Path(__file__).resolve().parent
WORKSPACE = SIM_DIR.parents[1]  # the colcon workspace: this repository is its src/
BUILD_DIR = WORKSPACE / "build" / "rover_sim"  # where colcon builds this package
```

Replace

```python
    build_dir: where the plugins are built (default sim/build); partition,
```

with

```python
    build_dir: where the plugins are built (default BUILD_DIR); partition,
```

Replace

```python
    parser.add_argument("--build-dir", help="plugin build directory (default sim/build)")
```

with

```python
    parser.add_argument("--build-dir", help="plugin build directory (default: the workspace's build/rover_sim)")
```

- [ ] **Step 4: Run the Environment tests**

```bash
cd /Users/alarion239/Desktop/hurc_ws/src/rover_sim/tests && /Users/alarion239/.pixi/bin/pixi run python -m unittest -v test_foundations.Environment 2>&1 | tail -3
```

Expected: `Ran 7 tests`, `OK`.

- [ ] **Step 5: Commit**

```bash
cd /Users/alarion239/Desktop/hurc_ws/src && git add rover_sim/gzenv.py rover_sim/tests/test_foundations.py && git commit -q -F - <<'EOF'
gzenv: the plugins from colcon's build directory

colcon builds rover_sim in <workspace>/build/rover_sim, so that is
gzenv.BUILD_DIR now (the patched media live there too, as they did in
sim/build). The station, tests and tools take it from gzenv as before;
test_foundations checks that the build puts the five plugins there.

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
EOF
```

### Task 8: Rendering tests skip where there is no GPU

**Files:**
- Create: `rover_sim/tests/gpu.py`
- Test: `rover_sim/tests/test_foundations.py` (new class `RenderingCheck`)
- Modify: `rover_sim/tests/test_render.py` (classes `Render`, `DigCues`, `Ruts`), `rover_sim/tests/test_fly_camera.py` (class `Rendering`), `rover_sim/tests/test_urc_sim.py` (`Worlds.test_camera_sees_the_start_post`)

- [ ] **Step 1: Write the failing tests**

In `rover_sim/tests/test_foundations.py` add `import gpu` above `from simulate import ...`:

```python
import gpu
from simulate import ROVER_URI, cpu_time_per_step, diffdrive, simulate, twist_at, variant_sdf, world_sdf
```

and add this class right after class `Environment` (before `def _write_four_band_geotiff`):

```python
class RenderingCheck(unittest.TestCase):
    """gpu.rendering(): whether the rendering tests run on this machine."""

    def test_rover_rendering_decides_when_set(self):
        self.assertEqual(gpu.rendering({"ROVER_RENDERING": "1"}, "linux", Path("/nonexistent")), (True, ""))
        self.assertEqual(gpu.rendering({"ROVER_RENDERING": "0"}, "darwin"), (False, "ROVER_RENDERING=0"))

    def test_every_mac_renders(self):
        self.assertEqual(gpu.rendering({}, "darwin", Path("/nonexistent")), (True, ""))

    def test_linux_renders_with_a_render_node_or_wsl2s_gpu(self):
        with tempfile.TemporaryDirectory() as d:
            dev = Path(d)
            available, why = gpu.rendering({}, "linux", dev)
            self.assertFalse(available)
            self.assertIn("no GPU", why)
            (dev / "dxg").touch()
            self.assertEqual(gpu.rendering({}, "linux", dev), (True, ""))
            (dev / "dxg").unlink()
            (dev / "dri").mkdir()
            (dev / "dri" / "renderD128").touch()
            self.assertEqual(gpu.rendering({}, "linux", dev), (True, ""))
```

- [ ] **Step 2: Run them to see them fail**

```bash
cd /Users/alarion239/Desktop/hurc_ws/src/rover_sim/tests && /Users/alarion239/.pixi/bin/pixi run python -m unittest test_foundations.RenderingCheck 2>&1 | tail -3
```

Expected: `ModuleNotFoundError: No module named 'gpu'`, `FAILED (errors=1)`.

- [ ] **Step 3: Write `rover_sim/tests/gpu.py`**

```python
"""Whether this machine can render: ogre2 needs a GPU (Metal on every Mac, a
render node on Linux, WSL2's /dev/dxg). The rendering tests take
@gpu.needs_gpu and skip, saying why, where it cannot (CI's Linux runners);
ROVER_RENDERING=1 or 0 overrides the guess."""
import os
import sys
import unittest
from pathlib import Path


def rendering(environ=None, platform=None, dev=Path("/dev")):
    """(True, "") where ogre2 can render, else (False, why)."""
    environ = os.environ if environ is None else environ
    platform = sys.platform if platform is None else platform
    forced = environ.get("ROVER_RENDERING")
    if forced in ("0", "1"):
        return (True, "") if forced == "1" else (False, "ROVER_RENDERING=0")
    if platform == "darwin" or any((dev / "dri").glob("renderD*")) or (dev / "dxg").exists():
        return True, ""
    return False, f"no GPU (no {dev}/dri/renderD*, no {dev}/dxg); ROVER_RENDERING=1 runs it anyway"


AVAILABLE, WHY = rendering()
needs_gpu = unittest.skipUnless(AVAILABLE, f"needs a GPU to render: {WHY}")
```

- [ ] **Step 4: Run them to see them pass**

```bash
cd /Users/alarion239/Desktop/hurc_ws/src/rover_sim/tests && /Users/alarion239/.pixi/bin/pixi run python -m unittest -v test_foundations.RenderingCheck 2>&1 | tail -3
```

Expected: `Ran 3 tests`, `OK`.

- [ ] **Step 5: Mark the rendering tests**

`rover_sim/tests/test_render.py`: replace

```python
from worldfiles import SIM_DIR

import gen_model  # noqa: E402  (worldfiles puts sim/ on the path)
```

with

```python
import gpu
from worldfiles import SIM_DIR

import gen_model  # noqa: E402  (worldfiles puts sim/ on the path)
```

and put `@gpu.needs_gpu` on its own line directly above each of `class Render(unittest.TestCase):`, `class DigCues(unittest.TestCase):` and `class Ruts(unittest.TestCase):`.

`rover_sim/tests/test_fly_camera.py`: replace

```python
from simulate import gen_model, world_sdf
```

with

```python
import gpu
from simulate import gen_model, world_sdf
```

and put `@gpu.needs_gpu` directly above `class Rendering(unittest.TestCase):`.

`rover_sim/tests/test_urc_sim.py`: replace

```python
from simulate import SIM_DIR, follow, simulate, spin_ratio
```

with

```python
import gpu
from simulate import SIM_DIR, follow, simulate, spin_ratio
```

and replace

```python
    def test_camera_sees_the_start_post(self):
```

with

```python
    @gpu.needs_gpu
    def test_camera_sees_the_start_post(self):
```

- [ ] **Step 6: Check the skip path and its message (no Gazebo starts)**

```bash
cd /Users/alarion239/Desktop/hurc_ws/src/rover_sim/tests && ROVER_RENDERING=0 /Users/alarion239/.pixi/bin/pixi run python -m unittest -v \
  test_fly_camera.Rendering test_render.Ruts test_urc_sim.Worlds.test_camera_sees_the_start_post 2>&1 | grep -E "skipped|^Ran|^OK"
```

Expected: every test line ends `skipped 'needs a GPU to render: ROVER_RENDERING=0'`; `Ran 13 tests` (Rendering 3, Ruts 9, the camera test 1) and `OK (skipped=13)`.

- [ ] **Step 7: Commit**

```bash
cd /Users/alarion239/Desktop/hurc_ws/src && git add rover_sim/tests && git commit -q -F - <<'EOF'
Tests: the rendering tests skip where there is no GPU, saying why

tests/gpu.py: every Mac renders, Linux with a render node, WSL2 with
/dev/dxg; ROVER_RENDERING=1 or 0 overrides. test_render's three classes,
test_fly_camera.Rendering and the start-post camera test take
@gpu.needs_gpu, so CI's Linux runner runs everything else. Nothing changes
where a GPU is.

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
EOF
```

### Task 9: Paths a user sees at run time; the worktree data linker

**Files:** Modify `rover_sim/station/link.py:366-367`, `rover_sim/tools/gz_media.py:162`, `rover_sim/tools/render_map.py:473`, `rover_sim/tools/link_data.sh:3,10`, `rover_sim/tests/test_foundations.py:136`, `rover_sim/tests/test_landscape.py:151,202`.

- [ ] **Step 1: Edit the station's hint**

`rover_sim/station/link.py`, replace

```python
        return (f"{model}: no state from it after the spawn request; its plugin needs "
                "GZ_SIM_SYSTEM_PLUGIN_PATH to include sim/build (start the world with pixi run sim or pixi run drive)")
```

with

```python
        return (f"{model}: no state from it after the spawn request; its plugin needs GZ_SIM_SYSTEM_PLUGIN_PATH "
                "to include the workspace's build/rover_sim (start the world with pixi run sim or pixi run drive)")
```

- [ ] **Step 2: Edit the tools' help defaults**

`rover_sim/tools/gz_media.py`: replace `help="default sim/build")` with `help="default: the workspace's build/rover_sim")`.

`rover_sim/tools/render_map.py`: replace `help="plugin build directory (default sim/build)")` with `help="plugin build directory (default: the workspace's build/rover_sim)")`.

- [ ] **Step 3: Edit the skip reasons that name the old data tool**

`rover_sim/tests/test_foundations.py`: replace `"NAIP 2024 is not linked (sim/tools/link_data.sh)"` with `"NAIP 2024 is not here (pixi run fetch-data)"`.

`rover_sim/tests/test_landscape.py`: replace `"the route-area DEM is not linked (sim/tools/link_data.sh)"` with `"the route-area DEM is not here (pixi run fetch-data)"` and `"the lidar DEMs are not linked (sim/tools/link_data.sh)"` with `"the lidar DEMs are not here (pixi run fetch-data)"`.

- [ ] **Step 4: `link_data.sh` on the new path**

`rover_sim/tools/link_data.sh`: replace

```bash
# from the main checkout into a git worktree: sim/tools/link_data.sh [main checkout]
```

with

```bash
# from the main checkout into a git worktree: rover_sim/tools/link_data.sh [main checkout]
```

and replace

```bash
git ls-files --others --ignored --exclude-standard -- sim/data | while read -r f; do
```

with

```bash
git ls-files --others --ignored --exclude-standard -- rover_sim/data | while read -r f; do
```

- [ ] **Step 5: Check**

```bash
cd /Users/alarion239/Desktop/hurc_ws/src
git grep -n -E "sim/build|tools/link_data\.sh\)|-- sim/data" -- rover_sim/station rover_sim/tests rover_sim/tools rover_sim/gzenv.py
bash -n rover_sim/tools/link_data.sh && echo "syntax ok"
cd rover_sim/tests && /Users/alarion239/.pixi/bin/pixi run python -m unittest test_foundations.Environment test_landscape.Swatches 2>&1 | tail -1
```

Expected: the grep prints nothing; `syntax ok`; `OK` or `OK (skipped=...)` (the rasters are linked in Task 10).

- [ ] **Step 6: Commit**

```bash
cd /Users/alarion239/Desktop/hurc_ws/src && git add -u rover_sim && git commit -q -F - <<'EOF'
Paths: run-time messages and link_data.sh on the new layout

The station's plugin hint, gz_media's and render_map's --build-dir help,
the skip reasons that named link_data.sh (now pixi run fetch-data), and
link_data.sh itself (rover_sim/data). Comments and docstrings keep sim/
(rover_sim/README.md says how to read them).

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
EOF
```

### Task 10: Link the rasters from SOURCE

**Files:** none tracked (ten git-ignored symlinks).

- [ ] **Step 1: Symlink every raster**

```bash
cd /Users/alarion239/Desktop/hurc_ws/src
for f in $(cd /Users/alarion239/Desktop/Rover/sim/data && ls dem/*.tif imagery/*.tif); do
  ln -s "/Users/alarion239/Desktop/Rover/sim/data/$f" "rover_sim/data/$f"
done
ls -lL rover_sim/data/dem/*.tif rover_sim/data/imagery/*.tif | wc -l
git status --porcelain
git check-ignore -v rover_sim/data/dem/route_area_lidar_0p5m.tif
```

Expected: `10`; `git status --porcelain` prints nothing; `.gitignore:<line>:rover_sim/data/**/*.tif	rover_sim/data/dem/route_area_lidar_0p5m.tif`.

### Task 11: `pixi run fetch-data`

**Files:**
- Create: `rover_sim/data/rasters.json`
- Create: `rover_sim/tools/fetch_data.py`
- Test: `rover_sim/tests/test_fetch_data.py`

- [ ] **Step 1: Write the manifest `rover_sim/data/rasters.json`**

The numbers were measured on SOURCE's files on 2026-10-09 (SHA-256 of each file; `content_sha256` = SHA-256 of `urc.dem.read_raster`'s float32 bands followed by `repr` of its geotransform tuple).

```json
{
 "what": "The git-ignored rasters under rover_sim/data, which pixi run fetch-data (tools/fetch_data.py) fetches when missing. sha256: the file; content_sha256: its pixels and georeferencing as urc.dem.read_raster reads them; shape: bands, rows, columns; transform: the GDAL geotransform; provenance: the tracked .json that says where it came from; official: rebuilt from the exportImage requests its provenance lists; required: read by the simulation (the others only by the research and tools/make_relief_swatches.py).",
 "mirror": "https://github.com/Harvard-HURC/Rover-Software/releases/download/data-2026-10-06/",
 "rasters": [
  {
   "path": "dem/far_dem_3dep_60km.tif",
   "provenance": "dem/far_dem_3dep_60km.json",
   "required": true,
   "official": true,
   "bytes": 9439142,
   "sha256": "cced8ed48550a918c131ae424ca97f2b699eb066726dc855efd2849b50fe6b7e",
   "content_sha256": "ea0b760e94e43d2e9c8188252ca8c87ae8119fe61b2e976fa1c09c9a8031870a",
   "shape": [1, 1440, 1500],
   "dtype": "float32",
   "transform": [-111.15, 0.0005, 0.0, 38.7, 0.0, -0.0004999999999999993]
  },
  {
   "path": "dem/mdrs_area_3dep.tif",
   "provenance": "dem/mdrs_area_3dep.json",
   "required": true,
   "official": true,
   "bytes": 15024991,
   "sha256": "22d2c12851c1347dbcc24028cad2f2ea25c19e5e8adc6915f803dafc4e97bdfc",
   "content_sha256": "063c7bcee906e9c6f8b81b0c356da076b37ed9336f5a148e0e9c7c7b198d341a",
   "shape": [1, 2617, 3490],
   "dtype": "float32",
   "transform": [-110.8100044091711, 1.1463844797180156e-05, 0.0, 38.419994708994714, 0.0, -1.1463844797179682e-05]
  },
  {
   "path": "dem/mdrs_area_lidar2018_0p5m.tif",
   "provenance": "dem/mdrs_area_lidar2018_0p5m.json",
   "required": false,
   "official": false,
   "bytes": 58431428,
   "sha256": "f09b7a3688f525ff977241f5c49bf953cd268ea0c47f0f4fa76352fd3ccfb5b5",
   "content_sha256": "833648e4dc8e263c08bf09fbc479f10b321bea1ba7c7de194eb37aa357a9b8ee",
   "shape": [1, 5840, 6980],
   "dtype": "float32",
   "transform": [-110.8100044091711, 5.731922398590078e-06, 0.0, 38.416334263340396, 0.0, -4.510348660943074e-06]
  },
  {
   "path": "dem/route_area_3dep.tif",
   "provenance": "dem/route_area_3dep.json",
   "required": true,
   "official": true,
   "bytes": 8799642,
   "sha256": "a278817e4fc2b1965ee86242128dd84d72fc10fe807c60e3ab95f2e668d627f2",
   "content_sha256": "c33b2b54f46d7dd8fd0fba0b7002b7842c886522d8a6eefadbd224e4ae8b4278",
   "shape": [1, 2442, 2268],
   "dtype": "float32",
   "transform": [-110.79, 1.1463844797182741e-05, 0.0, 38.43199735449736, 0.0, -1.1463844797182049e-05]
  },
  {
   "path": "dem/route_area_lidar_0p5m.tif",
   "provenance": "dem/route_area_lidar_0p5m.json",
   "required": true,
   "official": false,
   "bytes": 41457287,
   "sha256": "499c9454eac5433bbb0797bd2118b7b5b6295e283129a4dc727578f773847224",
   "content_sha256": "0d2da6c0449e0690788c634e454d7f69651cf2fbbd04313a8e677934c352d3db",
   "shape": [1, 6207, 4536],
   "dtype": "float32",
   "transform": [-110.79, 5.731922398588238e-06, 0.0, 38.43199735449736, 0.0, -4.5103486609427355e-06]
  },
  {
   "path": "dem/route_area_lidar_0p5m_source.tif",
   "provenance": "dem/route_area_lidar_0p5m.json",
   "required": false,
   "official": false,
   "bytes": 214914,
   "sha256": "2787f7f34c4aa7ad559dbfc2d75f4c903ee7cd7bd6cbd398029bf4f2d924f39c",
   "content_sha256": "b587fd9040d401333934e4e042d5c7e41a897098e2e2bf97e09287998d53f266",
   "shape": [1, 6207, 4536],
   "dtype": "uint8",
   "transform": [-110.79, 5.731922398588238e-06, 0.0, 38.43199735449736, 0.0, -4.5103486609427355e-06]
  },
  {
   "path": "imagery/mdrs_area_naip2024.tif",
   "provenance": "imagery/mdrs_area_naip2024.json",
   "required": false,
   "official": true,
   "bytes": 60386308,
   "sha256": "8f4d689ba36eb34a81ce53601fdbc3581e9767e3ad5b136780eabffc79ca14ed",
   "content_sha256": "b1dbde60f9ad6946651ad6c9a260bf222c67ef9f9303cc96076dbd35af683201",
   "shape": [4, 5234, 6980],
   "dtype": "uint8",
   "transform": [-110.8100044091711, 5.73192239859063e-06, 0.0, 38.41999470899472, 0.0, -5.731922398590967e-06]
  },
  {
   "path": "imagery/naip2021_far_60km.tif",
   "provenance": "imagery/naip2021_far_60km.json",
   "required": true,
   "official": true,
   "bytes": 18681182,
   "sha256": "851347de489c11c5e957a8a60ac8738c10f66c2e13b08209509b3e5d79882bcd",
   "content_sha256": "3a99825b255a25cd680e406465fc602f269e9c62fdb0232ae87a554ea0b1dabd",
   "shape": [3, 2400, 2500],
   "dtype": "uint8",
   "transform": [-111.15, 0.0003, 0.0, 38.7, 0.0, -0.00029999999999999954]
  },
  {
   "path": "imagery/naip2021_route_area.tif",
   "provenance": "imagery/naip2021_route_area.json",
   "required": false,
   "official": false,
   "bytes": 69697630,
   "sha256": "d7b6ef5e8f8aeda0e049c0fc736b18da4631fa604a74ee7a314337c80741e769",
   "content_sha256": "ce82e853e252183169d41ef2999d6270406e31336556b200e677430825fdc7c1",
   "shape": [3, 5600, 5200],
   "dtype": "uint8",
   "transform": [-110.79, 5e-06, 0.0, 38.432, 0.0, -5e-06]
  },
  {
   "path": "imagery/route_area_naip2024.tif",
   "provenance": "imagery/route_area_naip2024.json",
   "required": true,
   "official": true,
   "bytes": 37737738,
   "sha256": "6a33c4e476dedde5e2bd371a8d9c3527ce9ccb3d819986a9c35795b8d345981a",
   "content_sha256": "8eef835aaf2a3dfc57913a176bc24c21b6a5f609367c4a0694cc0dcba434215f",
   "shape": [4, 4884, 4536],
   "dtype": "uint8",
   "transform": [-110.79, 5.731922398589246e-06, 0.0, 38.43199735449737, 0.0, -5.731922398590323e-06]
  }
 ]
}
```

- [ ] **Step 2: Write the failing tests `rover_sim/tests/test_fetch_data.py`**

```python
#!/usr/bin/env python3
"""pixi run fetch-data (tools/fetch_data.py) without the network: the raster
manifest against what the simulation reads and what is here, the dry run of
a fresh clone, official tiles made into a raster and checked, the team's copy
as the fallback, and nothing installed that does not match."""
import contextlib
import io
import json
import shutil
import sys
import tempfile
import unittest
from pathlib import Path

import numpy as np

from worldfiles import SIM_DIR

sys.path.insert(0, str(SIM_DIR / "tools"))
import fetch_data  # noqa: E402
from urc import dem, farfield  # noqa: E402
from urc.missions import autonomy  # noqa: E402

MANIFEST = json.loads((fetch_data.DATA_DIR / fetch_data.MANIFEST).read_text())
STEP = 0.001  # [deg] the made-up raster's pixel
TRANSFORM = (-110.79, STEP, 0.0, 38.43, 0.0, -STEP)  # its corner and pixel (GDAL geotransform)
MIRROR = "https://mirror.test/"


def raster_bytes(bands, transform):
    """A GeoTIFF of bands (B, H, W) as fetch_data writes it."""
    with tempfile.TemporaryDirectory() as d:
        path = Path(d) / "r.tif"
        fetch_data.write_geotiff(path, bands, fetch_data.geotags(transform))
        return path.read_bytes()


class Server:
    """The network: url -> body; it records what was asked and 404s the rest."""

    def __init__(self, bodies):
        self.bodies, self.asked = dict(bodies), []

    def __call__(self, url):
        self.asked.append(url)
        if url not in self.bodies:
            raise OSError(f"HTTP Error 404: {url}")
        return self.bodies[url]


def case(directory, bands):
    """A data directory holding the manifest and provenance of one raster,
    dem/made_up.tif = bands (B, H, W), made of 2 x 2 exportImage tiles. Returns
    (data directory, {url: body} of the tiles and of the team's copy)."""
    data = Path(directory)
    (data / "dem").mkdir()
    count, height, width = bands.shape
    whole = raster_bytes(bands, TRANSFORM)
    (data / "whole.tif").write_bytes(whole)
    lon0, dlon, _, lat0, _, dlat = TRANSFORM
    bodies, urls = {MIRROR + "made_up.tif": whole}, []
    for row in (0, height // 2):
        for col in (0, width // 2):
            tile = bands[:, row:row + height // 2, col:col + width // 2]
            url = f"https://example.test/exportImage?row={row}&col={col}"
            bodies[url] = raster_bytes(tile, (lon0 + col * dlon, dlon, 0.0, lat0 + row * dlat, 0.0, dlat))
            urls.append(url)
    (data / "dem" / "made_up.json").write_text(json.dumps({"requests": urls}))
    entry = {"path": "dem/made_up.tif", "provenance": "dem/made_up.json", "required": True, "official": True,
             "bytes": len(whole), "sha256": fetch_data.file_sha256(data / "whole.tif"),
             "content_sha256": fetch_data.content_sha256(data / "whole.tif"), "shape": list(bands.shape),
             "dtype": str(bands.dtype), "transform": list(TRANSFORM)}
    (data / "whole.tif").unlink()
    (data / fetch_data.MANIFEST).write_text(json.dumps({"mirror": MIRROR, "rasters": [entry]}))
    return data, bodies


def run(data, server, *flags):
    """fetch_data.main on data with the fake network: (exit code, printed lines)."""
    out = io.StringIO()
    with contextlib.redirect_stdout(out):
        code = fetch_data.main([*flags, "--data", str(data)], get=server)
    return code, out.getvalue().splitlines()


def dem_band(height=6, width=8):
    return (1350.0 + np.arange(height * width, dtype=np.float32).reshape(1, height, width) / 7.0).astype(np.float32)


def naip_bands(height=6, width=8):
    return (np.arange(4 * height * width).reshape(4, height, width) * 37 % 256).astype(np.uint8)


class Manifest(unittest.TestCase):
    def test_every_raster_the_simulation_reads_is_required(self):
        required = {e["path"] for e in MANIFEST["rasters"] if e["required"]}
        for path in (*dem.SITE_DEMS, farfield.FAR_DEM, farfield.FAR_IMAGERY, autonomy.DEM_PATH, autonomy.NAIP_PATH):
            self.assertIn(f"{Path(path).parent.name}/{Path(path).name}", required)

    def test_each_raster_has_its_provenance_and_an_official_one_its_requests(self):
        for e in MANIFEST["rasters"]:
            with self.subTest(e["path"]):
                provenance = json.loads((fetch_data.DATA_DIR / e["provenance"]).read_text())
                requests = fetch_data.requests_of(provenance)
                self.assertEqual(bool(requests) and all("exportImage" in url for url in requests), e["official"])

    def test_the_rasters_here_are_the_manifests(self):
        here = [e for e in MANIFEST["rasters"] if (fetch_data.DATA_DIR / e["path"]).exists()]
        if not here:
            self.skipTest("no rasters here (pixi run fetch-data)")
        for e in here:
            with self.subTest(e["path"]):
                self.assertTrue(fetch_data.matches(e, fetch_data.DATA_DIR / e["path"]))


class DryRun(unittest.TestCase):
    def test_a_fresh_clone_says_what_it_would_fetch_and_touches_nothing(self):
        with tempfile.TemporaryDirectory() as d:
            data = Path(d)
            shutil.copy(fetch_data.DATA_DIR / fetch_data.MANIFEST, data)
            for e in MANIFEST["rasters"]:
                (data / e["provenance"]).parent.mkdir(exist_ok=True)
                shutil.copy(fetch_data.DATA_DIR / e["provenance"], data / e["provenance"])
            before = sorted(data.rglob("*"))
            server = Server({})
            code, lines = run(data, server, "--dry-run")
            self.assertEqual((code, server.asked, sorted(data.rglob("*"))), (0, [], before))
            self.assertEqual(len(lines), sum(e["required"] for e in MANIFEST["rasters"]))
            self.assertIn("dem/route_area_3dep.tif: missing, would fetch from the official source, 1 request(s)", lines)
            self.assertIn("imagery/route_area_naip2024.tif: missing, would fetch from the official source, "
                          "9 request(s)", lines)
            self.assertIn("dem/route_area_lidar_0p5m.tif: missing, would fetch from the team's copy", lines)


class Fetch(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()

    def tearDown(self):
        self.tmp.cleanup()

    def test_official_tiles_make_the_raster(self):
        for bands in (dem_band(), naip_bands()):
            with self.subTest(dtype=str(bands.dtype)), tempfile.TemporaryDirectory() as d:
                data, bodies = case(d, bands)
                server = Server(bodies)
                code, lines = run(data, server)
                self.assertEqual(code, 0, lines)
                self.assertNotIn(MIRROR + "made_up.tif", server.asked)
                got, transform = dem.read_raster(data / "dem" / "made_up.tif")
                np.testing.assert_array_equal(got, bands.astype(np.float32))
                self.assertEqual(transform, TRANSFORM)
                self.assertEqual(sorted(p.name for p in (data / "dem").iterdir()), ["made_up.json", "made_up.tif"])
                self.assertEqual(run(data, Server({}), "--verify"), (0, ["dem/made_up.tif: verified"]))

    def test_other_pixels_from_the_source_bring_the_teams_copy(self):
        data, bodies = case(self.tmp.name, dem_band())
        first = next(url for url in bodies if "row=0&col=0" in url)
        bodies[first] = raster_bytes(dem_band(3, 4) + 1.0, TRANSFORM)
        code, lines = run(data, Server(bodies))
        self.assertEqual(code, 0, lines)
        self.assertIn("dem/made_up.tif: the official source now returns other pixels; trying the team's copy", lines)
        self.assertEqual((data / "dem" / "made_up.tif").read_bytes(), bodies[MIRROR + "made_up.tif"])

    def test_nothing_is_installed_when_the_source_and_the_copy_fail(self):
        data, bodies = case(self.tmp.name, naip_bands())
        bodies = {url: body for url, body in bodies.items() if "row=0&col=0" not in url}
        bodies[MIRROR + "made_up.tif"] = b"not the raster"
        code, lines = run(data, Server(bodies))
        self.assertEqual(code, 1)
        self.assertTrue(lines[-1].startswith("dem/made_up.tif: FAILED"), lines)
        self.assertEqual(sorted(p.name for p in (data / "dem").iterdir()), ["made_up.json"])

    def test_a_raster_here_is_not_fetched_and_verify_catches_a_wrong_one(self):
        data, bodies = case(self.tmp.name, dem_band())
        (data / "dem" / "made_up.tif").write_bytes(bodies[MIRROR + "made_up.tif"])
        server = Server(bodies)
        self.assertEqual(run(data, server), (0, ["dem/made_up.tif: present"]))
        self.assertEqual(server.asked, [])
        (data / "dem" / "made_up.tif").write_bytes(raster_bytes(dem_band() + 0.5, TRANSFORM))
        self.assertEqual(run(data, server, "--verify"), (1, ["dem/made_up.tif: DIFFERS"]))


if __name__ == "__main__":
    unittest.main()
```

- [ ] **Step 3: Run them to see them fail**

```bash
cd /Users/alarion239/Desktop/hurc_ws/src/rover_sim/tests && /Users/alarion239/.pixi/bin/pixi run python -m unittest test_fetch_data 2>&1 | tail -3
```

Expected: `ModuleNotFoundError: No module named 'fetch_data'`, `FAILED (errors=1)`.

- [ ] **Step 4: Write `rover_sim/tools/fetch_data.py`**

(This module was run against tiles cut from SOURCE's real rasters while the plan was written: all six official rasters came back with their content verified.)

```python
#!/usr/bin/env python3
"""Fetch the git-ignored terrain rasters: pixi run fetch-data [--all] [--dry-run] [--verify].

data/rasters.json lists every raster under data/ that git ignores, with its
size, its SHA-256, the SHA-256 of its content (the pixels and georeferencing
urc.dem.read_raster reads) and the tracked .json that says where it came
from. A missing raster is fetched:
- from its official source when it is one or more ImageServer exportImage
  requests (`official`; its .json lists them as url, urls or requests): each
  tile is placed on the raster's grid by its own georeferencing, the mosaic
  is written as a deflate GeoTIFF and kept only if its content matches;
- otherwise, or when the source fails or now returns other pixels, from the
  team's copy (`mirror`, a GitHub release), kept only if the file matches.
By default only the rasters the simulation reads (`required`); --all also the
ones only the research and tools/make_relief_swatches.py read. Rasters already
here are left alone (--verify checks them against the manifest); the tracked
.json files are never written, so a fresh clone needs no --force.
"""
import argparse
import hashlib
import json
import os
import sys
import tempfile
import time
import urllib.request
from pathlib import Path

import numpy as np
from PIL import Image, TiffImagePlugin, TiffTags

SIM_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(SIM_DIR))
from urc import dem  # noqa: E402

DATA_DIR = SIM_DIR / "data"
MANIFEST = "rasters.json"  # in the data directory
TRIES = 3  # per download: the ImageServers answer HTTP 500 now and then
USER_AGENT = "Harvard-HURC Rover-Software fetch-data"


def file_sha256(path):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for block in iter(lambda: f.read(1 << 20), b""):
            h.update(block)
    return h.hexdigest()


def content_sha256(path):
    """SHA-256 of what the simulation reads from a raster: dem.read_raster's
    float32 bands (B, H, W) and geotransform. Any lossless encoding of the same
    pixels and georeferencing has the same one."""
    bands, transform = dem.read_raster(path)
    h = hashlib.sha256(np.ascontiguousarray(bands).tobytes())
    h.update(repr(tuple(float(v) for v in transform)).encode())
    return h.hexdigest()


def matches(entry, path):
    """Whether the raster at path is entry's: the same file, or (fetched from
    its official source, so encoded anew) the same content."""
    if file_sha256(path) == entry["sha256"]:
        return True
    try:
        return content_sha256(path) == entry["content_sha256"]
    except Exception:  # not a raster dem.read_raster accepts
        return False


def download(url):
    """The body at url, after up to TRIES attempts."""
    for attempt in range(TRIES):
        try:
            with urllib.request.urlopen(urllib.request.Request(url, headers={"User-Agent": USER_AGENT}),
                                        timeout=600) as response:
                return response.read()
        except OSError:
            if attempt == TRIES - 1:
                raise
            time.sleep(5 * (attempt + 1))


def requests_of(provenance):
    """The exportImage requests a raster was made from, as its .json lists them."""
    if "url" in provenance:
        return [provenance["url"]]
    return list(provenance.get("urls") or provenance.get("requests") or [])


def geotags(transform):
    """GeoTIFF tags {tag: (value, type)} for a GDAL geotransform (EPSG:4326, PixelIsArea)."""
    lon0, dlon, _, lat0, _, dlat = transform
    keys = [v for key, (value, _) in dem.GEOKEYS.items() for v in (key, 0, 1, value)]
    return {dem.MODEL_PIXEL_SCALE: ((dlon, -dlat, 0.0), TiffTags.DOUBLE),
            dem.MODEL_TIEPOINT: ((0.0, 0.0, 0.0, lon0, lat0, 0.0), TiffTags.DOUBLE),
            dem.GEO_KEY_DIRECTORY: ((1, 1, 0, len(dem.GEOKEYS), *keys), TiffTags.SHORT)}


def write_geotiff(path, bands, tags):
    """bands (B, H, W), one float32 band or three or four uint8 bands (R, G,
    B(, NIR)), as a deflate GeoTIFF with tags {tag: (value, type)}. PIL writes
    four bands only as RGBA, the fourth marked alpha (ExtraSamples 2); the mark
    is patched to 0, unspecified, as in NAIP's own files: dem.read_raster
    refuses an alpha band."""
    count = len(bands)
    if count == 1:
        img = Image.fromarray(np.ascontiguousarray(bands[0], np.float32))
    else:
        img = Image.fromarray(np.ascontiguousarray(np.moveaxis(bands, 0, -1), np.uint8))
    info = TiffImagePlugin.ImageFileDirectory_v2()
    for tag, (value, kind) in tags.items():
        info[tag] = value
        info.tagtype[tag] = kind
    img.save(path, format="TIFF", compression="tiff_deflate", tiffinfo=info)
    if count == 4:
        entry = dem.EXTRA_SAMPLES.to_bytes(2, "little") + b"\x03\x00\x01\x00\x00\x00"
        data = Path(path).read_bytes()
        if data.count(entry + b"\x02\x00") != 1:
            raise RuntimeError(f"{path}: not one alpha mark to clear")
        Path(path).write_bytes(data.replace(entry + b"\x02\x00", entry + b"\x00\x00"))


def read_tile(body):
    """(bands, geotransform) of a GeoTIFF body."""
    with tempfile.TemporaryDirectory() as d:
        path = Path(d) / "tile.tif"
        path.write_bytes(body)
        return dem.read_raster(path)


def mosaic(entry, provenance, get, out_path):
    """The raster rebuilt from its exportImage requests into out_path: each
    tile placed by its own georeferencing on the grid of entry["transform"]."""
    lon0, dlon, _, lat0, _, dlat = entry["transform"]
    out = np.zeros(entry["shape"], np.dtype(entry["dtype"]))
    covered = np.zeros(entry["shape"][1:], bool)
    for url in requests_of(provenance):
        bands, transform = read_tile(get(url))
        col, row = (transform[0] - lon0) / dlon, (transform[3] - lat0) / dlat
        if abs(col - round(col)) > 1e-3 or abs(row - round(row)) > 1e-3:
            raise ValueError(f"a tile lies off the raster's grid (column {col:.4f}, row {row:.4f}): {url}")
        col, row = round(col), round(row)
        _, h, w = bands.shape
        out[:, row:row + h, col:col + w] = bands
        covered[row:row + h, col:col + w] = True
    if not covered.all():
        raise ValueError(f"the requests leave {np.count_nonzero(~covered)} pixels uncovered")
    write_geotiff(out_path, out, geotags(entry["transform"]))


def fetch(entry, data_dir, mirror, get=download, log=print):
    """Fetch one raster into data_dir; True once it is in place and verified."""
    path = data_dir / entry["path"]
    path.parent.mkdir(parents=True, exist_ok=True)
    part = path.with_name(f".{path.name}.part")
    try:
        if entry["official"]:
            try:
                mosaic(entry, json.loads((data_dir / entry["provenance"]).read_text()), get, part)
                if content_sha256(part) == entry["content_sha256"]:
                    os.replace(part, path)
                    log(f"{entry['path']}: fetched from its official source, content verified")
                    return True
                log(f"{entry['path']}: the official source now returns other pixels; trying the team's copy")
            except Exception as e:  # any failure of the source: the team's copy is the fallback
                log(f"{entry['path']}: the official source failed ({e}); trying the team's copy")
        url = mirror + Path(entry["path"]).name
        try:
            part.write_bytes(get(url))
        except Exception as e:
            log(f"{entry['path']}: FAILED, no team's copy at {url} ({e})")
            return False
        if file_sha256(part) != entry["sha256"]:
            log(f"{entry['path']}: FAILED, the team's copy at {url} differs from rasters.json")
            return False
        os.replace(part, path)
        log(f"{entry['path']}: fetched from the team's copy, SHA-256 verified")
        return True
    finally:
        part.unlink(missing_ok=True)


def main(argv=None, get=download):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--all", action="store_true",
                        help="also the rasters only the research and the relief tool read")
    parser.add_argument("--dry-run", action="store_true", help="say what would be fetched; fetch nothing")
    parser.add_argument("--verify", action="store_true",
                        help="check the rasters here against rasters.json; fetch nothing")
    parser.add_argument("--data", type=Path, default=DATA_DIR, help=f"data directory (default {DATA_DIR})")
    args = parser.parse_args(argv)
    manifest = json.loads((args.data / MANIFEST).read_text())
    failed = 0
    for entry in manifest["rasters"]:
        if not (args.all or entry["required"]):
            continue
        path = args.data / entry["path"]
        if args.verify:
            state = "verified" if path.exists() and matches(entry, path) else "DIFFERS" if path.exists() else "MISSING"
            print(f"{entry['path']}: {state}")
            failed += state != "verified"
        elif path.exists():
            print(f"{entry['path']}: present")
        elif args.dry_run:
            if entry["official"]:
                count = len(requests_of(json.loads((args.data / entry["provenance"]).read_text())))
                print(f"{entry['path']}: missing, would fetch from the official source, {count} request(s)")
            else:
                print(f"{entry['path']}: missing, would fetch from the team's copy")
        elif not fetch(entry, args.data, manifest["mirror"], get):
            failed += 1
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
```

- [ ] **Step 5: Run the tests to see them pass**

```bash
cd /Users/alarion239/Desktop/hurc_ws/src/rover_sim/tests && /Users/alarion239/.pixi/bin/pixi run python -m unittest -v test_fetch_data 2>&1 | tail -3
```

Expected: `Ran 8 tests`, `OK` (`test_the_rasters_here_are_the_manifests` checks the ten linked rasters).

- [ ] **Step 6: Verify the linked data and the dry run through the task (no download)**

```bash
cd /Users/alarion239/Desktop/hurc_ws/src && P=/Users/alarion239/.pixi/bin/pixi
$P run fetch-data --verify --all
$P run fetch-data --dry-run
```

Expected: ten lines `<path>: verified`; then six lines `<path>: present` (the required ones: `dem/far_dem_3dep_60km.tif`, `dem/mdrs_area_3dep.tif`, `dem/route_area_3dep.tif`, `dem/route_area_lidar_0p5m.tif`, `imagery/naip2021_far_60km.tif`, `imagery/route_area_naip2024.tif`). Do not run `pixi run fetch-data` without `--dry-run`/`--verify` against an empty data directory: that downloads (Task 18 checks the fresh-clone dry run).

- [ ] **Step 7: Commit**

```bash
cd /Users/alarion239/Desktop/hurc_ws/src && git add rover_sim/data/rasters.json rover_sim/tools/fetch_data.py rover_sim/tests/test_fetch_data.py && git commit -q -F - <<'EOF'
fetch-data: the git-ignored rasters from their official sources, else the team's copy

pixi run fetch-data fetches the missing rasters listed in
rover_sim/data/rasters.json (size, SHA-256 of the file and of the content
dem.read_raster reads, provenance) and works on a fresh clone without
--force: the tracked .json files are never written.
- The six rasters that are ImageServer exportImage requests (3DEP DEMs,
  NAIP 2024 and 2021 imagery) are rebuilt from the requests their .json
  lists: tiles placed by their own georeferencing, kept only if the content
  matches.
- The other four (the 0.5 m lidar DEMs and their source map, made by
  processing that is not in the repository; the NAIP 2021 route mosaic,
  whose provenance has only a URL template), or an official source that
  fails or changed, come from the team's copy, the release data-2026-10-06
  of this repository, kept only if the file's SHA-256 matches. The release
  is made when the user approves it.
- --all also fetches what only the research and the relief tool read;
  --dry-run says what it would fetch; --verify checks what is here.
test_fetch_data runs every path offline against a fake server.

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
EOF
```

### Task 12: Generate and run the full suites on this Mac

**Files:** none (verification). A failure here is fixed in the task that caused it, as a new commit.

- [ ] **Step 1: List the tests in SOURCE and in the workspace**

```bash
T=/private/tmp/claude-502/rover-pr1; mkdir -p "$T"; P=/Users/alarion239/.pixi/bin/pixi
cat > "$T/list_tests.py" <<'EOF'
import sys
import unittest


def walk(suite):
    for test in suite:
        yield from walk(test) if isinstance(test, unittest.TestSuite) else [test]


for test in walk(unittest.defaultTestLoader.discover(".", pattern="test_*.py")):
    print(test.id())
EOF
(cd /Users/alarion239/Desktop/Rover/sim/tests && PYTHONDONTWRITEBYTECODE=1 $P run --frozen --manifest-path /Users/alarion239/Desktop/Rover/pixi.toml python "$T/list_tests.py" 2>/dev/null | sort > "$T/tests_source.txt")
(cd /Users/alarion239/Desktop/hurc_ws/src/rover_sim/tests && $P run python "$T/list_tests.py" 2>/dev/null | sort > "$T/tests_ws.txt")
wc -l < "$T/tests_source.txt"; wc -l < "$T/tests_ws.txt"; diff "$T/tests_source.txt" "$T/tests_ws.txt"
```

Expected: `495`, `507`, and a diff of additions only:
```
> test_fetch_data.DryRun.test_a_fresh_clone_says_what_it_would_fetch_and_touches_nothing
> test_fetch_data.Fetch.test_a_raster_here_is_not_fetched_and_verify_catches_a_wrong_one
> test_fetch_data.Fetch.test_nothing_is_installed_when_the_source_and_the_copy_fail
> test_fetch_data.Fetch.test_official_tiles_make_the_raster
> test_fetch_data.Fetch.test_other_pixels_from_the_source_bring_the_teams_copy
> test_fetch_data.Manifest.test_each_raster_has_its_provenance_and_an_official_one_its_requests
> test_fetch_data.Manifest.test_every_raster_the_simulation_reads_is_required
> test_fetch_data.Manifest.test_the_rasters_here_are_the_manifests
> test_foundations.Environment.test_colcon_builds_the_plugins_where_gzenv_looks
> test_foundations.RenderingCheck.test_every_mac_renders
> test_foundations.RenderingCheck.test_linux_renders_with_a_render_node_or_wsl2s_gpu
> test_foundations.RenderingCheck.test_rover_rendering_decides_when_set
```

(The diff lines appear at their sorted positions.)

- [ ] **Step 2: Build, generate the worlds and render the maps (GPU, about 3 minutes)**

```bash
T=/private/tmp/claude-502/rover-pr1
cd /Users/alarion239/Desktop/hurc_ws/src && GZ_IP=127.0.0.1 GZ_PARTITION=pr1_maps_$$ /Users/alarion239/.pixi/bin/pixi run sim-maps > "$T/maps.log" 2>&1; tail -5 "$T/maps.log"
ls rover_sim/worlds/
```

Expected: the media, the colcon build, the rover model, the five worlds (about 65 s) and five maps (about 64 s) are made; `rover_sim/worlds/` lists `rover_test.sdf` and for each of `proving_ground`, `urc_astrobiology`, `urc_autonomy`, `urc_delivery`, `urc_equipment_servicing` its `.sdf`, `.json`, `_map.jpg` and `_map.json` (the maps let `test_launcher`'s thumbnail test run instead of skipping).

- [ ] **Step 3: Run `pixi run test` in the background (about 15 minutes)**

```bash
T=/private/tmp/claude-502/rover-pr1
cd /Users/alarion239/Desktop/hurc_ws/src && GZ_IP=127.0.0.1 GZ_PARTITION=pr1_test_$$ /Users/alarion239/.pixi/bin/pixi run test > "$T/test.log" 2>&1; echo "exit $?" >> "$T/test.log"
```

(The suite gives every simulation its own partition; `GZ_PARTITION` covers anything that does not.)

Run it with `run_in_background` and wait for it to finish. Then:

```bash
T=/private/tmp/claude-502/rover-pr1
grep -E "tests passed|^Summary" "$T/test.log"; grep -E "^Ran |^OK|^FAILED|^exit" "$T/test.log"
```

Expected:
```
100% tests passed, 0 tests failed out of 57
100% tests passed, 0 tests failed out of 3
Summary: 2 packages finished [...]
Ran 507 tests in ...s
OK (skipped=9, expected failures=2)
exit 0
```

The 9 skips are the opt-in performance and slow tests (`test_perf.Budgets`, `test_drivetrain.Cost` x2, `test_urc_sim.MissionRoutes` x6), the two expected failures the slow-turn judder and dust in depth, as in SOURCE (its README, Performance: 495 tests, 9 skipped, 2 expected failures).

If a test fails: run the same test in SOURCE (`cd /Users/alarion239/Desktop/Rover/sim/tests && GZ_IP=127.0.0.1 GZ_PARTITION=pr1_src_$$ PYTHONDONTWRITEBYTECODE=1 pixi run --frozen --manifest-path ../../pixi.toml python -m unittest -v <test id>`). A test that fails in SOURCE too is not caused by this PR: write it down for the report. A test that passes in SOURCE and fails here is a regression: find the cause (paths, the build directory, the compiler) with superpowers:systematic-debugging, fix it in a new commit, and rerun the suite. If the compiler noted in Task 5 Step 3 is conda's and the regression is numeric (a calibration row, a tolerance), rebuild once with Apple's compiler to tell (`rm -rf build/rover_sim && pixi run --manifest-path src/pixi.toml colcon build --packages-select rover_sim --cmake-args -DCMAKE_CXX_COMPILER=/usr/bin/c++` from the workspace root, then `pixi run sim-media` and the failing test; afterwards `rm -rf build/rover_sim` and `pixi run build` again, so the workspace is back on the default compiler); report the result to the user instead of pinning a compiler in the build files.

- [ ] **Step 4: Nothing tracked changed**

```bash
cd /Users/alarion239/Desktop/hurc_ws/src && git status --porcelain && echo "tree clean"
```

Expected: `tree clean` (the regenerated rover and camera models equal the tracked ones; worlds, maps, rasters and caches are ignored).

---

## Part C: launch file, CI, documents, ignore rules, acceptance

### Task 13: The team's Gazebo launch file works off macOS

**Files:**
- Create: `rover_sim/tests/test_launch_files.py`
- Modify: `rover_sim/launch/gazebo.launch.py:42`

- [ ] **Step 1: Write the failing test `rover_sim/tests/test_launch_files.py`**

```python
#!/usr/bin/env python3
"""The package's launch files (rover_sim/launch) describe a launch on every
platform; they are only loaded here, nothing is started."""
import contextlib
import importlib.util
import io
import sys
import unittest
from unittest import mock

from launch import LaunchDescription

from worldfiles import SIM_DIR

GAZEBO_LAUNCH = SIM_DIR / "launch" / "gazebo.launch.py"


def describe(path, platform):
    """generate_launch_description() of a launch file as `platform` (sys.platform) runs it."""
    spec = importlib.util.spec_from_file_location(path.stem.replace(".", "_"), path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    with mock.patch.object(sys, "platform", platform), contextlib.redirect_stdout(io.StringIO()):
        return module.generate_launch_description()


class GazeboLaunch(unittest.TestCase):
    def test_macos_linux_and_windows_get_a_launch_description(self):
        for platform in ("darwin", "linux", "win32"):
            with self.subTest(platform):
                self.assertIsInstance(describe(GAZEBO_LAUNCH, platform), LaunchDescription)


if __name__ == "__main__":
    unittest.main()
```

- [ ] **Step 2: Run it to see it fail**

```bash
cd /Users/alarion239/Desktop/hurc_ws/src/rover_sim/tests && /Users/alarion239/.pixi/bin/pixi run python -m unittest -v test_launch_files 2>&1 | grep -E "NameError|^FAILED"
```

Expected: `NameError: name 'pkg_ros_gz_sim' is not defined` (for linux and win32), `FAILED (errors=2)`.

- [ ] **Step 3: Fix the launch file**

`rover_sim/launch/gazebo.launch.py`, replace

```python
				os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
```

with

```python
				os.path.join(gz_pkg, 'launch', 'gz_sim.launch.py')
```

(The file is indented with tabs; keep them.)

- [ ] **Step 4: Run it to see it pass, and load the installed launch file**

```bash
cd /Users/alarion239/Desktop/hurc_ws/src/rover_sim/tests && /Users/alarion239/.pixi/bin/pixi run python -m unittest -v test_launch_files 2>&1 | tail -1
/Users/alarion239/.pixi/bin/pixi run python -m unittest test_foundations.Environment test_launch_files 2>&1 | tail -1
cd /Users/alarion239/Desktop/hurc_ws && /Users/alarion239/.pixi/bin/pixi run --manifest-path src/pixi.toml bash -c \
  'colcon build --packages-select rover_sim > /dev/null && source install/local_setup.bash && ros2 launch rover_sim gazebo.launch.py --show-args'
```

Expected: `OK` twice (the second run imports `launch` after `worldfiles` has put `rover_sim/` on the path, as the full suite does: the ROS package still wins over the `launch/` folder, which has no `__init__.py`); then `Arguments (pass arguments as '<name>:=<value>'):` and `'world':` with `Name or path of the Gazebo SDF world file` and `(default: 'empty.sdf')`. Nothing starts.

- [ ] **Step 5: Commit**

```bash
cd /Users/alarion239/Desktop/hurc_ws/src && git add rover_sim/launch/gazebo.launch.py rover_sim/tests/test_launch_files.py && git commit -q -F - <<'EOF'
rover_sim: gazebo.launch.py works on Linux and Windows

Its non-macOS branch used pkg_ros_gz_sim, a name it never defined; the
share directory of ros_gz_sim is gz_pkg. test_launch_files loads the
launch description as macOS, Linux and Windows would.

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
EOF
```

### Task 14: CI

**Files:** Create `.github/workflows/ci.yml`.

- [ ] **Step 1: Write `.github/workflows/ci.yml`**

```yaml
# CI for every pull request and for main: the pixi tasks a laptop runs (README, Tests).
name: CI

on:
  pull_request:
  push:
    branches: [main]

permissions:
  contents: read

concurrency:
  group: ci-${{ github.ref }}
  cancel-in-progress: true

jobs:
  unix:
    name: ${{ matrix.name }}
    strategy:
      fail-fast: false
      matrix:
        include:
          - name: Ubuntu 24.04 (no GPU, the rendering tests skip)
            os: ubuntu-24.04
            rendering: "0"
          - name: macOS, Apple silicon (with the rendering tests)
            os: macos-15
            rendering: ""
    runs-on: ${{ matrix.os }}
    timeout-minutes: 150
    env:
      GZ_IP: 127.0.0.1
      # tests/gpu.py: 0 skips the rendering tests (the Ubuntu runner has no GPU, whatever device nodes its virtual
      # machine shows); empty lets gpu.py decide (every Mac renders).
      ROVER_RENDERING: ${{ matrix.rendering }}
    defaults:
      run:
        working-directory: src
    steps:
      # The repository is the src/ folder of a colcon workspace: colcon builds in the folder above it.
      - uses: actions/checkout@v7.0.1
        with:
          path: src
      - uses: prefix-dev/setup-pixi@v0.10.2
        with:
          pixi-version: v0.81.0
          manifest-path: src/pixi.toml
          locked: true
          cache: true
      # The build needs no terrain data, so it is checked even when fetch-data fails.
      - run: pixi run build
      # The terrain rasters, cached by their manifest and saved as soon as they are fetched, whatever the tests do.
      - name: Restore the terrain rasters
        id: rasters
        uses: actions/cache/restore@v6.1.0
        with:
          path: |
            src/rover_sim/data/dem/*.tif
            src/rover_sim/data/imagery/*.tif
          key: rasters-${{ runner.os }}-${{ hashFiles('src/rover_sim/data/rasters.json') }}
      - run: pixi run fetch-data
      - name: Save the terrain rasters
        if: steps.rasters.outputs.cache-hit != 'true'
        uses: actions/cache/save@v6.1.0
        with:
          path: |
            src/rover_sim/data/dem/*.tif
            src/rover_sim/data/imagery/*.tif
          key: rasters-${{ runner.os }}-${{ hashFiles('src/rover_sim/data/rasters.json') }}
      - run: pixi run test

  windows:
    name: Windows (rover_description, rover_control)
    runs-on: windows-2025
    timeout-minutes: 60
    defaults:
      run:
        working-directory: src
    steps:
      - uses: actions/checkout@v7.0.1
        with:
          path: src
      - uses: prefix-dev/setup-pixi@v0.10.2
        with:
          pixi-version: v0.81.0
          manifest-path: src/pixi.toml
          locked: true
          cache: true
      - run: pixi run build
```

The action versions are their releases older than two weeks on 2026-10-09 (`gh release list -R <repo>`: checkout v7.0.1 of 2026-07-20, setup-pixi v0.10.2 of 2026-08-28, cache v6.1.0 of 2026-06-26, whose `restore` and `save` sub-actions take `path` and `key`, and `restore` outputs `cache-hit`); all run on node24. The plain `actions/cache` would save only when the whole job succeeds, so until the suite is green every run would download the rasters again.

- [ ] **Step 2: Validate it**

```bash
cd /Users/alarion239/Desktop/hurc_ws/src && /Users/alarion239/.pixi/bin/pixi exec actionlint .github/workflows/ci.yml && echo "actionlint ok"
python3 -c "import sys, yaml; d = yaml.safe_load(open('.github/workflows/ci.yml')); print(sorted(d['jobs']))" 2>/dev/null \
  || /Users/alarion239/.pixi/bin/pixi run python -c "import yaml; d = yaml.safe_load(open('.github/workflows/ci.yml')); print(sorted(d['jobs']))"
```

Expected: `actionlint ok` (actionlint 1.7.12 from conda-forge, about 2 MB, into pixi's cache; `pixi clean cache --exec` removes it afterwards) and `['unix', 'windows']`.

- [ ] **Step 3: Commit**

```bash
cd /Users/alarion239/Desktop/hurc_ws/src && git add .github/workflows/ci.yml && git commit -q -F - <<'EOF'
CI: build and test on Ubuntu and macOS, build on Windows

GitHub Actions with prefix-dev/setup-pixi (pixi 0.81, the locked
environment, cached), the repository checked out as a workspace's src/.
Ubuntu 24.04 and macOS (Apple silicon): pixi run build, pixi run
fetch-data (the rasters cached by their manifest as soon as they are
fetched), pixi run test; the rendering tests skip on the GPU-less Ubuntu
runner (ROVER_RENDERING=0) and run on macOS. Windows: pixi run build
(rover_description and rover_control). fetch-data needs the data release
for the derived lidar DEM before the first green run.

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
EOF
```

### Task 15: The simulation's manual becomes `docs/sim/`

**Files:** Modify `docs/sim/manual.md` (the old `sim/README.md`); create `docs/sim/design-notes.md`, `docs/sim/gazebo-lessons.md`.

- [ ] **Step 1: Write the split script `$T/split_manual.py`**

````bash
T=/private/tmp/claude-502/rover-pr1; mkdir -p "$T" && cat > "$T/split_manual.py" <<'PYEOF'
"""Split the simulation's manual (docs/sim/manual.md, the old sim/README.md) into
manual.md, design-notes.md and gazebo-lessons.md, and point its paths at the
new layout. Run once from the repository root; it refuses a file it does not
recognise, and checks that no line is lost."""
import sys
from pathlib import Path

DOCS = Path("docs/sim")
lines = (DOCS / "manual.md").read_text().splitlines()
HEADINGS = {397: "## Design notes", 706: "## Known limitations",
            1291: "## Gazebo lessons (gz-sim 8.10, gz-rendering 8.2.2, DART, ogre2 on Metal)", 1534: "## Performance"}
if len(lines) != 1662 or any(lines[n - 1] != text for n, text in HEADINGS.items()):
    sys.exit("docs/sim/manual.md is not the sim/README.md this script was written for")
original = list(lines)


def part(first, last):
    """Lines first..last (1-based, inclusive)."""
    return lines[first - 1:last]


def promote(block):
    """### headings one level up (the section's own file)."""
    return [line[1:] if line.startswith("### ") else line for line in block]


design = ["# Design notes", "",
          "Moved from the simulation's manual ([manual.md](manual.md)); code comments cite them as",
          "\"sim/README.md, design notes\"."] + promote(part(398, 705))
lessons = ["# Gazebo lessons (gz-sim 8.10, gz-rendering 8.2.2, DART, ogre2 on Metal)", "",
           "Moved from the simulation's manual ([manual.md](manual.md)); code comments cite them as",
           "\"sim/README.md, Gazebo lessons\"."] + part(1292, 1533)
manual = (part(1, 396)
          + ["## Design notes", "",
             "In [design-notes.md](design-notes.md): why the differential is a plugin, not a mimic joint; the",
             "drivetrain; visible dig-in; skid-steer friction.", ""]
          + part(706, 1290)
          + ["## Gazebo lessons", "",
             "In [gazebo-lessons.md](gazebo-lessons.md): what gz-sim 8.10, gz-rendering 8.2.2, DART and ogre2 on",
             "Metal taught us while building the worlds, the station and the realism work.", ""]
          + part(1534, 1662))

QUICK_START_OLD = """```bash
pixi run sim                    # build the plugins, regenerate rover and worlds, open Gazebo (rover_test)
pixi run drive urc_autonomy     # or drive from the browser: starts the world headless, opens the station page
pixi run sim-bridge             # second terminal: ROS 2 <-> Gazebo topics
pixi run ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.3}, angular: {z: 0.2}}'
pixi run sim-test               # headless tests (~15 min with the build)
pixi run sim-maps               # orthophoto maps for the station's Map view (GPU, ~1 min)
pixi run sim-perf               # performance budgets (~25 min, wants an otherwise idle machine)
pixi run sim-slow               # the mission routes driven by the physical rover (~40 min)
pixi run sim-realism            # the realism report (sim/data/research/realism_report.json)
```"""
QUICK_START_NEW = """From the repository's root (`<workspace>/src`), after `pixi install`:

```bash
pixi run fetch-data             # the terrain rasters, once (git-ignored; --all for the research ones too)
pixi run sim                    # build the plugins (colcon), regenerate rover and worlds, open Gazebo (rover_test)
pixi run drive urc_autonomy     # or drive from the browser: starts the world headless, opens the station page
pixi run launcher               # every world as a tile: a click starts it with its station
pixi run sim-bridge             # second terminal: ROS 2 <-> Gazebo topics
pixi run ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.3}, angular: {z: 0.2}}'
pixi run test                   # the C++ tests and the simulation's suite (~15 min with the build)
pixi run sim-test               # the simulation's suite only
pixi run sim-maps               # orthophoto maps for the station's Map view (GPU, ~1 min)
pixi run sim-perf               # performance budgets (~25 min, wants an otherwise idle machine)
pixi run sim-slow               # the mission routes driven by the physical rover (~40 min)
pixi run sim-realism            # the realism report (rover_sim/data/research/realism_report.json)
```

Paths in this manual are relative to `rover_sim/` unless they start with a
top-level folder of the repository (`rover_sim/`, `rover_control/`, `docs/`)."""

# (old, new, how many times old occurs in the manual)
REPLACEMENTS = [
    (QUICK_START_OLD, QUICK_START_NEW, 1),
    ("`driver/include/rover_driver/config.hpp`", "`rover_control/include/rover_driver/config.hpp`", 1),
    ("`sim/worlds`", "`rover_sim/worlds`", 2),
    ("so `sim/run.sh`", "so `rover_sim/run.sh`", 1),
    ("| `plugins/tests/` | C++ unit tests (`ctest --test-dir sim/build`):",
     "| `plugins/tests/` | C++ unit tests (`pixi run colcon-test`):", 1),
    ("| `tools/gz_media.py`, `patches/` | The patched gz-rendering media (terrain roughness, sky, haze), built into "
     "`sim/build` |",
     "| `tools/gz_media.py`, `patches/` | The patched gz-rendering media (terrain roughness, sky, haze), built into "
     "the workspace's `build/rover_sim` |", 1),
    ("| `tools/fetch_dem.py`, `tools/link_data.sh` | Fetches a USGS 3DEP DEM; links the git-ignored rasters into a "
     "git worktree |",
     "| `tools/fetch_data.py`, `data/rasters.json` | `pixi run fetch-data`: fetches the git-ignored rasters (their "
     "official sources, else the team's copy) and checks them against the manifest |\n"
     "| `tools/fetch_dem.py`, `tools/link_data.sh` | Fetches a USGS 3DEP DEM for a new area; links the git-ignored "
     "rasters into a git worktree |", 1),
    ("| `data/research/` | Research inputs of the realism spec, the gates (`gates.json`, `gates/`), the realism "
     "report |",
     "| `data/research/` | The research files the code reads or writes: the roughness targets, the lidar windows' "
     "measurements, the realism report and its contact sheet; the rest of the research is in `docs/research/` |",
     1),
    ("(`sim/build` missing from `GZ_SIM_SYSTEM_PLUGIN_PATH`)",
     "(the workspace's `build/rover_sim` missing from `GZ_SIM_SYSTEM_PLUGIN_PATH`)", 1),
    ("![The URC worlds](../docs/urc_worlds.png)", "![The URC worlds](../urc_worlds.png)", 1),
    ("`sim/build/gz-rendering-media`", "`build/rover_sim/gz-rendering-media` (in the workspace)", 1),
    ("`sim/urc/missions/<name>.py`", "`rover_sim/urc/missions/<name>.py`", 1),
    ("`python sim/gen_worlds.py", "`python rover_sim/gen_worlds.py", 2),
    ("`sim/models`", "`rover_sim/models`", 1),
    ("`ctest --test-dir sim/build` 3 C++ programs", "`pixi run colcon-test` 3 C++ programs", 1),
    ("""  `pixi run drive` (they build the plugins and set `GZ_SIM_SYSTEM_PLUGIN_PATH`
  to `sim/build`).""",
     """  `pixi run drive` (they build the plugins and set `GZ_SIM_SYSTEM_PLUGIN_PATH`
  to the workspace's `build/rover_sim`).""", 1),
    ("run `python sim/tools/gz_media.py`", "run `python rover_sim/tools/gz_media.py`", 1),
    ("""  git-ignored. Their `.json` names the source; `python sim/tools/fetch_dem.py
  --force` fetches the 3DEP DEM again (it refuses without `--force` when the
  `.json` exists). In a git worktree, link the main checkout's:
  `sim/tools/link_data.sh <main checkout>`.""",
     """  git-ignored. `pixi run fetch-data` fetches the missing ones and checks
  them against `data/rasters.json` (each raster's `.json` says where it came
  from). In a git worktree, link the main checkout's instead:
  `rover_sim/tools/link_data.sh <main checkout>`.""", 1),
]

text = "\n".join(manual) + "\n"
for old, new, count in REPLACEMENTS:
    if text.count(old) != count:
        sys.exit(f"expected {count} x {old[:60]!r} in the manual, found {text.count(old)}")
    text = text.replace(old, new)
(DOCS / "manual.md").write_text(text)
(DOCS / "design-notes.md").write_text("\n".join(design) + "\n")
(DOCS / "gazebo-lessons.md").write_text("\n".join(lessons) + "\n")

# Nothing lost: every line of the old README, with the replacements above applied, is a line of one of the
# three files (a heading may have moved up one level).
new_lines = set()
for name in ("manual.md", "design-notes.md", "gazebo-lessons.md"):
    for line in (DOCS / name).read_text().splitlines():
        new_lines.update([line, "#" + line] if line.startswith("#") else [line])
expected = "\n".join(original) + "\n"
for old, new, _ in REPLACEMENTS:
    expected = expected.replace(old, new)
lost = [line for line in expected.splitlines() if line not in new_lines]
if lost:
    sys.exit("lines lost:\n" + "\n".join(lost))
print(f"manual.md {len(text.splitlines())} lines, design-notes.md {len(design)}, gazebo-lessons.md {len(lessons)}; "
      f"no line lost")
PYEOF
````

- [ ] **Step 2: Run it**

```bash
T=/private/tmp/claude-502/rover-pr1
cd /Users/alarion239/Desktop/hurc_ws/src && python3 "$T/split_manual.py"
grep -n -E "(^|[^a-zA-Z_/.-])(sim|driver)/" docs/sim/manual.md
grep -n "^## " docs/sim/manual.md
```

Expected: `manual.md 1129 lines, design-notes.md 312, gazebo-lessons.md 246; no line lost`; the first grep prints nothing; the headings are Quick start, Files, Topics, Changing the rover, Driver station, Design notes, Known limitations, URC 2027 mission worlds, Gazebo lessons, Performance, Troubleshooting.

(The script was tried on a copy of SOURCE's `sim/README.md` while this plan was written, with this result.)

- [ ] **Step 3: Commit**

```bash
cd /Users/alarion239/Desktop/hurc_ws/src && git add docs/sim && git commit -q -F - <<'EOF'
Docs: the simulation's manual in docs/sim, design notes and Gazebo lessons beside it

The old sim/README.md (docs/sim/manual.md since the move) loses its two
longest sections to design-notes.md and gazebo-lessons.md, word for word
(their subsections one heading level up), and points at them. Its quick
start, file table, plugin and media paths and troubleshooting follow the
new layout (pixi run fetch-data, pixi run test, the workspace's
build/rover_sim). No other line changed.

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
EOF
```

### Task 16: READMEs and the papers' references

**Files:** Modify `README.md`, `rover_sim/README.md`, `docs/superpowers/specs/2026-10-06-urc-realism-design.md:1295,1297,1306`; create `rover_control/README.md`, `docs/research/README.md`.

- [ ] **Step 1: Write `README.md` (the team's text, extended)**

````markdown
# HURC Rover Software

Welcome to the central repository for the HURC Rover's control software!

### Setup
We use [pixi](https://pixi.prefix.dev/latest/#installation) to manage ROS packages with [RoboStack](https://robostack.github.io/GettingStarted.html)

1. Install pixi
2. Create a new folder to be your root ROS workspace (ex: `/home/username/hurc_ws/`)
3. Clone this repo into a subfolder called `src` (ex: `/home/username/hurc_ws/src/`)
4. Navigate to the `src` folder and run `pixi install`
5. Run `pixi shell` from this folder anytime you need to interact with installed packages (like ROS)

### Usage
To build and source our custom packages for testing, follow these steps:
1. Starting from a pixi shell, go to the root folder of the workspace (such that you can see the `src/` folder)
2. Run `colcon build` and wait for all packages to build (you can use `--packages-select package_name other_package_name ...` to build only specific packages, and you can use `--continue-on-error` to continue building other packages after one fails)
3. Source the appropriate setup file from the `install` folder (ex: on bash, run `source install/local_setup.bash`)
You should now be able to use launch files, executables, and assets installed by any of our packages in your current shell.

From the `src` folder, `pixi run build` does step 2 for you (`colcon build` in the workspace root), and the tasks below build what they need first.

### Simulation
The rover's Gazebo simulation (`rover_sim`) runs on macOS and Linux; on Windows it runs in WSL2 (see Platforms). From the `src` folder:
1. `pixi run fetch-data` downloads the terrain rasters the simulation reads, once (about 130 MB of USGS elevation and lidar data and USDA NAIP imagery; git ignores them)
2. `pixi run sim` builds the simulation's plugins, generates the rover and the worlds and opens Gazebo on the test ground (`pixi run sim urc_autonomy` opens a URC mission world)
3. `pixi run drive urc_autonomy` drives the rover from the browser; `pixi run launcher` shows every world as a tile that starts it with its driver station

The manual is [docs/sim/manual.md](docs/sim/manual.md), with its [design notes](docs/sim/design-notes.md) and [Gazebo lessons](docs/sim/gazebo-lessons.md).

### Tests
- `pixi run test` runs the C++ tests of `rover_control` and `rover_sim` (`colcon test`) and the simulation's suite: about 15 minutes on an Apple M4. Tests that render skip on a machine without a GPU and say why
- CI (`.github/workflows/ci.yml`) runs the same tasks on every pull request: on Ubuntu 24.04 and macOS (Apple silicon) `pixi run build`, `pixi run fetch-data` and `pixi run test` (the rendering tests on macOS only), on Windows `pixi run build`

### Platforms
- **macOS (Apple silicon):** everything
- **Linux (Ubuntu 24.04):** everything; the camera views and the rendering tests need a GPU
- **Windows:** `pixi run build` builds `rover_description` and `rover_control` natively, and RViz and the ROS 2 tools run in `pixi shell`. The Gazebo simulation runs in WSL2: install Ubuntu 24.04 (`wsl --install -d Ubuntu-24.04`), install pixi inside it, clone this repository into a Linux folder (for example `~/hurc_ws/src`, not under `/mnt/c`, which is slow) and follow the steps above there. WSLg shows Gazebo's window; open the address the driver station prints in a Windows browser

## Project Structure
Most of our work is structured into folders called "packages". Whenever you create a new package, please add a description of it here:
- `rover_description`: Contains the URDFs that define the rover's structure, joints, actuators, sensors, etc
- `rover_sim`: Contains SDFs, URDFs, launch configurations, and any other tools that help with simulating the rover in Gazebo. Today it holds the URC simulation: Gazebo plugins for the drivetrain and motors, the rocker differential and the driver station's cameras; the URC 2027 mission worlds and their generator; a browser driver station, a launcher and a referee (macOS and Linux)
- `rover_control`: The rover's drive code. For now the swerve design's kinematics library `rover_driver` with its tests and tools, superseded by tank drive; the ros2_control work replaces it

Beside the packages, `docs/` holds the design specs and plans (`docs/superpowers`), the simulation's manual (`docs/sim`) and the research behind it (`docs/research`).

## Contributing
Please do not push directly to the main branch. Any time you want to do work locally, please create a new branch with a descriptive name for that feature. To merge your work into the main branch, please create a pull request that can be reviewed by the team. (You can pull from your branch on the rover or base station any time you need to test with real hardware).
````

- [ ] **Step 2: Write `rover_sim/README.md` (the team's text, extended)**

````markdown
# Rover Sim
This package contains any useful tools, SDFs, or URDFs that are useful for simulating the rover in Gazebo. For example, this might contain simplified rover descriptions that abstract away the drivetrain or SDFs for each testing scenario we want to run in the simulator (obstacle courses, interactibles, entire missions, etc).

Today it holds the URC simulation: a model of our rover (rocker suspension with a differential, skid-steer drive with a DC motor per wheel, an RGB-D camera on a pan-tilt head), the URC 2027 mission worlds on MDRS ground, a proving ground, a browser driver station, a launcher and a referee. It runs on macOS and Linux (on Windows in WSL2). The manual is [docs/sim/manual.md](../docs/sim/manual.md), with its [design notes](../docs/sim/design-notes.md) and [Gazebo lessons](../docs/sim/gazebo-lessons.md).

### Quick start
From the repository's root (`<workspace>/src`), after `pixi install`:

```bash
pixi run fetch-data             # the terrain rasters, once (git-ignored)
pixi run sim                    # build the plugins, generate rover and worlds, open Gazebo (rover_test)
pixi run sim urc_autonomy       # a mission world (also urc_equipment_servicing, urc_delivery, urc_astrobiology, proving_ground)
pixi run drive urc_autonomy     # drive from the browser
pixi run launcher               # every world as a tile
pixi run test                   # the C++ tests and the simulation's suite
```

### Layout
| Path | What it is |
|---|---|
| `plugins/` | Gazebo system plugins (RoverDrivetrain, RockerDifferential, JointMonitor, ChaseCamera, FlyCamera) and their C++ tests; colcon builds them into the workspace's `build/rover_sim` and installs them to `lib/rover_sim` |
| `gen_model.py`, `viewers.py`, `models/` | The rover's parameters and its generated Gazebo model; the driver station's cameras |
| `gen_worlds.py`, `urc/`, `worlds/` | The URC world generator and the worlds (`rover_test.sdf` is tracked; the rest is generated and git-ignored) |
| `station/`, `launcher/`, `referee.py` | Driver station, launcher, URC referee |
| `gzenv.py`, `run.sh`, `bridge.yaml` | The Gazebo environment (the one place it is set), `pixi run sim`, the ROS 2 bridge's topics |
| `tools/` | Fetching the rasters, the patched rendering media, maps, the realism report, relief swatches |
| `data/` | Terrain data: the rasters (git-ignored, `pixi run fetch-data`, listed in `data/rasters.json`) and the `.json` that says where each came from, relief swatches, the soil map, the research files the code reads |
| `tests/` | The simulation's suite (`pixi run sim-test`) |

Comments and docstrings written before the simulation joined this repository call this folder `sim/` (read `rover_sim/`), cite `sim/README.md` (now the three files in `docs/sim/`) and `sim/data/research/` (now `docs/research/`, except the four files kept in `data/research/`).

### Launch Files
- `gazebo.launch.py`: Start Gazebo using the correct method for your operating system. Set the `world` argument to change which SDF is loaded

### SDF Worlds
- `worlds/rover_test.sdf`: test ground: a 10 cm step under the left wheels, a 15° ramp to a 0.3 m platform, rocks
- `worlds/urc_autonomy.sdf`, `urc_equipment_servicing.sdf`, `urc_delivery.sdf`, `urc_astrobiology.sdf`: the URC 2027 field missions at their real places near MDRS, and `proving_ground.sdf`, a test course; generated by `pixi run sim-worlds` (git-ignored). The manual describes them
````

- [ ] **Step 3: Write `rover_control/README.md`**

```markdown
# Rover Control
The rover's drive code. For now it holds `rover_driver`, the kinematics library of the swerve design (2D and 3D inverse and forward kinematics, steering). The rover switched to tank drive on 2026-10-05, so the library is superseded; it stays until the ros2_control work replaces it with the drive controller's configuration, the hardware interface and `rover.launch.py` (design spec `docs/superpowers/specs/2026-10-09-rover-software-integration-design.md`, section 6).

- `include/rover_driver/`, `src/`: the library (C++17, Eigen)
- `tools/rover_state.cpp`: the kinematics of one command as JSON; `tools/viz.py` draws them (`pixi run control-viz`)
- `tests/`: GoogleTest and CTest tests (`pixi run colcon-test`)
```

- [ ] **Step 4: Write `docs/research/README.md`**

```markdown
# Research

The measurements, prototypes and gate runs behind the simulation's design
(`docs/superpowers/specs/2026-10-06-urc-realism-design.md`), moved here from
`sim/data/research/` when the simulation joined this repository
(2026-10-09). The simulation reads none of it: the four research files its
code reads or writes stay in `rover_sim/data/research/`
(`terrain_targets.json`, `mdrs_terrain_measurements.json`,
`realism_report.json`, `realism_contact_sheet.jpg`).

- `gates.json`, `gates/`: the WS-0 gate runs G1 to G8 (physics speed per
  solver, the drivetrain prototype, the fly camera, rendering) and their
  scripts
- `drive/`: the drivetrain prototype (motor, contact and dig-in models) and
  its experiments
- `flycam/`: the fly camera prototype and its measurements
- `render/`: the rendering prototype, its steps and before/after pictures
- the rest: terrain, colour and micro-relief measurements of the MDRS and
  route areas (`lidar_microrelief.json`, `colour_stats.json`,
  `strata_colour_ramps.json`, the soil survey, close-ups and overviews)

Code comments cite these files as `sim/data/research/<file>`: read
`docs/research/<file>`. The gate and prototype scripts expect the tree as it
was before the move (they look for `sim/` beside them); run them from that
tree, the parent of the commit that moved it:
`git checkout $(git rev-list -1 HEAD -- sim/gen_model.py)^`.
```

- [ ] **Step 5: Cite the papers by reference only**

In `docs/superpowers/specs/2026-10-06-urc-realism-design.md` replace

```
16. Mandow et al. 2007, "Experimental kinematics for wheeled skid-steer mobile robots", IROS (`papers/SkidDrive.pdf`):
```

with

```
16. Mandow et al. 2007, "Experimental kinematics for wheeled skid-steer mobile robots", IROS:
```

replace

```
17. Baril et al. 2020, arXiv:2004.05131, skid-steer on sub-arctic terrain (`papers/SkidOnSubArctic.pdf`).
```

with

```
17. Baril et al. 2020, arXiv:2004.05131, skid-steer on sub-arctic terrain.
```

and replace

```
21. Toupet et al. 2020, J. Field Robotics 37:699 (`papers/Inverse3d.pdf`).
```

with

```
21. Toupet et al. 2020, J. Field Robotics 37:699.
```

- [ ] **Step 6: Check**

```bash
cd /Users/alarion239/Desktop/hurc_ws/src
git grep -n "papers/" -- . ':!docs/superpowers/specs/2026-10-09-rover-software-integration-design.md' ':!docs/superpowers/plans'
for f in README.md rover_sim/README.md rover_control/README.md docs/research/README.md docs/sim/*.md; do grep -o -E "\]\([^)]+\)" "$f" | sed -E 's/^\]\(|\)$//g' | grep -v '^http' | while read -r l; do [ -e "$(dirname "$f")/$l" ] || echo "broken link in $f: $l"; done; done
```

Expected: the grep prints nothing (only the integration spec and this plan mention `papers/`, as history); no broken link (the manual's only relative links are its two new files and `../urc_worlds.png`, checked on SOURCE's README while this plan was reviewed).

- [ ] **Step 7: Commit**

```bash
cd /Users/alarion239/Desktop/hurc_ws/src && git add README.md rover_sim/README.md rover_control/README.md docs/research/README.md docs/superpowers/specs/2026-10-06-urc-realism-design.md && git commit -q -F - <<'EOF'
Docs: READMEs for the packages, the simulation, tests and platforms

- README.md (the team's, extended): pixi run build beside colcon build,
  the simulation's quick start with fetch-data, tests and CI, platform
  notes (WSL2 for the simulation on Windows), rover_sim and rover_control
  in the package list, docs/.
- rover_sim/README.md (the team's, extended): quick start, layout, how to
  read the sim/ paths in old comments, the worlds.
- rover_control/README.md and docs/research/README.md are new.
- The realism spec cites the publishers' papers by reference only (papers/
  left the history).

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
EOF
```

### Task 17: Nothing of ours is silently ignored

**Files:** `.gitignore` only if the review finds something.

- [ ] **Step 1: Tracked files that an ignore rule matches**

```bash
cd /Users/alarion239/Desktop/hurc_ws/src && git ls-files -ci --exclude-standard && echo "--- none above means none"
```

Expected: nothing above `--- none above means none`.

- [ ] **Step 2: What is ignored in the working tree**

```bash
cd /Users/alarion239/Desktop/hurc_ws/src && git status --porcelain --ignored | sed -E 's#(rover_sim/models/urc_)[^/]+/#\1*/#' | sort | uniq -c
```

Expected: no `??` line (nothing untracked and unignored) and only `!!` lines for `.pixi/` content, `__pycache__/` folders, the ten rasters under `rover_sim/data/`, `rover_sim/models/urc_*/` and the generated `rover_sim/worlds/urc_*`, `proving_ground.*` and `*_map.*` files.

- [ ] **Step 3: The template's catch-alls against our tree**

```bash
cd /Users/alarion239/Desktop/hurc_ws/src
git ls-files | grep -E "(^|/)(bin|lib|build|install|log|logs|devel|msg|srv|cfg)/" | head
git ls-files | grep -E "\.(pcd|dox|wikidoc|user|cfgc)$" | head
git check-ignore -v --no-index rover_control/lib/x rover_sim/bin/x rover_sim/plugins/build/x
```

Expected: the first two print nothing (no tracked file under a folder or with an extension the ROS template ignores); the last shows the template's `lib/`, `bin/` and `build/` rules matching, which only matters if a package later adds such a folder: note it in the PR description for the team. If Step 1 or 2 showed a file of ours ignored, add a `!` rule for it under the simulation's block in `.gitignore` and commit with `.gitignore: keep <path> tracked`.

### Task 18: The team's README steps on a fresh clone

**Files:** none (a temporary workspace in `$T/freshws`, deleted at the end).

- [ ] **Step 1: Clone the branch into a new workspace's `src` and install**

```bash
T=/private/tmp/claude-502/rover-pr1
rm -rf "$T/freshws" && mkdir -p "$T/freshws"
git clone --branch import/simulation /Users/alarion239/Desktop/hurc_ws/src "$T/freshws/src"
cd "$T/freshws/src" && /Users/alarion239/.pixi/bin/pixi install --locked > "$T/fresh_install.log" 2>&1; tail -1 "$T/fresh_install.log"
```

Run the install in the background. Expected: `✔ The default environment has been installed.` (from pixi's cache: hardlinks, little new disk).

- [ ] **Step 2: `colcon build` from the workspace root, then source it**

```bash
T=/private/tmp/claude-502/rover-pr1
cd "$T/freshws" && /Users/alarion239/.pixi/bin/pixi run --manifest-path src/pixi.toml colcon build > "$T/fresh_build.log" 2>&1; tail -2 "$T/fresh_build.log"
/Users/alarion239/.pixi/bin/pixi run --manifest-path src/pixi.toml bash -c 'source install/local_setup.bash && ros2 pkg list | grep "^rover_"'
```

Expected: `Summary: 3 packages finished`; `rover_control`, `rover_description`, `rover_sim`.

- [ ] **Step 3: fetch-data on a fresh clone, dry run (no download), and a clean tree**

```bash
T=/private/tmp/claude-502/rover-pr1
cd "$T/freshws/src" && PYTHONDONTWRITEBYTECODE=1 /Users/alarion239/.pixi/bin/pixi run fetch-data --dry-run
git status --porcelain --ignored
```

(`PYTHONDONTWRITEBYTECODE=1`: otherwise importing `urc.dem` leaves an ignored `rover_sim/urc/__pycache__/` that the status check would list.)

Expected:
```
dem/far_dem_3dep_60km.tif: missing, would fetch from the official source, 1 request(s)
dem/mdrs_area_3dep.tif: missing, would fetch from the official source, 2 request(s)
dem/route_area_3dep.tif: missing, would fetch from the official source, 1 request(s)
dem/route_area_lidar_0p5m.tif: missing, would fetch from the team's copy
imagery/naip2021_far_60km.tif: missing, would fetch from the official source, 1 request(s)
imagery/route_area_naip2024.tif: missing, would fetch from the official source, 9 request(s)
```
and `git status` lists only `!! .pixi/...` (build/, install/ and log/ are in the workspace root, outside the repository).

- [ ] **Step 4: Delete the temporary workspace**

```bash
rm -rf /private/tmp/claude-502/rover-pr1/freshws && df -h /Users/alarion239/Desktop | tail -1
```

### Task 19: Hand-off: the data release command, cleanup, report

**Files:** none.

- [ ] **Step 1: The command that publishes the team's copy of the rasters (do not run it)**

Give the user this command in the report. It needs their approval: it creates a release and a tag on GitHub.

```bash
gh release create data-2026-10-06 --repo Harvard-HURC/Rover-Software --target main \
  --title "Terrain rasters (fetched 2026-10-06)" \
  --notes "The git-ignored rasters of rover_sim/data: USGS 3DEP elevation and lidar, USDA NAIP imagery (public domain). pixi run fetch-data takes them from here when their official source cannot rebuild them; rover_sim/data/rasters.json holds their SHA-256." \
  /Users/alarion239/Desktop/Rover/sim/data/dem/*.tif /Users/alarion239/Desktop/Rover/sim/data/imagery/*.tif
```

The assets keep their file names, which `rasters.json`'s `mirror` URL expects; the release does not depend on the pull request (its tag only marks `main`).

- [ ] **Step 2: The suites on the branch as it will be pushed**

Task 12 ran them before the launch-file fix, its test (Task 13) and the documents. Run `pixi run test` once more on the final branch, in the background (about 15 minutes):

```bash
T=/private/tmp/claude-502/rover-pr1
cd /Users/alarion239/Desktop/hurc_ws/src && GZ_IP=127.0.0.1 GZ_PARTITION=pr1_final_$$ /Users/alarion239/.pixi/bin/pixi run test > "$T/final_test.log" 2>&1; echo "exit $?" >> "$T/final_test.log"
```

Then:

```bash
T=/private/tmp/claude-502/rover-pr1
grep -E "tests passed|^Summary" "$T/final_test.log"; grep -E "^Ran |^OK|^FAILED|^exit" "$T/final_test.log"
cd /Users/alarion239/Desktop/hurc_ws/src && git status --porcelain && echo "tree clean"
```

Expected: as in Task 12 Step 3, with `Ran 508 tests` (Task 13 added `test_launch_files`), `OK (skipped=9, expected failures=2)`, `exit 0`, then `tree clean`. A failure here is handled as in Task 12 Step 3.

- [ ] **Step 3: SOURCE untouched, the branch complete, nothing pushed**

```bash
cd /Users/alarion239/Desktop/Rover && git status --porcelain && git log --oneline -1
cd /Users/alarion239/Desktop/hurc_ws/src && git status --porcelain && git log --oneline origin/main..import/simulation | head -15
git branch -vv | grep import/simulation
git ls-remote --heads origin import/simulation | wc -l
```

Expected: SOURCE clean, its last commit a commit of this plan (`Plan PR 1: critic fixes...` or a later revision of the plan); SRC clean; the branch's newest commits are the ones of Tasks 2 to 17 (merge, layout x2, pixi, gzenv, rendering skips, paths, fetch-data, launch fix, CI, docs x2, and Task 17's if it found something); `import/simulation` has no upstream; `0` (the branch does not exist on GitHub).

- [ ] **Step 4: Remove the temporary files**

```bash
rm -rf /private/tmp/claude-502/rover-pr1 && ls /private/tmp/claude-502 | grep -c rover-pr1
```

Expected: `0`. The workspace keeps its environment (`src/.pixi`), `build/`, `install/`, `log/`, the generated worlds and the raster symlinks.

- [ ] **Step 5: Report to the user**

Report: the branch and its commits; the test results of Task 12 and Step 2 (counts, skips, expected failures, any failure that SOURCE shares); the compiler the build used (Task 5 Step 3); the fresh-clone result; that nothing is pushed; and the decisions that need them: the data release (Step 1; without it fetch-data cannot get `route_area_lidar_0p5m.tif` on a fresh clone and CI stops at its fetch step, after the build), and the push of `import/simulation` with the pull request (title "Import the simulation", description from the commits and the acceptance list below). The description also says:

- Merge with **Create a merge commit**, not squash or rebase (the repository allows all three). A squash or a rebase would leave the imported history out of `main` (spec 9.1: every commit with its message and author, `git log --follow` and blame back to `sim/` and `driver/`) and break `docs/research/README.md`'s command for the tree before the move.
- Review commit by commit: against `main`, which has no `sim/`, the Files tab shows about 700 added files; the renames show only in the moves-only commit (`Layout: sim/ becomes rover_sim/ ...`, 678 renames, no line changed).
- Bisecting: the pixi tasks do not run on the three commits from the moves-only commit (47fdf57) to `pixi: three platforms ...` (a5cf8eb): their tasks or `.gitignore` name the old paths, or gzenv does not yet look in colcon's build directory, as it does from 6ad7091 on. Skip them (`git bisect skip`), or bisect `main` with `git bisect --first-parent` once the merge commit is there.
- `.pixi/config.toml` comes with the simulation's history and lets the packages run their post-link scripts: graphviz, gtk3 and gdk-pixbuf, from the team's `ros-jazzy-desktop`, build their caches with them (without graphviz's, `dot`, rqt_graph and view_frames fail). The simulation needs none of them.
- Until the data release exists, fetch-data cannot get `route_area_lidar_0p5m.tif`: the Ubuntu and macOS jobs stop at their fetch step, and a teammate links the rasters from a clone that has them (`rover_sim/tools/link_data.sh <that clone>`, README).

---

## Acceptance (spec 9.2, row 1)

| Criterion | Where it is shown |
|---|---|
| History imported without `papers/`, every commit's message and author kept, team's `main` an ancestor | Task 1 Step 3, Task 2 Step 7 |
| `sim/` to `rover_sim/` (ament_cmake), `driver/` to `rover_control/` unchanged, research to `docs/research`, renames detected | Task 3 (678 R100), Task 4 |
| The team's README steps work (clone into `<ws>/src`, `pixi install`, `colcon build`) | Task 18 |
| `colcon build` builds the three packages; plugins installed | Task 5 |
| pixi resolves osx-arm64, linux-64 and win-64; one manifest, tasks from the repository root | Task 6 |
| The full simulation suite passes on this Mac, with no change in behaviour | Task 12 (507 tests, 9 skipped, 2 expected failures; SOURCE's 495 among them), Task 19 Step 2 on the final branch (508) |
| rover_control's tests pass | Task 5 Step 5, Task 12 and Task 19 Step 2 (57 tests) |
| fetch-data works on a fresh clone without `--force` (logic and dry run) | Task 11, Task 18 Step 3 |
| The team's launch-file bug fixed | Task 13 |
| CI file valid | Task 14 Step 2 (CI green on GitHub needs the push and the data release) |
| README extended; `sim/README.md` split into `rover_sim/README.md` and `docs/sim/` without losing content | Tasks 15 and 16 |
| Nothing of ours ignored | Task 17 |

## Risks

| Risk | Handling |
|---|---|
| The 0.5 m lidar DEMs (one of them required by `urc_autonomy`) cannot be rebuilt from official sources with code in the repository | fetch-data takes them from the team's release, which needs the user's approval (Task 19); until then CI fails at fetch-data. The alternative is a later PR that scripts the lidar processing from the USGS tiles and point clouds |
| The official ImageServers may return other pixels later (3DEP is a dynamic mosaic) or be down | fetch-data checks the content and falls back to the team's copy; downloads were not run for this plan (offline tests only) |
| Linux and Windows are built for the first time in CI: GCC on the plugins, MSVC on rover_control, numeric tolerances on Linux, wall-clock-sensitive tests on slower runners | Nothing can be built for them locally (no Docker); expect a fix-up commit after the first CI run |
| conda's clang 21 may replace Apple's compiler for the plugins and rover_control (the team's `ros-dev-tools` brings `cxx-compiler`) | Task 5 Step 3 records which one CMake took; the full suite (Task 12) decides; a test that passes in SOURCE and fails here is treated as a regression, and Task 12 Step 3 says how to tell a compiler effect |
| macOS runners may not render with ogre2 (virtualised GPU), the suite may take over an hour there (3 cores), and 7 GB of memory is tight for generating the 4097² Autonomy world and rendering | `ROVER_RENDERING: "0"` in the macOS matrix entry is the fallback for rendering; timeout 150 minutes; a larger runner if memory runs out |
| The suite always runs Gazebo transport on 127.0.0.1 (simulate.py's default `GZ_IP`), which was only ever run on macOS; Linux's loopback interface has no MULTICAST flag, which gz-transport's discovery may need | If the Ubuntu job's simulations hear nothing from Gazebo, add a step `sudo ip link set lo multicast on` before `pixi run test` (GitHub's runners allow sudo) and say so in the README's Linux note |
| The team may object to dropping `ament_lint_auto` in `rover_sim` | Stated in the CMake file and the commit; re-enabling is a few lines once the code is brought to ROS style |
| The team's `main` moves before the push | Task 2 Step 1 stops if it moved; at push time a new merge of `main` may be needed |
| A pull request with about 700 renames and a 1.5 MB lock is hard to review | The move commit is renames only; the other commits are small and described |
