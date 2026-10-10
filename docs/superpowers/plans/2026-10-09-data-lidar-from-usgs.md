# Data PR: the lidar DEMs and the NAIP 2021 mosaic from USGS Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** On the branch `data/lidar-from-usgs` (from `import/simulation`), `pixi run fetch-data` rebuilds the four rasters it cannot rebuild today from official USGS data: the two 0.5 m lidar DEMs and the lidar source map by a script from USGS's own lidar tiles and point clouds, the NAIP 2021 route mosaic from its USGS ImageServer requests. Each rebuild is checked against `rover_sim/data/rasters.json` (bit-identical on the Mac that made the manifest, within a stated tolerance elsewhere), so every raster is official and "link it from a teammate" leaves the documentation.

**Architecture:** A new module `rover_sim/tools/usgs_lidar.py` holds the two recipes (`mdrs_area`, `route_area`): the steps that made the files on 2026-10-06, recovered from the research transcript and kept verbatim, with GDAL's command-line tools, laspy, OpenCV and numpy. `tools/fetch_data.py` stays the one entry point: a raster whose provenance names a recipe is rebuilt from the USGS files that provenance lists, each downloaded file checked against its size and SHA-256; the result is kept if its content matches the manifest, or, not bit-identical, if its stats (min, max, 16 x 16 block means) are within 1e-3. The NAIP 2021 mosaic needs no new code: its four exact exportImage requests go into its provenance.

**Tech Stack:** Python 3.12 (unittest), GDAL 3.13.3 command-line tools (gdalbuildvrt, gdalwarp, gdal_translate), PROJ 9.8.1, laspy 2.7.0 with lazrs-python 0.8.2, OpenCV 4.13.0, numpy 2.5.3, Pillow; pixi 0.81 (RoboStack Jazzy channel, which serves conda-forge); GitHub Actions (unchanged jobs).

---

## Places and rules

| Name | Path |
|---|---|
| WT | `/Users/alarion239/Desktop/hurc_ws/wt-data`: the git worktree of this plan, branch `data/lidar-from-usgs`, created from `import/simulation` at `9208047` |
| SRC | `/Users/alarion239/Desktop/hurc_ws/src`: the main checkout (branch `pr2/rover-description`, PR 2's work). Read-only for this plan: `link_data.sh` reads its ignored files, and Task 14 clones from its repository |
| SOURCE | `/Users/alarion239/Desktop/Rover`: read-only. SRC's raster symlinks point into it |
| S | `/private/tmp/claude-502/-Users-alarion239-Desktop-Rover/d048031d-a613-49e0-ad24-8cb3b1a40ce3/scratchpad/data`: downloads (`S/sources`), work directories, scripts (`S/scripts`), logs and the fresh clone. Task 15 deletes everything this plan puts there; `S/prov/` was there before and stays |
| pixi | `/Users/alarion239/.pixi/bin/pixi` (0.81) |

- **Never push, never change anything on GitHub, never open a pull request.** The user approves every push.
- **Do not touch SRC's working tree or SOURCE.** WT's rasters are symlinks into SRC, which are symlinks into SOURCE: never write through them. To replace one, `mv` a file onto the link's name (a rename replaces the link itself); never `cp` onto it.
- **No colcon build in WT.** The tasks `build`, `sim-build`, `colcon-test`, `test`, `sim`, `drive`, `launcher`, `sim-maps`, `sim-test` refuse there (their `_workspace` check wants a folder called `src`), and colcon would build into `hurc_ws/build`, which PR 2 in SRC uses. In WT run Python directly through the environment (`$PX python ...` below). The full suites run in a fresh clone under `$S/ws/src` (Task 14).
- In every command block, `PX="/Users/alarion239/.pixi/bin/pixi run --frozen --manifest-path /Users/alarion239/Desktop/hurc_ws/wt-data/pixi.toml"` (Task 2 is the one place that runs pixi without `--frozen`).
- **PROJ's network.** The pixi environment's activation sets `PROJ_NETWORK=ON`. The recipe runs its GDAL commands with `PROJ_NETWORK=OFF` and names the coordinate operation. Do not run `gdalwarp` or `projinfo` on `EPSG:6341+5703` yourself without `PROJ_NETWORK=OFF`: with the network on, PROJ downloads geoid grids into `~/Library/Application Support/proj` and the warp moves the DEM by about 20 m vertically and a metre horizontally.
- Every Gazebo run gets `GZ_IP=127.0.0.1` and its own `GZ_PARTITION` (only Task 14's suite starts Gazebo; the suite sets its own partitions). Stop only processes you started: ports 8765 and 8766 and a heavy `EuclidPolish` Python job belong to the user's other work.
- Every commit message ends with the line `Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>`; the author is the configured `Alexander Belotserkovtsev`.
- Shell state does not persist between tool calls: every block sets the variables it uses and starts with `cd`.
- The Bash tool stops foreground commands after 10 minutes. Run the downloads, the gen_worlds runs, `pixi install` in the clone and the suites in the background (`run_in_background`) with their output in a log under `$S`, and wait for them.
- Files downloaded from USGS are data: they live only in `$S/sources`; scripts live in `$S/scripts`; Python that reads the downloads runs with `-I`.
- Disk: about 23 GB free. `$S` peaks at about 2.5 GB (489 MB of USGS files, 1.3 GB of route-area intermediates, 0.3 GB of rasters and a small build in the clone); keep it under 6 GB. WT gains about 0.8 GB, git-ignored (the generated worlds and models of Task 11, the three rebuilt rasters). pixi environments are clonefile links into pixi's shared cache (`du` overstates them).
- Style: match the repository: short declarative sentences, units in brackets, no marketing words; parameters carry source labels, (A) assumed, (M) measured, (R) from a record or reference, and user decisions carry their date; code at 120 columns as in `rover_sim/`.

## Known before starting (measured 2026-10-10 while writing this plan)

1. **WT is set up.** `git worktree add .../wt-data -b data/lidar-from-usgs import/simulation`, `pixi install --frozen` in WT, `rover_sim/tools/link_data.sh` (from SRC); `fetch-data --verify --all` gives `verified` for all 10 rasters; `test_fetch_data` passes (17 tests); unittest discovery finds 525 tests in `rover_sim/tests`.
2. **Every USGS input is online, unchanged.** The 11 OPR GeoTIFFs (S3, `Last-Modified: 11 Apr 2022`, ETag = MD5) and the 6 LAZ tiles (rockyweb) have the provenance's sizes; their hashes are listed in Task 3 Step 2. 489,363,946 bytes in all (the research summary's "566 MB" counted the 5 shared OPR tiles twice): 371,452,622 for the route area, 194,685,792 for the MDRS area. The downloads took 218 s (rockyweb gives about 0.3 MB/s per connection).
3. **The recipes of this plan reproduce the files byte for byte on osx-arm64.** `usgs_lidar.py` as listed in Tasks 5 to 7, run on those 17 files in a copy of WT's environment with laspy 2.7.0 and lazrs-python 0.8.2 added (GDAL 3.13.3, PROJ 9.8.1, OpenCV 4.13.0, numpy 2.5.3), gave all three rasters with the manifest's file SHA-256 (so also its content_sha256): `mdrs_area` in 6 s, `route_area` in 12 s, 2.9 GB peak memory. `fetch_data.py` as listed in Tasks 8 to 10, with the manifest of Task 12, rebuilt and installed all three in 18 s and `--verify` passed.
4. **The source CRS must stay `EPSG:6341+5703`.** On OPR tile 12SWH1752, the MDRS warp with the compound CRS and `PROJ_NETWORK=OFF` (or with `-ct` and the null pipeline) matched the file on all 108,424 interior pixels; with the 2D `EPSG:6341` (the research summary's recommendation 4), 1,879 of them differed by one float32 step (1.2e-4 m), with or without the network; with the network on (the environment's default) every pixel differed (geoid and datum grids). PROJ offers one operation offline for both source CRSs, the null pipeline in `usgs_lidar.UTM_TO_LONLAT`; GDAL rounds the 2D case differently. For the 3DEP fill warp (EPSG:4326 to EPSG:6341) `-ct LONLAT_TO_UTM` gives the same MD5 as without it.
5. **The route probe.** OPR 12SWH2052 and LAZ 12SWH2052 alone through `route_area` matched the file on every interior 2018-only pixel (1,214,259) and on every 2020-only pixel farther than 10 px from the probe's edge (902,386); the differences sat within 6 px of the probe's edge (the fill and the distance transforms see the missing neighbour tiles).
6. **The OPR tiles do not overlap.** Their extents (from their headers): 2 km tiles 1648, 1650, 1848, 1850, 2048, 2050 (x 516-522 km by 2 km, y 4248-4252 km) and 1 km tiles 1652, 1752, 1852, 1952, 2052 (y 4252-4253 km). Only 1850, 1852, 1952, 2050, 2052 intersect the route grid (518000-521000 E x 4250000-4254000 N); with nearest resampling and no reprojection the VRT's other tiles cannot change a pixel of it.
7. **NAIP 2021.** A 200 x 200 request with `mosaicRule` lockRaster 195681, encoded as in Task 4, returned the file's top-left 200 x 200 pixels exactly.
8. **pixi.** Adding the four dependencies of Task 2 resolves with the lock's preferences: the lock gains laspy 2.7.0 and lazrs-python 0.8.2 for linux-64 and osx-arm64 and the noarch m2r2 2.1.0 and mistune 3.3.4 (laspy's dependencies), 92 lines; no other package changes. `libgdal-core` ships the GDAL command-line tools; `proj` ships `projinfo` and `proj`.
9. **The research summary garbled one hash.** It quotes the route DEM's content_sha256 as `0d2da6c0...51cf2fbd01d3d8f3d254964fc8a811`; rasters.json has `0d2da6c0449e0690788c634e454d7f69651cf2fbbd04313a8e677934c352d3db`. rasters.json is right (Known 3).
10. A side effect of writing this plan: the probe with the network on created `~/Library/Application Support/proj/cache.db` (1.4 MB, PROJ's grid cache). This plan never runs PROJ with the network on.

## Decisions this plan makes

1. **One recipe module, fetch-data as the one entry point.** `tools/usgs_lidar.py` turns local USGS files into rasters (no network). `tools/fetch_data.py` downloads and checks the files, runs the recipe a raster's provenance names (`"recipe": "mdrs_area"` or `"route_area"`), and verifies and installs the result. A recipe that makes two rasters (the route DEM and its source map) runs once per fetch-data run. There is no second command and no new pixi task.
2. **The steps are the 2026-10-06 steps, verbatim** (the scratch copies `S/prov/scripts/{grid2020.py,blend.py,warp_route.sh,warp_mdrs.sh}` and the transcript `agent-a096fbd64515bdc6d.jsonl` lines 361-374 and 193-196): the class-2 cell mean with float64 `np.bincount` sums in the LAZ tiles' order, the coverage of 4 m blocks closed by one block, the normalized-Gaussian fill with sigma 0.7, 1.5, 3, 6, 12, 25, the distance-transform feathering with float64 weights, the EHdr BIL with the pixel-centre origin, the shift constants -0.8925 m E / +0.6175 m N, Python's float strings for `-te` and `-a_ullr`, the compound source CRS of the DEM warps (Known 4). The one addition: every warp names its coordinate operation (`-ct`, the null pipelines PROJ picks offline) and runs with `PROJ_NETWORK=OFF`, so no network or local grid can change it.
3. **The route recipe reads 5 OPR tiles, not 11.** The 2026-10-06 VRT held all 11, but only 5 intersect the route grid and the warp onto it is a nearest copy without reprojection (Known 6), so the other 6 cannot change a pixel; Known 3 confirms the bytes. A default `fetch-data` run downloads 371 MB instead of 489 MB.
4. **Every USGS file is pinned by size and SHA-256** in the provenance (rockyweb's ETags are no checksums); the OPR tiles also keep their MD5, S3's ETag, as USGS's own checksum. A file whose bytes differ fails the rebuild before any processing; a file already in the sources directory with the right SHA-256 is used again.
5. **Tolerance where bits cannot match.** The manifest gets `stats` for the three lidar rasters: min, max and the means of 16 x 16 blocks, rounded to 1e-6. A rebuild whose content_sha256 differs but whose shape, georeferencing and stats are within `TOLERANCE = 1e-3` (1 mm for the DEMs, a thousandth of a code for the map) is kept, and fetch-data says `content within 0.001 of rasters.json's stats, not bit-identical`; `--verify` and `matches` accept it the same way. Platform rounding (OpenCV's blur, GDAL's bilinear kernel with or without fused multiply-add) is a few float32 steps, about 0.1 mm; a wrong tile, shift or step is centimetres. The rasters' hashes stay those of the 2026-10-06 files, made on osx-arm64; their provenance says so (`rebuilt`).
6. **NAIP 2021 through the existing exportImage path.** Its provenance gets the four exact requests (bbox of 2026-10-06, `size=2600,2800`, `bandIds=0,1,2`) with `mosaicRule` lockRaster 195681, so a newer NAIP year on the service cannot replace it; the 2026-10-06 requests were the same without the rule (recorded in `mosaic_rule`).
7. **The "cannot rebuild" path stays, unused.** After this PR every raster is official. `world_missing`, `needs_world`, `skip_if_missing`, gen_worlds' skip and their tests stay as the path for a future raster that has no script yet (removing them would touch 15 test files for no change in behaviour); their message says "link it from a clone that has it" instead of "a teammate's clone", and `test_urc_unit.MissingRaster` uses a made-up entry. The failure hint of fetch-data (a clone that has the rasters can link them when a source is down) stays.
8. **CI keeps its jobs and its raster cache.** `pixi run fetch-data` (the required rasters) now also rebuilds the Autonomy DEM, on a cache miss only: 371 MB from USGS and about 20 s of processing; the cache, keyed by rasters.json, holds the result. CI does not run `--all` (the MDRS DEM, the source map and the research imagery are read by no world or test); Task 11 and Task 14 rebuild them here.
9. **pixi: four dependencies, macOS and Linux only** (`[target.unix.dependencies]`, where the simulation's dependencies are): `libgdal-core` and `proj` declared explicitly with their minor versions pinned (GDAL is in the environment today only through gz-common5; PROJ 9.9 is on conda-forge), `laspy` and `lazrs-python`. numpy and OpenCV keep their ranges: a newer one can only cost bit-identity, which the tolerance covers.
10. **Sources.** USGS only (the user, 2026-10-09: no data release; the lidar DEMs rebuilt from USGS's public lidar): The National Map's S3 bucket for the OPR tiles, rockyweb for the point clouds, the USGS NAIP Plus ImageServer for NAIP 2021. USDA and UGRC are not needed. The AWS EPT copy of the 2020 point clouds (Z and class identical, XY within 0.4 mm) is a documented fallback in the risks, not code.

## File structure

Created:
- `rover_sim/tools/usgs_lidar.py`: the two recipes and their steps.
- `rover_sim/tests/test_usgs_lidar.py`: the grids against the manifest and provenance, the steps on small arrays, both recipes end to end on made-up tiles.

Modified:
- `rover_sim/tools/fetch_data.py`: stats and tolerance (`stats`, `within`, `verdict`, `check`), pinned downloads (`source_files`), recipes (`recipe_of`, `inputs_of`, `Rebuilds`, `plan`, `fetch`, `main --sources`), the neutral `LINK`.
- `rover_sim/tests/test_fetch_data.py`: `Stats`, `Sources`, `Lidar`; the manifest tests for recipes; the dry run's Autonomy line; `LINK`.
- `rover_sim/tests/test_urc_unit.py`: `MissingRaster` with a made-up unofficial raster and a test that the Autonomy DEM is fetched like the others.
- `rover_sim/gen_worlds.py`: docstring only.
- `rover_sim/data/rasters.json`: four entries official, three with stats, `what`.
- `rover_sim/data/dem/mdrs_area_lidar2018_0p5m.json`, `rover_sim/data/dem/route_area_lidar_0p5m.json`: pinned source files, `recipe`, `rebuilt`.
- `rover_sim/data/imagery/naip2021_route_area.json`: `requests` and `mosaic_rule` instead of `url_template`.
- `pixi.toml`, `pixi.lock`: four dependencies; the fetch-data comment.
- `README.md`, `rover_sim/README.md`, `docs/sim/manual.md`, `docs/research/README.md`, `.github/workflows/ci.yml` (a comment).

Unchanged on purpose: `rover_sim/urc/` (its sources key the media caches), the station, `run.sh`, `worldfiles.py`, `link_data.sh`, CI's jobs and cache keys.

---

## Part A: environment and inputs

### Task 1: The worktree, its environment and rasters

**Files:** none.

- [ ] **Step 1: Check what the plan's author set up**

```bash
cd /Users/alarion239/Desktop/hurc_ws/wt-data && git status --short --branch && git log --oneline -1 && git rev-parse --short import/simulation
ls -l rover_sim/data/dem/*.tif rover_sim/data/imagery/*.tif | awk '{print $9, $10, $11}' | head -3
ls .pixi/envs/default/bin/gdalwarp
```

Expected: `## data/lidar-from-usgs`, no changes; the last commit is this plan's (`Lidar from USGS: the plan`), on top of `9208047` (which `import/simulation` prints); the rasters are symlinks into `/Users/alarion239/Desktop/hurc_ws/src/rover_sim/data/...`; the environment has `gdalwarp`. If the worktree or the environment is missing, create them: `git -C /Users/alarion239/Desktop/hurc_ws/src worktree add /Users/alarion239/Desktop/hurc_ws/wt-data -b data/lidar-from-usgs import/simulation`, then in WT `/Users/alarion239/.pixi/bin/pixi install --frozen` and `bash rover_sim/tools/link_data.sh`.

- [ ] **Step 2: The rasters and the fetch-data tests**

```bash
PX="/Users/alarion239/.pixi/bin/pixi run --frozen --manifest-path /Users/alarion239/Desktop/hurc_ws/wt-data/pixi.toml"
cd /Users/alarion239/Desktop/hurc_ws/wt-data && $PX python rover_sim/tools/fetch_data.py --verify --all; echo "exit $?"
cd rover_sim/tests && $PX python -m unittest test_fetch_data 2>&1 | tail -3
mkdir -p /private/tmp/claude-502/-Users-alarion239-Desktop-Rover/d048031d-a613-49e0-ad24-8cb3b1a40ce3/scratchpad/data/scripts
```

Expected: ten lines `<raster>: verified`, `exit 0`; `Ran 17 tests`, `OK`.

### Task 2: pixi: GDAL's tools, PROJ and laspy

**Files:** Modify `pixi.toml` (`[target.unix.dependencies]`), `pixi.lock`.

- [ ] **Step 1: A test that the environment lacks laspy**

```bash
PX="/Users/alarion239/.pixi/bin/pixi run --frozen --manifest-path /Users/alarion239/Desktop/hurc_ws/wt-data/pixi.toml"
cd /Users/alarion239/Desktop/hurc_ws/wt-data && $PX python -I -c "import laspy, lazrs; print(laspy.__version__)"
```

Expected: `ModuleNotFoundError: No module named 'laspy'`.

- [ ] **Step 2: Declare the four dependencies**

In `pixi.toml`, after the line `aiohttp = ">=3.14.3,<4"` (the last line of `[target.unix.dependencies]`), add:

```toml
# fetch-data's lidar rebuild (tools/usgs_lidar.py): GDAL's command-line tools and PROJ (until now only there through
# gz-common5) and the LAZ reader. rasters.json's hashes of the lidar DEMs were made with GDAL 3.13.3 and PROJ 9.8.1
# (R: their provenance); another minor version may round differently, which fetch-data then accepts only within its
# tolerance, so the minor versions are pinned.
libgdal-core = ">=3.13.3,<3.14"
proj = ">=9.8.1,<9.9"
laspy = ">=2.7.0,<3"
lazrs-python = ">=0.8.2,<0.9"
```

- [ ] **Step 3: Update the lock and the environment**

```bash
S=/private/tmp/claude-502/-Users-alarion239-Desktop-Rover/d048031d-a613-49e0-ad24-8cb3b1a40ce3/scratchpad/data
cd /Users/alarion239/Desktop/hurc_ws/wt-data && git show HEAD:pixi.lock > "$S/pixi.lock.before" && /Users/alarion239/.pixi/bin/pixi install > "$S/install.log" 2>&1; tail -1 "$S/install.log"
diff <(grep -o 'conda: https://[^ ]*' "$S/pixi.lock.before" | sort -u) <(grep -o 'conda: https://[^ ]*' pixi.lock | sort -u); git diff --stat pixi.lock
```

Expected: `✔ The default environment has been installed.`; the diff shows only added lines, `laspy-2.7.0-...` and `lazrs-python-0.8.2-...` for `linux-64` and `osx-arm64` and `noarch/m2r2-2.1.0-...` and `noarch/mistune-3.3.4-...` (newer builds of the same versions are fine); `pixi.lock | 92 +++...` or close to it, insertions only. If any line starts with `<` (a package removed or changed), stop: pixi re-solved more than it should; `git checkout pixi.lock` and retry `pixi install`.

- [ ] **Step 4: Run the Step 1 command again**

```bash
PX="/Users/alarion239/.pixi/bin/pixi run --frozen --manifest-path /Users/alarion239/Desktop/hurc_ws/wt-data/pixi.toml"
cd /Users/alarion239/Desktop/hurc_ws/wt-data && $PX python -I -c "import laspy, lazrs, cv2, numpy; print(laspy.__version__, cv2.__version__, numpy.__version__)"
$PX bash -c 'gdalinfo --version; proj 2>&1 | head -1; echo PROJ_NETWORK=$PROJ_NETWORK'
```

Expected: `2.7.0 4.13.0 2.5.3`; `GDAL 3.13.3 "Iowa City", released 2026/08/13`; `Rel. 9.8.1, April 10th, 2026`; `PROJ_NETWORK=ON` (the activation's; the recipe overrides it).

- [ ] **Step 5: Commit**

```bash
cd /Users/alarion239/Desktop/hurc_ws/wt-data && git add pixi.toml pixi.lock && git commit -q -F - <<'EOF'
Lidar from USGS: pixi gets GDAL's tools, PROJ and laspy

fetch-data's lidar rebuild runs gdalbuildvrt, gdalwarp and gdal_translate
(libgdal-core, until now only there through gz-common5), PROJ's projinfo
and laspy with lazrs-python. GDAL and PROJ are pinned to the minor versions
that made the lidar DEMs (3.13.3, 9.8.1). macOS and Linux only, like the
rest of the simulation. The lock gains laspy, lazrs-python, m2r2 and mistune;
nothing else changes.

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
EOF
git log --oneline -1
```

### Task 3: Download the USGS files and pin them in the provenance

**Files:** Modify `rover_sim/data/dem/mdrs_area_lidar2018_0p5m.json`, `rover_sim/data/dem/route_area_lidar_0p5m.json`. Scratch: `$S/sources/` (the 17 files, kept until Task 15), `$S/scripts/hashes.py`, `$S/scripts/pin_sources.py`.

- [ ] **Step 1: Download the 11 OPR tiles and the 6 LAZ tiles (background, about 4 minutes)**

```bash
S=/private/tmp/claude-502/-Users-alarion239-Desktop-Rover/d048031d-a613-49e0-ad24-8cb3b1a40ce3/scratchpad/data
mkdir -p "$S/sources" && cd "$S/sources"
OPR=https://prd-tnm.s3.amazonaws.com/StagedProducts/Elevation/OPR/Projects/UT_StateWide_2018_A18/UT_Southern_QL1_2018/TIFF
LPC=https://rockyweb.usgs.gov/vdelivery/Datasets/Staged/Elevation/LPC/Projects/UT_StatewideSouth_2020_A20/UT_StatewideSouth_3_2020/LAZ
start=$(date +%s)
for t in 1648 1650 1652 1752 1848 1850 1852 1952 2048 2050 2052; do
  curl -sS --fail --retry 3 -o USGS_OPR_UT_StateWide_2018_A18_12SWH$t.tif "$OPR/USGS_OPR_UT_StateWide_2018_A18_12SWH$t.tif" &
done
for t in 1852 1853 1952 1953 2052 2053; do
  curl -sS --fail --retry 3 --max-time 1800 -o USGS_LPC_UT_StatewideSouth_2020_A20_12SWH$t.laz "$LPC/USGS_LPC_UT_StatewideSouth_2020_A20_12SWH$t.laz" &
done
wait; echo "took $(( $(date +%s) - start )) s"; ls | wc -l
```

Run it with `run_in_background` and wait for it. Expected: `took` about 200-300 s, `17`.

- [ ] **Step 2: Sizes, MD5 and SHA-256; the OPR tiles' ETags**

Create `$S/scripts/hashes.py`:

```python
"""Size, MD5 and SHA-256 of every downloaded USGS file: python -I hashes.py <sources dir> <hashes.json>"""
import hashlib
import json
import sys
from pathlib import Path

out = {}
for path in sorted(Path(sys.argv[1]).iterdir()):
    if path.suffix in (".tif", ".laz"):
        data = path.read_bytes()
        out[path.name] = {"size_bytes": len(data), "md5": hashlib.md5(data).hexdigest(),
                          "sha256": hashlib.sha256(data).hexdigest()}
        print(path.name, len(data), out[path.name]["md5"], out[path.name]["sha256"])
Path(sys.argv[2]).write_text(json.dumps(out, indent=1) + "\n")
print(len(out), "files,", sum(v["size_bytes"] for v in out.values()), "bytes")
```

Run it, then compare the OPR MD5s with S3's ETags:

```bash
S=/private/tmp/claude-502/-Users-alarion239-Desktop-Rover/d048031d-a613-49e0-ad24-8cb3b1a40ce3/scratchpad/data
PX="/Users/alarion239/.pixi/bin/pixi run --frozen --manifest-path /Users/alarion239/Desktop/hurc_ws/wt-data/pixi.toml"
cd "$S" && $PX python -I scripts/hashes.py sources hashes.json
OPR=https://prd-tnm.s3.amazonaws.com/StagedProducts/Elevation/OPR/Projects/UT_StateWide_2018_A18/UT_Southern_QL1_2018/TIFF
for t in 1648 1650 1652 1752 1848 1850 1852 1952 2048 2050 2052; do
  printf '%s ' $t; curl -sSI "$OPR/USGS_OPR_UT_StateWide_2018_A18_12SWH$t.tif" | tr -d '\r' | awk 'tolower($1)=="etag:" {gsub(/"/, "", $2); print $2}'
done > etags.txt
$PX python -I -c "
import json, sys
h = json.load(open(sys.argv[1]))
bad = [t for t, etag in (line.split() for line in open(sys.argv[2])) if h[f'USGS_OPR_UT_StateWide_2018_A18_12SWH{t}.tif']['md5'] != etag]
print('ETags match the MD5s' if not bad else f'ETag MISMATCH: {bad}')" hashes.json etags.txt
```

Expected (as measured on 2026-10-10):

```
USGS_LPC_UT_StatewideSouth_2020_A20_12SWH1852.laz 46617883 cf46239fb66ec60f396a06417df74710 3268f97b1e41deef9515318cefddd1845d138b1543df1370bb081ff2827aaebb
USGS_LPC_UT_StatewideSouth_2020_A20_12SWH1853.laz 61371872 972c83f2c0b792f303fc9b7b208fafdc c23be61ba9132eb96756991c62fce3cf2dc525a7afc8b78ccd609a63f4b1a847
USGS_LPC_UT_StatewideSouth_2020_A20_12SWH1952.laz 37346057 d7f7146b906863ed37a5d6dfde2aa06c f190ff9cde49f0e5c074582024f7d7563e3c496cade002617c9b360f9e64d1f7
USGS_LPC_UT_StatewideSouth_2020_A20_12SWH1953.laz 60917504 9076ca0a7fd2192a657c6ee92afd67ce 181dda69ab6590e5d20ac16296107e7847f077ac32d5e4c0dd5609bb5615960b
USGS_LPC_UT_StatewideSouth_2020_A20_12SWH2052.laz 27455881 189cbfc4dc2e3f4974857633da613869 12f4ba5be267ffd16586b123c24e3b9fc9f46f2fbd01d3d8f3d254964fc8a811
USGS_LPC_UT_StatewideSouth_2020_A20_12SWH2053.laz 60968957 ac52a598ac2dab476158741086f5beee 791a3accd03f043c897ab39a41103ac8c77a56cc7514b7c7d7ffd003540576fd
USGS_OPR_UT_StateWide_2018_A18_12SWH1648.tif 28429086 e82d50325e2260a12dbf1cd64ec04ee4 ba69c1843796edc66031973a5baf5b652069212586cac9e12d61d05ed1bebc14
USGS_OPR_UT_StateWide_2018_A18_12SWH1650.tif 26010499 1d5dd1121b7d35c20b4fa98ddacdccfc 0a041269d62feb8e99dd9c44b7ef1293b4796456c55dbc8493d9fab73b9a267e
USGS_OPR_UT_StateWide_2018_A18_12SWH1652.tif 313146 381d3f4b4a2a958fc78c35c8382b7166 92f7ea59ec45830a1fdc210034c9c623c27c295fb1fb530db9b4230f3baaee0f
USGS_OPR_UT_StateWide_2018_A18_12SWH1752.tif 1347935 c15f815c42a3a47cd1df34be812e9646 afa7d1fa4495370df2793ace221253c8ba8a98ef629a2e6cf6d486a13186a3f7
USGS_OPR_UT_StateWide_2018_A18_12SWH1848.tif 31081331 bb579c38ac58c019e90daf4e01bd5dc1 e94f1f657ebb41ecf44e9d52866f34d79986c30d1657644285bbca75a9837e2e
USGS_OPR_UT_StateWide_2018_A18_12SWH1850.tif 32255622 78bf8511ae8595ddbd2aec502076e0ee 9f38cb6d030e242e66f687e9548fd87d6aba08f1f2d4282b23e745d9a523c333
USGS_OPR_UT_StateWide_2018_A18_12SWH1852.tif 2522713 27470c9befdf3e767aca38db3c94950e 7a1ff5fdd9ee50ae0e1c21e06d61ca702314ee816399d734e6ea3d07152f0961
USGS_OPR_UT_StateWide_2018_A18_12SWH1952.tif 3649743 2d33c1199384b7a9ad28e8e9e55cbb86 0b2ab3a390c4865c3f647efefa3cbb940483ce432d20063249a8e2e3e9c17152
USGS_OPR_UT_StateWide_2018_A18_12SWH2048.tif 30729327 59ead725867eb34c2122592a6acca9f2 bca9e144a50378f7b42a15bbd6f0e231edac191961ba953242c046a47246559d
USGS_OPR_UT_StateWide_2018_A18_12SWH2050.tif 33575511 5ea175ce6d0939383ffc8980487f6b02 7b3ebb68677b240df70849f78ddeaf12ff9a27c31b710eb6db3ffa848ab484a6
USGS_OPR_UT_StateWide_2018_A18_12SWH2052.tif 4770879 156a8b6632a3fd7760aabe976b925521 8b3bf4a8690473b145ad266a765df3a32aa99f1c262b31ad367a6365e60fb890
17 files, 489363946 bytes
ETags match the MD5s
```

If a size or hash differs from this list, stop and tell the user: USGS changed a file since 2026-10-10, and the rebuild would no longer be the file the worlds were made from (an ETag mismatch alone means the download was damaged: download that file again).

- [ ] **Step 3: Write the pins into the two provenance files**

Create `$S/scripts/pin_sources.py`:

```python
"""Writes the size, MD5 (OPR tiles: S3's ETag) and SHA-256 of each USGS file into the lidar DEMs' provenance, and
narrows the route area's 2018 tiles to the 5 that intersect its UTM grid: python -I pin_sources.py <dem dir> <hashes>"""
import json
import sys
from pathlib import Path

dem_dir, hashes = Path(sys.argv[1]), json.loads(Path(sys.argv[2]).read_text())
LAZ_2052 = "12f4ba5be267ffd16586b123c24e3b9fc9f46f2fbd01d3d8f3d254964fc8a811"  # measured 2026-10-10
ROUTE_OPR = ("12SWH1850", "12SWH1852", "12SWH1952", "12SWH2050", "12SWH2052")
NOTE = ("the 5 of mdrs_area_lidar2018_0p5m.json's 11 tiles that intersect the UTM grid 518000-521000 E x "
        "4250000-4254000 N (the tiles do not overlap); the run of 2026-10-06 put all 11 in its VRT, and its nearest "
        "resampling onto that grid copies these tiles' pixels, so the other 6 cannot change one")


def pinned(tile):
    name = tile["url"].rsplit("/", 1)[1]
    h = hashes[name]
    if "size_bytes" in tile and tile["size_bytes"] != h["size_bytes"]:
        sys.exit(f"{name}: {h['size_bytes']} bytes downloaded, the provenance says {tile['size_bytes']}")
    out = {"url": tile["url"], "size_bytes": h["size_bytes"]}
    if "published" in tile:
        out["published"] = tile["published"]
    if name.endswith(".tif"):
        out["md5"] = h["md5"]
    out["sha256"] = h["sha256"]
    return out


mdrs_path = dem_dir / "mdrs_area_lidar2018_0p5m.json"
mdrs = json.loads(mdrs_path.read_text())
mdrs["source_tiles"] = [pinned(t) for t in mdrs["source_tiles"]]
mdrs_path.write_text(json.dumps(mdrs, indent=2))

route_path = dem_dir / "route_area_lidar_0p5m.json"
route = json.loads(route_path.read_text())
opr = {t["url"]: t for t in mdrs["source_tiles"]}
out = {}
for key, value in route.items():
    if key == "source_tiles_2018_opr":
        out[key] = [opr[url] for url in value if any(tile in url for tile in ROUTE_OPR)]
        out["source_tiles_2018_opr_note"] = NOTE
    elif key == "source_tiles_2020_lpc":
        out[key] = [pinned(t) for t in value]
    else:
        out[key] = value
assert len(out["source_tiles_2018_opr"]) == 5 and len(out["source_tiles_2020_lpc"]) == 6
assert out["source_tiles_2020_lpc"][4]["sha256"] == LAZ_2052, "LAZ 12SWH2052 is not the file measured on 2026-10-10"
route_path.write_text(json.dumps(out, indent=2))
for t in out["source_tiles_2018_opr"] + out["source_tiles_2020_lpc"]:
    print(t["url"].rsplit("/", 1)[1], t["size_bytes"], t["sha256"][:12])
print("route area:", sum(t["size_bytes"] for t in out["source_tiles_2018_opr"] + out["source_tiles_2020_lpc"]),
      "bytes; MDRS area:", sum(t["size_bytes"] for t in mdrs["source_tiles"]), "bytes")
```

The two files keep their format (`json.dumps(indent=2)`, no newline at the end, as they are now). Run it:

```bash
S=/private/tmp/claude-502/-Users-alarion239-Desktop-Rover/d048031d-a613-49e0-ad24-8cb3b1a40ce3/scratchpad/data
PX="/Users/alarion239/.pixi/bin/pixi run --frozen --manifest-path /Users/alarion239/Desktop/hurc_ws/wt-data/pixi.toml"
cd /Users/alarion239/Desktop/hurc_ws/wt-data && $PX python -I "$S/scripts/pin_sources.py" rover_sim/data/dem "$S/hashes.json"
git diff --stat
```

Expected: 11 lines (the 5 OPR tiles 1850, 1852, 1952, 2050, 2052, then the 6 LAZ tiles, with sizes and SHA-256 prefixes as in Step 2), then `route area: 371452622 bytes; MDRS area: 194685792 bytes`; the diff touches the two `.json` files only.

- [ ] **Step 4: Nothing that reads them breaks**

```bash
PX="/Users/alarion239/.pixi/bin/pixi run --frozen --manifest-path /Users/alarion239/Desktop/hurc_ws/wt-data/pixi.toml"
cd /Users/alarion239/Desktop/hurc_ws/wt-data/rover_sim/tests && $PX python -m unittest test_fetch_data test_landscape 2>&1 | tail -3
```

Expected: `OK` (the lidar entries are still unofficial, so nothing reads the new fields yet; `test_landscape` reads `datum_shift_applied`, unchanged). The Autonomy world's sheet embeds this `.json` (`urc/missions/autonomy.py`, `terrain_source`), so `sim-worlds` regenerates that world next time; that is expected.

- [ ] **Step 5: Commit**

```bash
cd /Users/alarion239/Desktop/hurc_ws/wt-data && git add rover_sim/data/dem/mdrs_area_lidar2018_0p5m.json rover_sim/data/dem/route_area_lidar_0p5m.json && git commit -q -F - <<'EOF'
Lidar from USGS: the provenance pins every USGS file

Each OPR tile and LAZ tile the lidar DEMs were made from gets its size and
SHA-256 (downloaded 2026-10-10; the OPR tiles also their MD5, S3's ETag);
rockyweb's ETags are no checksums. The route area lists the 5 OPR tiles that
intersect its UTM grid: the tiles do not overlap and the 2026-10-06 run
copied their pixels onto the grid by nearest resampling, so the other 6 of
its VRT cannot change one.

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
EOF
git log --oneline -1
```

### Task 4: The NAIP 2021 route mosaic from its exportImage requests

**Files:** Modify `rover_sim/data/imagery/naip2021_route_area.json`, `rover_sim/data/rasters.json`. Scratch: `$S/scripts/naip2021.py`, `$S/naip/`.

- [ ] **Step 1: Write the requests into the provenance**

Create `$S/scripts/naip2021.py`:

```python
"""The NAIP 2021 route mosaic: `requests` writes its four exportImage requests into its provenance, `official`
marks it official in rasters.json: python -I naip2021.py <rover_sim/data> requests|official"""
import json
import re
import sys
from pathlib import Path

data, mode = Path(sys.argv[1]), sys.argv[2]
SERVICE = "https://imagery.nationalmap.gov/arcgis/rest/services/USGSNAIPPlus/ImageServer/exportImage"
LOCK = "%7B%22mosaicMethod%22%3A+%22esriMosaicLockRaster%22%2C+%22lockRasterIds%22%3A+%5B195681%5D%7D"
BOXES = ("-110.79,38.418,-110.777,38.432", "-110.777,38.418,-110.76400000000001,38.432",
         "-110.79,38.404,-110.777,38.418", "-110.777,38.404,-110.76400000000001,38.418")
if mode == "requests":
    requests = [f"{SERVICE}?bbox={box}&bboxSR=4326&imageSR=4326&size=2600,2800&bandIds=0,1,2&format=tiff"
                f"&pixelType=U8&mosaicRule={LOCK}&interpolation=RSP_BilinearInterpolation&f=image" for box in BOXES]
    path = data / "imagery" / "naip2021_route_area.json"
    out = {}
    for key, value in json.loads(path.read_text()).items():
        if key == "url_template":
            out["requests"] = requests
            out["mosaic_rule"] = ("lockRaster 195681, the 2021 raster the service's default mosaic (ByAttribute on "
                                  "Year) chose for the requests of 2026-10-06, which were these without mosaicRule; "
                                  "on 2026-10-10 the service returned the file's pixels with and without it")
        else:
            out[key] = value
    path.write_text(json.dumps(out, indent=1))
elif mode == "official":
    manifest = json.loads((data / "rasters.json").read_text())
    entry = next(e for e in manifest["rasters"] if e["path"] == "imagery/naip2021_route_area.tif")
    entry["official"] = True
    del entry["worlds"], entry["rebuild"]
    text = json.dumps(manifest, indent=1)
    text = re.sub(r"\[\s*([^\[\]{}]*?)\s*\]", lambda m: "[" + ", ".join(v.strip() for v in m.group(1).split(",")) + "]",
                  text)  # one line per list of numbers or strings, as rasters.json has them
    (data / "rasters.json").write_text(text + "\n")
else:
    sys.exit(f"naip2021.py: unknown mode {mode!r}")
```

The regular expression rewrites rasters.json exactly as it is formatted now (checked: dumping the current file this way gives it byte for byte). Run the first mode:

```bash
S=/private/tmp/claude-502/-Users-alarion239-Desktop-Rover/d048031d-a613-49e0-ad24-8cb3b1a40ce3/scratchpad/data
PX="/Users/alarion239/.pixi/bin/pixi run --frozen --manifest-path /Users/alarion239/Desktop/hurc_ws/wt-data/pixi.toml"
cd /Users/alarion239/Desktop/hurc_ws/wt-data && $PX python -I "$S/scripts/naip2021.py" rover_sim/data requests && git diff --stat
```

Expected: `rover_sim/data/imagery/naip2021_route_area.json` changed only.

- [ ] **Step 2: The manifest test now fails**

```bash
PX="/Users/alarion239/.pixi/bin/pixi run --frozen --manifest-path /Users/alarion239/Desktop/hurc_ws/wt-data/pixi.toml"
cd /Users/alarion239/Desktop/hurc_ws/wt-data/rover_sim/tests && $PX python -m unittest test_fetch_data.Manifest 2>&1 | grep -E "^FAIL|AssertionError|^Ran"
```

Expected: `FAIL: test_each_raster_has_its_provenance_and_an_official_one_its_requests (... subtest imagery/naip2021_route_area.tif)` with `AssertionError: True != False`: the provenance lists exportImage requests, so the raster must be official.

- [ ] **Step 3: Mark it official**

```bash
S=/private/tmp/claude-502/-Users-alarion239-Desktop-Rover/d048031d-a613-49e0-ad24-8cb3b1a40ce3/scratchpad/data
PX="/Users/alarion239/.pixi/bin/pixi run --frozen --manifest-path /Users/alarion239/Desktop/hurc_ws/wt-data/pixi.toml"
cd /Users/alarion239/Desktop/hurc_ws/wt-data && $PX python -I "$S/scripts/naip2021.py" rover_sim/data official && git diff rover_sim/data/rasters.json
```

Expected: the diff shows only the NAIP 2021 entry: `"official": false` becomes `true`, its `worlds` and `rebuild` lines are gone.

- [ ] **Step 4: The tests pass**

```bash
PX="/Users/alarion239/.pixi/bin/pixi run --frozen --manifest-path /Users/alarion239/Desktop/hurc_ws/wt-data/pixi.toml"
cd /Users/alarion239/Desktop/hurc_ws/wt-data/rover_sim/tests && $PX python -m unittest test_fetch_data 2>&1 | tail -3
```

Expected: `Ran 17 tests`, `OK`.

- [ ] **Step 5: fetch-data rebuilds it from USGS (about 91 MB)**

A data directory with a manifest that lists only this raster:

```bash
S=/private/tmp/claude-502/-Users-alarion239-Desktop-Rover/d048031d-a613-49e0-ad24-8cb3b1a40ce3/scratchpad/data
PX="/Users/alarion239/.pixi/bin/pixi run --frozen --manifest-path /Users/alarion239/Desktop/hurc_ws/wt-data/pixi.toml"
W=/Users/alarion239/Desktop/hurc_ws/wt-data/rover_sim/data
rm -rf "$S/naip" && mkdir -p "$S/naip/imagery" && cp "$W/imagery/naip2021_route_area.json" "$S/naip/imagery/"
$PX python -I -c "
import json, sys
m = json.load(open(sys.argv[1]))
m['rasters'] = [e for e in m['rasters'] if e['path'] == 'imagery/naip2021_route_area.tif']
json.dump(m, open(sys.argv[2], 'w'))" "$W/rasters.json" "$S/naip/rasters.json"
cd /Users/alarion239/Desktop/hurc_ws/wt-data && $PX python -I rover_sim/tools/fetch_data.py --all --data "$S/naip" > "$S/naip.log" 2>&1; echo "exit $?" >> "$S/naip.log"; cat "$S/naip.log"
```

Run it with `run_in_background` (four 22.7 MB responses; the service sometimes answers HTTP 500, which fetch-data retries). Expected:

```
imagery/naip2021_route_area.tif: fetched from its official source, content verified
exit 0
```

Then `rm -rf "$S/naip" "$S/naip.log"`. If the service returns other pixels (`FAILED, the official source now returns other pixels than rasters.json lists`), stop and tell the user: raster 195681 changed on the service.

- [ ] **Step 6: Commit**

```bash
cd /Users/alarion239/Desktop/hurc_ws/wt-data && git add rover_sim/data/imagery/naip2021_route_area.json rover_sim/data/rasters.json && git commit -q -F - <<'EOF'
Lidar from USGS: fetch-data rebuilds the NAIP 2021 route mosaic

Its provenance gave only a URL template; it now lists the four exportImage
requests of 2026-10-06 (USGS NAIP Plus ImageServer, 2600 x 2800 px each),
with the mosaic locked to raster 195681, the 2021 image the service's
default mosaic chose then, so a newer NAIP year cannot replace it. The
existing exportImage path of fetch-data rebuilds it with the file's content
(checked 2026-10-10), so the raster is official.

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
EOF
git log --oneline -1
```

## Part B: the recipe

### Task 5: `usgs_lidar`: the grids of 2026-10-06

**Files:** Create `rover_sim/tools/usgs_lidar.py`, `rover_sim/tests/test_usgs_lidar.py`.

- [ ] **Step 1: Write the test file with its helpers and the grid tests**

Create `rover_sim/tests/test_usgs_lidar.py`:

```python
#!/usr/bin/env python3
"""tools/usgs_lidar.py without the network: the grids of 2026-10-06 against rasters.json and the provenance, the
route area's gridding, coverage, fill and feathering on small arrays, the EHdr header, the LAZ reader, and both
recipes end to end on made-up tiles with GDAL's tools."""
import json
import tempfile
import unittest
from pathlib import Path

import numpy as np

from worldfiles import SIM_DIR

import fetch_data  # noqa: E402  (worldfiles puts tools/ on the path)
import usgs_lidar as L  # noqa: E402
from urc import dem  # noqa: E402

MANIFEST = {e["path"]: e for e in fetch_data.load_manifest()["rasters"]}
DEM_DIR = SIM_DIR / "data" / "dem"
NAN = np.nan


def plane(x, y):
    """The made-up ground [m] at UTM (x, y): its values at the cell centres and the points below lie on the LAZ
    files' 1 mm lattice, so gridding them loses nothing."""
    return 1400.0 + 0.02 * (np.asarray(x) - 518000.0) + 0.04 * (np.asarray(y) - 4253950.0)


def opr_tile(path, x0, y1, cols, rows):
    """A made-up OPR tile: plane() at the centres of 0.5 m cells of EPSG:6341 from (x0, y1)."""
    grid = L.UtmGrid(x0, y1, 0.5, cols, rows, 0)
    xs = x0 + (np.arange(cols) + 0.5) * 0.5
    ys = y1 - (np.arange(rows) + 0.5) * 0.5
    L.write_ehdr(path.with_suffix(".bil"), plane(*np.meshgrid(xs, ys)).astype(np.float32), grid)
    L.gdal("gdal_translate", "-q", "-a_srs", "EPSG:6341", path.with_suffix(".bil"), path)
    return path


def laz_tile(path, grid, holes=()):
    """A made-up LAZ tile over the grid's first lpc_rows rows: per cell (except `holes`, (row, col)) three ground
    returns at plane() + 1 cm round its centre and one class-1 return 5 m above."""
    import laspy
    rows, cols = np.meshgrid(np.arange(grid.lpc_rows), np.arange(grid.cols), indexing="ij")
    keep = np.ones(rows.shape, bool)
    for r, c in holes:
        keep[r, c] = False
    cx = grid.x0 + (cols[keep] + 0.5) * grid.res
    cy = grid.y1 - (rows[keep] + 0.5) * grid.res
    x = np.concatenate([cx, cx + 0.1, cx - 0.1, cx])
    y = np.concatenate([cy, cy + 0.1, cy - 0.1, cy])
    z = plane(x, y) + 0.01
    cls = np.concatenate([np.full(3 * cx.size, 2), np.full(cx.size, 1)]).astype(np.uint8)
    z[cls == 1] += 5.0
    header = laspy.LasHeader(point_format=3, version="1.2")
    header.scales = np.array([0.001, 0.001, 0.001])
    header.offsets = np.array([518000.0, 4253900.0, 1300.0])
    las = laspy.LasData(header)
    las.x, las.y, las.z, las.classification = x, y, z, cls
    las.write(path)
    return path


def fill_raster(path, value=1500.0):
    """A made-up 3DEP DEM in EPSG:4326 round the made-up area: one value everywhere."""
    (lat1, lon0), (lat0, lon1) = dem.utm_to_wgs84(517980.0, 4254020.0, 12), dem.utm_to_wgs84(518044.0, 4253930.0, 12)
    step = 1e-4
    cols, rows = int((lon1 - lon0) / step) + 1, int((lat1 - lat0) / step) + 1
    transform = (lon0, step, 0.0, lat1, 0.0, -step)
    fetch_data.write_geotiff(path, np.full((1, rows, cols), value, np.float32), fetch_data.geotags(transform))
    return path


def target_at(x, y, cols, rows):
    """A target grid whose north-west corner is at UTM (x, y), pixels of LON_PIXEL x LAT_PIXEL."""
    north, west = dem.utm_to_wgs84(x, y, 12)
    return L.Target(west, north - rows * L.LAT_PIXEL, west + cols * L.LON_PIXEL, north, cols, rows)


class Grids(unittest.TestCase):
    """The grids and bounds of 2026-10-06, tied to rasters.json and the provenance."""

    def test_the_targets_give_the_manifests_georeferencing(self):
        for path, target in (("dem/mdrs_area_lidar2018_0p5m.tif", L.MDRS_TARGET),
                             ("dem/route_area_lidar_0p5m.tif", L.ROUTE_TARGET),
                             ("dem/route_area_lidar_0p5m_source.tif", L.ROUTE_TARGET)):
            with self.subTest(path):
                west, north, east, south = map(float, target.ullr())  # what gdal_translate -a_ullr sets
                self.assertEqual([west, (east - west) / target.cols, 0.0, north, 0.0, (south - north) / target.rows],
                                 MANIFEST[path]["transform"])
                self.assertEqual([1, target.rows, target.cols], MANIFEST[path]["shape"])

    def test_the_bounds_are_the_strings_of_2026_10_06(self):
        self.assertEqual(L.MDRS_TARGET.te(), ["-110.80999419132665", "38.38998826430441", "-110.7699853729845",
                                              "38.416328700484314"])
        self.assertEqual(L.MDRS_TARGET.ullr(), ["-110.8100044091711", "38.416334263340396", "-110.76999559082894",
                                                "38.38999382716049"])
        self.assertEqual(L.ROUTE_TARGET.te(), ["-110.78998978215556", "38.40399605750281", "-110.76398978215556",
                                               "38.43199179164128"])
        self.assertEqual(L.ROUTE_TARGET.ullr(), ["-110.79", "38.43199735449736", "-110.76400000000001",
                                                 "38.40400162035889"])
        self.assertEqual((L.ROUTE_GRID.te(), L.ROUTE_GRID.tr()),
                         (["518000.0", "4250000.0", "521000.0", "4254000.0"], ["0.5", "0.5"]))

    def test_the_shift_is_the_provenances(self):
        for name in ("mdrs_area_lidar2018_0p5m.json", "route_area_lidar_0p5m.json"):
            with self.subTest(name):
                shift = json.loads((DEM_DIR / name).read_text())["datum_shift_applied"]
                self.assertEqual(L.SHIFT_DEG, (shift["dlon_deg"], shift["dlat_deg"]))


if __name__ == "__main__":
    unittest.main()
```

The expected strings are those of the 2026-10-06 commands: `-te` is `print(W-(DLON), S-(DLAT), E-(DLON), N-(DLAT))` of Python floats, `-a_ullr` their `repr`.

- [ ] **Step 2: Run it to see it fail**

```bash
PX="/Users/alarion239/.pixi/bin/pixi run --frozen --manifest-path /Users/alarion239/Desktop/hurc_ws/wt-data/pixi.toml"
cd /Users/alarion239/Desktop/hurc_ws/wt-data/rover_sim/tests && $PX python -m unittest test_usgs_lidar 2>&1 | tail -4
```

Expected: `ModuleNotFoundError: No module named 'usgs_lidar'`, `FAILED (errors=1)`.

- [ ] **Step 3: Create the module with its constants and grids**

Create `rover_sim/tools/usgs_lidar.py`:

```python
#!/usr/bin/env python3
"""The 0.5 m lidar DEMs rebuilt from USGS 3DEP lidar: fetch_data.py runs the recipe a raster's provenance names
("recipe": "mdrs_area" or "route_area") on the USGS files that provenance lists, which fetch_data downloads and
checks.

mdrs_area makes dem/mdrs_area_lidar2018_0p5m.tif from the 11 tiles of the 2018 UT_Southern_QL1 0.5 m OPR DEM: one
VRT, warped bilinear to EPSG:4326 onto the file's grid (its columns continue the 3DEP lattice at half its pixel,
its rows are square in metres).

route_area makes dem/route_area_lidar_0p5m.tif and its source map dem/route_area_lidar_0p5m_source.tif on the UTM
grid 518000-521000 E x 4250000-4254000 N at 0.5 m: the 2018 OPR DEM (nearest), the class-2 ground returns of the
2020 UT_StatewideSouth point clouds (the mean per cell; the empty cells inside the survey's coverage filled by
normalized Gaussian convolution), the two feathered by distance to each survey's edge, and the cells with neither
filled from dem/route_area_3dep.tif; written as EHdr BIL and warped bilinear (the map: nearest) to EPSG:4326 onto
route_area_3dep.tif's columns at half its pixel.

Both shift the result by the measured (-0.8925, +0.6175) m east, north (the .json's datum_shift_applied): PROJ's
NAD83(2011) -> WGS84 is a null transformation, while the 3DEP and NAIP services apply a real shift. The steps are
the ones that made the files on 2026-10-06, kept as they were (the sigma list, the float64 weights, the EHdr
pixel-centre origin, Python's float strings for -te and -a_ullr, the compound source CRS EPSG:6341+5703 of the DEM
warps: GDAL rounds the 2D EPSG:6341 differently in the last bit); with GDAL 3.13.3, PROJ 9.8.1, OpenCV 4.13.0,
numpy 2.5.3 and laspy 2.7.0 on osx-arm64 they give the files byte for byte (rasters.json's sha256). The warps name
the null coordinate operations PROJ picks offline (-ct) and run with PROJ_NETWORK=OFF, so neither PROJ's network
nor a local grid can add a geoid or a datum shift.
"""
import os
import shutil
import subprocess
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np

NODATA = -999999  # the OPR tiles' no-data value, kept through the warps (R: their GeoTIFF tags)
# [deg] the datum shift (lon, lat): -0.8925 m east, +0.6175 m north (M: phase correlation of hillshades against
# mdrs_area_3dep.tif, 2026-10-06), at 87347.19 and 111004.13 m per degree (38.405 N, GRS80).
SHIFT_DEG = (-0.8925 / 87347.19, 0.6175 / 111004.13)
LON_PIXEL = 5.731922398589246e-06  # [deg] half route_area_3dep.tif's pixel: 0.5007 m east-west at 38.405 N
LAT_PIXEL = 4.5103486609432234e-06  # [deg] the same 0.5007 m north-south
SIGMAS = (0.7, 1.5, 3, 6, 12, 25)  # [cells] the normalized-Gaussian fill's passes, until every cell is filled
BLOCK = 8  # [cells] the 2020 survey's coverage: the 4 m blocks with a ground return, closed by one block
SOURCE_2018, SOURCE_2020, SOURCE_BLEND, SOURCE_3DEP = 1, 2, 3, 4  # the source map's codes (0: none)
ROUTE_FILL = "dem/route_area_3dep.tif"  # the 1 m 3DEP DEM the route grid's cells with no lidar take
# The coordinate operations PROJ 9.8.1 picks offline between NAD83(2011) UTM 12N (EPSG:6341; with NAVD88 heights
# EPSG:6341+5703) and EPSG:4326: null datum transformations (projinfo). The warps name them (gdalwarp -ct), so
# neither PROJ's network (the pixi environment's activation sets PROJ_NETWORK=ON) nor a local grid can add a geoid
# or a datum shift.
UTM_TO_LONLAT = ("+proj=pipeline +step +inv +proj=utm +zone=12 +ellps=GRS80 +step +proj=unitconvert +xy_in=rad "
                 "+xy_out=deg +step +proj=axisswap +order=2,1")
LONLAT_TO_UTM = ("+proj=pipeline +step +proj=axisswap +order=2,1 +step +proj=unitconvert +xy_in=deg +xy_out=rad "
                 "+step +proj=utm +zone=12 +ellps=GRS80")


@dataclass(frozen=True)
class Target:
    """An EPSG:4326 output grid: its edges [deg] and its size [px]."""
    west: float
    south: float
    east: float
    north: float
    cols: int
    rows: int

    def te(self):
        """gdalwarp's -te: the edges moved against the datum shift, so the warp samples the ground that the services
        put at these edges."""
        dlon, dlat = SHIFT_DEG
        return [str(v) for v in (self.west - dlon, self.south - dlat, self.east - dlon, self.north - dlat)]

    def ullr(self):
        """gdal_translate's -a_ullr: the edges themselves."""
        return [repr(v) for v in (self.west, self.north, self.east, self.south)]


@dataclass(frozen=True)
class UtmGrid:
    """A NAD83(2011) UTM 12N grid: the corner (x0, y1) [m] of cell (0, 0) (west, north), the cell size [m], the size
    [cells], and how many rows from the north the 2020 point clouds cover."""
    x0: float
    y1: float
    res: float
    cols: int
    rows: int
    lpc_rows: int

    def te(self):
        return [repr(v) for v in (self.x0, self.y1 - self.rows * self.res, self.x0 + self.cols * self.res, self.y1)]

    def tr(self):
        return [repr(self.res), repr(self.res)]


# The grids of 2026-10-06 (provenance): the bounds as the commands gave them (W, N and the counts; S or E derived).
MDRS_TARGET = Target(-110.8100044091711, 38.38999382716049, -110.76999559082894,
                     38.38999382716049 + 5840 * LAT_PIXEL, 6980, 5840)
ROUTE_TARGET = Target(-110.79, 38.43199735449736 - 6207 * LAT_PIXEL, -110.79 + 4536 * LON_PIXEL,
                      38.43199735449736, 4536, 6207)
ROUTE_GRID = UtmGrid(518000.0, 4254000.0, 0.5, 6000, 8000, 4000)  # the LAZ tiles cover 4252000-4254000 N
```

The imports the later tasks need (`os`, `shutil`, `subprocess`, `Path`, `cv2`, `numpy`) are there from the start.

- [ ] **Step 4: Run the tests**

```bash
PX="/Users/alarion239/.pixi/bin/pixi run --frozen --manifest-path /Users/alarion239/Desktop/hurc_ws/wt-data/pixi.toml"
cd /Users/alarion239/Desktop/hurc_ws/wt-data/rover_sim/tests && $PX python -m unittest -v test_usgs_lidar 2>&1 | tail -4
```

Expected: `Ran 3 tests`, `OK`.

- [ ] **Step 5: Commit**

```bash
cd /Users/alarion239/Desktop/hurc_ws/wt-data && git add rover_sim/tools/usgs_lidar.py rover_sim/tests/test_usgs_lidar.py && git commit -q -F - <<'EOF'
Lidar from USGS: usgs_lidar.py and the grids of 2026-10-06

The module of the lidar rebuild starts with its constants (the measured
datum shift, the fill's sigmas, the coverage blocks, the null coordinate
operations) and the grids the files were made on: the EPSG:4326 targets with
the bounds as the 2026-10-06 commands gave them, the route area's UTM grid.
Tests tie them to rasters.json's georeferencing and the provenance's shift.

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
EOF
git log --oneline -1
```

### Task 6: `usgs_lidar`: the route area's steps

**Files:** Modify `rover_sim/tools/usgs_lidar.py` (append), `rover_sim/tests/test_usgs_lidar.py` (a class before the `if __name__` block).

- [ ] **Step 1: Write the tests**

In `rover_sim/tests/test_usgs_lidar.py`, insert before the final `if __name__ == "__main__":` block:

```python
class RouteSteps(unittest.TestCase):
    """The route area's gridding, coverage, fill and feathering on small arrays."""

    def test_cell_means_average_and_count_the_points_of_every_tile(self):
        grid = L.UtmGrid(100.0, 200.0, 0.5, 4, 4, 2)
        a = (np.array([100.1, 100.4, 101.2, 99.9, 100.1]), np.array([199.9, 199.6, 199.2, 199.9, 198.9]),
             np.array([10.0, 12.0, 7.0, 50.0, 60.0]))  # the last two lie west of the grid and south of row 1
        b = (np.array([100.2]), np.array([199.8]), np.array([14.0]))
        mean, count = L.cell_means(iter([a, b]), grid)
        want = np.full((2, 4), NAN, np.float32)
        want[0, 0], want[1, 2] = 12.0, 7.0
        np.testing.assert_array_equal(mean, want)
        np.testing.assert_array_equal(count, [[3, 0, 0, 0], [0, 0, 1, 0]])
        self.assertEqual((mean.dtype, count.dtype), (np.float32, np.uint16))

    def test_fill_nan_fills_the_holes_and_keeps_the_rest(self):
        a = np.full((20, 20), 5.0, np.float32)
        a[5, 5] = a[10:14, 10:14] = NAN
        filled = L.fill_nan(a)
        np.testing.assert_allclose(filled, 5.0, atol=1e-5)
        np.testing.assert_array_equal(filled[np.isfinite(a)], a[np.isfinite(a)])
        half = np.full((10, 40), NAN, np.float32)
        half[:, :20] = 3.0  # the far cells need the wide passes
        np.testing.assert_allclose(L.fill_nan(half), 3.0, atol=1e-5)
        full = np.arange(12, dtype=np.float32).reshape(3, 4)
        self.assertIsNot(L.fill_nan(full), full)
        np.testing.assert_array_equal(L.fill_nan(full), full)

    def test_coverage_keeps_the_surveyed_blocks_and_closes_one_block_holes(self):
        count = np.ones((40, 40), np.uint16)
        count[16:24, 16:24] = 0  # the middle block of 5 x 5
        self.assertTrue(L.coverage(count).all())
        count = np.zeros((40, 40), np.uint16)
        count[3, 0:16:8] = 1  # one point in each block of the first two block columns
        want = np.zeros((40, 40), bool)
        want[:8, :16] = True
        np.testing.assert_array_equal(L.coverage(count), want)

    def test_lpc_surface_fills_inside_the_coverage_on_the_north_rows(self):
        grid = L.UtmGrid(0.0, 0.0, 0.5, 24, 24, 16)
        mean = np.full((16, 24), NAN, np.float32)
        mean[:, :8] = 7.0
        mean[3, 3] = NAN
        count = np.zeros((16, 24), np.uint16)
        count[:, :8] = 1
        count[3, 3] = 0
        z20 = L.lpc_surface(mean, count, grid)
        self.assertEqual(z20.shape, (24, 24))
        np.testing.assert_allclose(z20[:16, :8], 7.0, atol=1e-5)  # the empty cell filled
        self.assertTrue(np.isnan(z20[:16, 8:]).all() and np.isnan(z20[16:]).all())

    def test_blend_feathers_where_both_surveys_have_data(self):
        o18 = np.full((3, 12), NAN, np.float32)
        o18[:, :8] = 100.0
        z20 = np.full((3, 12), NAN, np.float32)
        z20[:, 4:] = 101.0
        z, src = L.blend(o18, z20, np.full((3, 12), 50.0, np.float32))
        np.testing.assert_allclose(z[1], [100, 100, 100, 100, 100.2, 100.4, 100.6, 100.8, 101, 101, 101, 101],
                                   atol=1e-5)
        np.testing.assert_array_equal(src[1], [1, 1, 1, 1, 3, 3, 3, 3, 2, 2, 2, 2])
        self.assertEqual(z.dtype, np.float32)

    def test_blend_fills_the_gaps_from_3dep_and_marks_the_rest_no_data(self):
        o18 = np.full((3, 12), NAN, np.float32)
        o18[:, :4] = 100.0
        z20 = np.full((3, 12), NAN, np.float32)
        z20[:, 8:] = 101.0
        dep = np.full((3, 12), NAN, np.float32)
        dep[:, 4:6] = 50.0
        z, src = L.blend(o18, z20, dep)
        np.testing.assert_array_equal(z[1], [100] * 4 + [50] * 2 + [L.NODATA] * 2 + [101] * 4)
        np.testing.assert_array_equal(src[1], [1] * 4 + [4] * 2 + [0] * 2 + [2] * 4)

    def test_write_ehdr_writes_the_headers_of_2026_10_06(self):
        with tempfile.TemporaryDirectory() as d:
            bil = Path(d) / "z.bil"
            L.write_ehdr(bil, np.zeros((2, 3), np.float32), L.ROUTE_GRID)
            self.assertEqual(bil.with_suffix(".hdr").read_text(),
                             "BYTEORDER I\nLAYOUT BIL\nNROWS 8000\nNCOLS 6000\nNBANDS 1\nNBITS 32\nPIXELTYPE FLOAT\n"
                             "ULXMAP 518000.25\nULYMAP 4253999.75\nXDIM 0.5\nYDIM 0.5\nNODATA -999999\n")
            L.write_ehdr(bil, np.array([[1, 2, 3]], np.uint8), L.ROUTE_GRID)
            self.assertEqual(bil.with_suffix(".hdr").read_text(),
                             "BYTEORDER I\nLAYOUT BIL\nNROWS 8000\nNCOLS 6000\nNBANDS 1\nNBITS 8\n"
                             "ULXMAP 518000.25\nULYMAP 4253999.75\nXDIM 0.5\nYDIM 0.5\n")
            self.assertEqual(bil.read_bytes(), b"\x01\x02\x03")

    def test_ground_returns_reads_the_class_2_points_of_a_laz_tile(self):
        with tempfile.TemporaryDirectory() as d:
            grid = L.UtmGrid(518000.0, 4254000.0, 0.5, 2, 2, 1)
            path = laz_tile(Path(d) / "t.laz", grid)
            self.assertEqual(path.read_bytes()[:4], b"LASF")
            x, y, z = L.ground_returns(path)
        self.assertEqual(len(x), 6)  # 3 ground returns in each of 2 cells; the 2 class-1 returns dropped
        np.testing.assert_allclose(sorted(x), [518000.15, 518000.25, 518000.35, 518000.65, 518000.75, 518000.85],
                                   atol=1e-6)
        np.testing.assert_allclose(z, plane(x, y) + 0.01, atol=1e-6)
```

The headers are the ones `blend.py` wrote on 2026-10-06 (`S/prov/scripts/blend.py`, lines 38 and 40).

- [ ] **Step 2: Run them to see them fail**

```bash
PX="/Users/alarion239/.pixi/bin/pixi run --frozen --manifest-path /Users/alarion239/Desktop/hurc_ws/wt-data/pixi.toml"
cd /Users/alarion239/Desktop/hurc_ws/wt-data/rover_sim/tests && $PX python -m unittest test_usgs_lidar 2>&1 | grep -E "AttributeError|^Ran|^FAILED" | sort | uniq -c
```

Expected: `AttributeError: module 'usgs_lidar' has no attribute ...` for `cell_means`, `fill_nan`, `coverage`, `lpc_surface`, `blend`, `write_ehdr`, `ground_returns`; `Ran 11 tests`, `FAILED (errors=8)`.

- [ ] **Step 3: Implement the steps**

Append to `rover_sim/tools/usgs_lidar.py`:

```python


def ground_returns(path):
    """(x, y, z) [m] of the class-2 (ground) returns of a LAZ tile."""
    import laspy  # only the route-area rebuild needs it
    las = laspy.read(path)
    g = np.asarray(las.classification) == 2
    return np.asarray(las.x)[g], np.asarray(las.y)[g], np.asarray(las.z)[g]


def cell_means(tiles, grid):
    """(mean z float32, count uint16) of the points per cell of the grid's first lpc_rows rows; tiles yields (x, y, z)
    per tile. The float64 sums run over the tiles in their order, as on 2026-10-06; a cell with no point is NaN."""
    n = grid.lpc_rows * grid.cols
    s, c = np.zeros(n), np.zeros(n, np.int64)
    for x, y, z in tiles:
        col = np.floor((x - grid.x0) / grid.res).astype(np.int64)
        row = np.floor((grid.y1 - y) / grid.res).astype(np.int64)
        ok = (col >= 0) & (col < grid.cols) & (row >= 0) & (row < grid.lpc_rows)
        idx = row[ok] * grid.cols + col[ok]
        s += np.bincount(idx, weights=z[ok], minlength=n)
        c += np.bincount(idx, minlength=n)
    mean = np.where(c > 0, s / np.maximum(c, 1), np.nan).reshape(grid.lpc_rows, grid.cols).astype(np.float32)
    return mean, c.reshape(grid.lpc_rows, grid.cols).astype(np.uint16)


def fill_nan(a):
    """a with its NaN cells filled by normalized Gaussian convolution of the valid ones, sigma growing over SIGMAS
    until every cell is filled (a cell farther than the last pass reaches stays NaN)."""
    a = a.astype(np.float32).copy()
    m = np.isfinite(a)
    if m.all():
        return a
    out = a.copy()
    for s in SIGMAS:
        w = cv2.GaussianBlur(m.astype(np.float32), (0, 0), s)
        v = cv2.GaussianBlur(np.where(m, a, 0).astype(np.float32), (0, 0), s)
        f = (~np.isfinite(out)) & (w > 1e-3)
        out[f] = v[f] / w[f]
        if np.isfinite(out).all():
            break
    return out


def coverage(count, block=BLOCK):
    """Where a survey covers the grid: the block x block cells blocks with any point, closed by one block (3 x 3)."""
    rows, cols = count.shape
    blk = count.reshape(rows // block, block, cols // block, block).sum(axis=(1, 3)) > 0
    blk = cv2.morphologyEx(blk.astype(np.uint8), cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))
    return np.repeat(np.repeat(blk, block, 0), block, 1).astype(bool)


def lpc_surface(mean, count, grid):
    """The 2020 surface on the whole grid: the cell means, filled, inside the coverage; NaN elsewhere."""
    z20 = np.full((grid.rows, grid.cols), np.nan, np.float32)
    f = fill_nan(mean)
    f[~coverage(count)] = np.nan
    z20[:grid.lpc_rows] = f
    return z20


def blend(o18, z20, dep):
    """(z float32, source uint8) from the 2018 DEM, the 2020 surface and the 3DEP fill (NaN where each has no data):
    where both surveys have data the 2020 weight is its distance to its edge over the sum of both distances; cells
    with neither take the fill (source 4), cells with nothing are NODATA (source 0)."""
    v18, v20 = np.isfinite(o18), np.isfinite(z20)
    d18 = cv2.distanceTransform(v18.astype(np.uint8), cv2.DIST_L2, 5)
    d20 = cv2.distanceTransform(v20.astype(np.uint8), cv2.DIST_L2, 5)
    w20 = np.where(v20 & v18, d20 / np.maximum(d18 + d20, 1e-6), np.where(v20, 1.0, 0.0))
    z = np.where(v18 | v20, np.nan_to_num(o18) * (1 - w20) + np.nan_to_num(z20) * w20, np.nan).astype(np.float32)
    src = np.where(v18 & v20, SOURCE_BLEND, np.where(v20, SOURCE_2020, np.where(v18, SOURCE_2018, 0))).astype(np.uint8)
    gap = ~np.isfinite(z)
    z[gap] = dep[gap]
    src[gap & np.isfinite(dep)] = SOURCE_3DEP
    z[~np.isfinite(z)] = NODATA
    return z, src


def write_ehdr(bil, array, grid):
    """array (rows, cols), float32 or uint8, as the EHdr raster bil (.bil) with its .hdr beside it; ULXMAP and
    ULYMAP are the centre of cell (0, 0)."""
    floating = array.dtype == np.float32
    array.astype("<f4" if floating else np.uint8).tofile(bil)
    lines = ["BYTEORDER I", "LAYOUT BIL", f"NROWS {grid.rows}", f"NCOLS {grid.cols}", "NBANDS 1",
             f"NBITS {32 if floating else 8}", *(["PIXELTYPE FLOAT"] if floating else []),
             f"ULXMAP {grid.x0 + grid.res / 2!r}", f"ULYMAP {grid.y1 - grid.res / 2!r}", f"XDIM {grid.res!r}",
             f"YDIM {grid.res!r}", *([f"NODATA {NODATA}"] if floating else [])]
    Path(bil).with_suffix(".hdr").write_text("\n".join(lines) + "\n")
```

These are `grid2020.py` and `blend.py` of 2026-10-06 (`S/prov/scripts/`) cut into functions, with the same operations in the same order and dtypes: `cell_means` is lines 7-17 of `grid2020.py`, `fill_nan` lines 5-16 of `blend.py`, `coverage` and `lpc_surface` lines 18-24, `blend` lines 27-36, `write_ehdr` lines 37-40. Do not "simplify" them: the float64 weights (`np.where(v20, 1.0, 0.0)` promotes `w20`), the float32 casts and the sigma list decide the last bits.

- [ ] **Step 4: Run the tests**

```bash
PX="/Users/alarion239/.pixi/bin/pixi run --frozen --manifest-path /Users/alarion239/Desktop/hurc_ws/wt-data/pixi.toml"
cd /Users/alarion239/Desktop/hurc_ws/wt-data/rover_sim/tests && $PX python -m unittest -v test_usgs_lidar 2>&1 | tail -4
```

Expected: `Ran 11 tests`, `OK`.

- [ ] **Step 5: Commit**

```bash
cd /Users/alarion239/Desktop/hurc_ws/wt-data && git add rover_sim/tools/usgs_lidar.py rover_sim/tests/test_usgs_lidar.py && git commit -q -F - <<'EOF'
Lidar from USGS: the route area's gridding, fill and feathering

The steps of 2026-10-06 as functions, operation for operation: the class-2
ground returns of a LAZ tile, their float64 mean per 0.5 m cell over the
tiles in order, the coverage of 4 m blocks closed by one block, the
normalized-Gaussian fill (sigma 0.7 to 25 cells), the feathering by distance
to each survey's edge with the 3DEP fill, and the EHdr BIL with its
pixel-centre origin. Tests on small arrays and a made-up LAZ tile.

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
EOF
git log --oneline -1
```

### Task 7: `usgs_lidar`: GDAL's tools and the two recipes

**Files:** Modify `rover_sim/tools/usgs_lidar.py` (append), `rover_sim/tests/test_usgs_lidar.py` (a class before the `if __name__` block).

- [ ] **Step 1: Write the end-to-end tests**

In `rover_sim/tests/test_usgs_lidar.py`, insert before the final `if __name__ == "__main__":` block:

```python
class Recipes(unittest.TestCase):
    """Both recipes end to end on made-up tiles, with GDAL's tools and laspy."""

    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.dir = Path(self.tmp.name)
        (self.dir / "work").mkdir()
        (self.dir / "data" / "dem").mkdir(parents=True)

    def check_gradients(self, z, rows):
        """In `rows` the made-up plane's slopes: 0.04 per m north (rows run south), 0.02 per m east."""
        np.testing.assert_allclose(np.diff(z[rows], axis=0), -0.04 * L.LAT_PIXEL * 111004.13, atol=1e-3)
        np.testing.assert_allclose(np.diff(z[rows], axis=1), 0.02 * L.LON_PIXEL * 87347.19, atol=1e-3)

    def test_route_area_makes_the_dem_and_its_source_map(self):
        grid = L.UtmGrid(518000.0, 4254000.0, 0.5, 48, 96, 48)  # 24 x 48 m; the point cloud covers the north half
        files = {"source_tiles_2018_opr": [opr_tile(self.dir / "opr.tif", 518000.0, 4253988.0, 48, 72)],
                 "source_tiles_2020_lpc": [laz_tile(self.dir / "lpc.laz", grid, holes=[(10, 10)])]}
        fill_raster(self.dir / "data" / L.ROUTE_FILL)
        target = target_at(518002.0, 4253998.0, 36, 80)
        made = L.route_area(files, self.dir / "data", self.dir / "work", grid=grid, target=target)
        self.assertEqual(sorted(made), ["dem/route_area_lidar_0p5m.tif", "dem/route_area_lidar_0p5m_source.tif"])
        (z,), transform = dem.read_raster(made["dem/route_area_lidar_0p5m.tif"])
        (src,), map_transform = dem.read_raster(made["dem/route_area_lidar_0p5m_source.tif"])
        west, north, east, south = map(float, target.ullr())
        self.assertEqual(transform, (west, (east - west) / 36, 0.0, north, 0.0, (south - north) / 80))
        self.assertEqual(map_transform, transform)
        self.assertEqual((z.shape, src.shape), ((80, 36), (80, 36)))
        # the shift samples 0.62 m south of each pixel: the point cloud alone north of 4253988 N, both surveys
        # down to 4253976 N, the 2018 DEM alone south of it
        self.assertTrue((src[:15] == L.SOURCE_2020).all() and (src[23:38] == L.SOURCE_BLEND).all()
                        and (src[50:] == L.SOURCE_2018).all(), src[:, 0])
        self.assertEqual(set(np.unique(src)), {L.SOURCE_2018, L.SOURCE_2020, L.SOURCE_BLEND})
        self.check_gradients(z, slice(0, 15))  # the filled empty cell among them
        self.check_gradients(z, slice(50, 80))
        self.assertTrue(1400.0 < z.min() and z.max() < 1402.6)  # the class-1 returns, 5 m up, are left out

    def test_route_area_needs_the_3dep_fill(self):
        with self.assertRaisesRegex(FileNotFoundError, "dem/route_area_3dep.tif is not here"):
            L.route_area({}, self.dir / "data", self.dir / "work")

    def test_mdrs_area_warps_the_opr_tiles(self):
        tiles = [opr_tile(self.dir / "a.tif", 518000.0, 4253988.0, 24, 72),
                 opr_tile(self.dir / "b.tif", 518012.0, 4253988.0, 24, 72)]
        target = target_at(518002.0, 4253986.0, 36, 40)
        made = L.mdrs_area({"source_tiles": tiles}, self.dir / "data", self.dir / "work", target=target)
        (z,), transform = dem.read_raster(made["dem/mdrs_area_lidar2018_0p5m.tif"])
        west, north, east, south = map(float, target.ullr())
        self.assertEqual(transform, (west, (east - west) / 36, 0.0, north, 0.0, (south - north) / 40))
        self.assertTrue(np.isfinite(z).all() and z.min() > 1400.0)
        self.check_gradients(z, slice(0, 40))
```

The made-up hole is one cell: its neighbours are symmetric, so normalized convolution of a plane gives the plane there (an L-shaped hole would not, by a few millimetres).

- [ ] **Step 2: Run them to see them fail**

```bash
PX="/Users/alarion239/.pixi/bin/pixi run --frozen --manifest-path /Users/alarion239/Desktop/hurc_ws/wt-data/pixi.toml"
cd /Users/alarion239/Desktop/hurc_ws/wt-data/rover_sim/tests && $PX python -m unittest test_usgs_lidar.Recipes 2>&1 | grep -E "AttributeError|^Ran|^FAILED"
```

Expected: `AttributeError: module 'usgs_lidar' has no attribute 'gdal'` (the helpers write the made-up tiles with it) or `'route_area'`; `Ran 3 tests`, `FAILED (errors=3)`.

- [ ] **Step 3: Implement GDAL's runner, the warps and the recipes**

Append to `rover_sim/tools/usgs_lidar.py`:

```python


def gdal(*args):
    """Runs a GDAL command-line tool, offline as far as PROJ goes; raises with its message when it fails."""
    tool = shutil.which(args[0])
    if tool is None:
        raise RuntimeError(f"{args[0]} not found: run fetch-data in the pixi environment (pixi run fetch-data)")
    result = subprocess.run([tool, *map(str, args[1:])], env={**os.environ, "PROJ_NETWORK": "OFF"},
                            capture_output=True, text=True)
    if result.returncode:
        raise RuntimeError(f"{args[0]} failed: {result.stderr.strip() or result.stdout.strip()}")


def read_float(path):
    """The float32 band of a GeoTIFF gdalwarp wrote, no data (below -1000) as NaN."""
    log = cv2.utils.logging
    level = log.getLogLevel()
    log.setLogLevel(log.LOG_LEVEL_ERROR)  # libtiff warns about every GeoTIFF tag it does not know
    try:
        a = cv2.imread(str(path), cv2.IMREAD_UNCHANGED)
    finally:
        log.setLogLevel(level)
    if a is None:
        raise ValueError(f"{path}: OpenCV cannot read it")
    a = a.astype(np.float32)
    a[a < -1000] = np.nan
    return a


def warp_dem(src, target, out):
    """A float32 raster in UTM 12N with NAVD88 heights (src) bilinear onto the target, as a deflate GeoTIFF with the
    float predictor."""
    tmp = Path(out).with_name(f"{Path(out).stem}_warped.tif")
    gdal("gdalwarp", "-q", "-overwrite", "-s_srs", "EPSG:6341+5703", "-t_srs", "EPSG:4326", "-ct", UTM_TO_LONLAT,
         "-te", *target.te(), "-ts", target.cols, target.rows, "-r", "bilinear", "-srcnodata", NODATA,
         "-dstnodata", NODATA, "-ot", "Float32", src, tmp)
    gdal("gdal_translate", "-q", "-a_ullr", *target.ullr(), "-co", "COMPRESS=DEFLATE", "-co", "PREDICTOR=3",
         "-co", "TILED=NO", tmp, out)
    tmp.unlink()


def warp_map(src, target, out):
    """A uint8 raster in UTM 12N (src) nearest onto the target, as a deflate GeoTIFF."""
    tmp = Path(out).with_name(f"{Path(out).stem}_warped.tif")
    gdal("gdalwarp", "-q", "-overwrite", "-s_srs", "EPSG:6341", "-t_srs", "EPSG:4326", "-ct", UTM_TO_LONLAT,
         "-te", *target.te(), "-ts", target.cols, target.rows, "-r", "near", src, tmp)
    gdal("gdal_translate", "-q", "-a_ullr", *target.ullr(), "-co", "COMPRESS=DEFLATE", tmp, out)
    tmp.unlink()


def mdrs_area(files, data_dir, work, target=MDRS_TARGET):
    """{raster: file in work} made from files["source_tiles"], the 2018 OPR tiles."""
    vrt = Path(work) / "opr2018.vrt"
    gdal("gdalbuildvrt", "-q", "-overwrite", vrt, *sorted(files["source_tiles"]))
    out = Path(work) / "mdrs_area_lidar2018_0p5m.tif"
    warp_dem(vrt, target, out)
    return {"dem/mdrs_area_lidar2018_0p5m.tif": out}


def route_area(files, data_dir, work, grid=ROUTE_GRID, target=ROUTE_TARGET, fill=ROUTE_FILL):
    """{raster: file in work} made from files["source_tiles_2018_opr"] (the OPR tiles that intersect the grid),
    files["source_tiles_2020_lpc"] (the LAZ tiles, in their order) and data_dir/fill."""
    work, fill_path = Path(work), Path(data_dir) / fill
    if not fill_path.is_file():
        raise FileNotFoundError(f"{fill} is not here (fetch-data fetches it before the lidar)")
    vrt = work / "opr2018.vrt"
    gdal("gdalbuildvrt", "-q", "-overwrite", vrt, *sorted(files["source_tiles_2018_opr"]))
    gdal("gdalwarp", "-q", "-overwrite", "-te", *grid.te(), "-tr", *grid.tr(), "-r", "near", "-dstnodata", NODATA,
         vrt, work / "opr2018_utm.tif")
    gdal("gdalwarp", "-q", "-overwrite", "-s_srs", "EPSG:4326", "-t_srs", "EPSG:6341", "-ct", LONLAT_TO_UTM,
         "-te", *grid.te(), "-tr", *grid.tr(), "-r", "bilinear", fill_path, work / "fill_utm.tif")
    mean, count = cell_means((ground_returns(p) for p in files["source_tiles_2020_lpc"]), grid)
    z20 = lpc_surface(mean, count, grid)
    z, src = blend(read_float(work / "opr2018_utm.tif"), z20, read_float(work / "fill_utm.tif"))
    write_ehdr(work / "route_utm.bil", z, grid)
    write_ehdr(work / "route_source_utm.bil", src, grid)
    del mean, count, z20, z, src
    out_dem, out_map = work / "route_area_lidar_0p5m.tif", work / "route_area_lidar_0p5m_source.tif"
    warp_dem(work / "route_utm.bil", target, out_dem)
    warp_map(work / "route_source_utm.bil", target, out_map)
    return {"dem/route_area_lidar_0p5m.tif": out_dem, "dem/route_area_lidar_0p5m_source.tif": out_map}


@dataclass(frozen=True)
class Recipe:
    inputs: tuple  # the provenance's keys that list the USGS files it reads
    outputs: tuple  # the rasters it makes, relative to the data directory
    make: object  # make(files {input: [paths]}, data_dir, work) -> {output: file in work}


RECIPES = {
    "mdrs_area": Recipe(("source_tiles",), ("dem/mdrs_area_lidar2018_0p5m.tif",), mdrs_area),
    "route_area": Recipe(("source_tiles_2018_opr", "source_tiles_2020_lpc"),
                         ("dem/route_area_lidar_0p5m.tif", "dem/route_area_lidar_0p5m_source.tif"), route_area),
}
```

The commands are `warp_mdrs.sh` and `warp_route.sh` of 2026-10-06 and the two warps before `blend.py` (transcript line 365: the OPR VRT nearest onto the UTM grid, `route_area_3dep.tif` bilinear onto it), with `-ct` added (Decision 2). Keep `EPSG:6341+5703` in `warp_dem` and `EPSG:6341` in `warp_map`, as the files were made (Known 4).

- [ ] **Step 4: Run the tests**

```bash
PX="/Users/alarion239/.pixi/bin/pixi run --frozen --manifest-path /Users/alarion239/Desktop/hurc_ws/wt-data/pixi.toml"
cd /Users/alarion239/Desktop/hurc_ws/wt-data/rover_sim/tests && $PX python -m unittest -v test_usgs_lidar 2>&1 | tail -4
```

Expected: `Ran 14 tests`, `OK` (about a second).

- [ ] **Step 5: Commit**

```bash
cd /Users/alarion239/Desktop/hurc_ws/wt-data && git add rover_sim/tools/usgs_lidar.py rover_sim/tests/test_usgs_lidar.py && git commit -q -F - <<'EOF'
Lidar from USGS: the two recipes, with GDAL's tools

mdrs_area warps the 11 OPR tiles of 2018 onto the MDRS-area grid; route_area
grids the 2020 point clouds, feathers them into the 2018 DEM, fills the rest
from the 3DEP DEM and warps the result and its source map onto the route
area's grid. The commands are those of 2026-10-06; every warp now names the
null coordinate operation PROJ picks offline and runs with PROJ_NETWORK=OFF
(the pixi environment turns the network on, and with it PROJ would add a
geoid and a datum shift). The DEM warps keep the compound source CRS: GDAL
rounds the 2D one differently in the last bit. Tested end to end on made-up
tiles.

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
EOF
git log --oneline -1
```

## Part C: fetch-data

### Task 8: fetch-data: stats and the tolerance

**Files:** Modify `rover_sim/tools/fetch_data.py`, `rover_sim/tests/test_fetch_data.py`.

- [ ] **Step 1: Write the tests**

In `rover_sim/tests/test_fetch_data.py`, insert before `class Help(unittest.TestCase):`:

```python
class Stats(unittest.TestCase):
    """What a lidar rebuild that is not bit-identical must match: min, max and 16 x 16 block means within 1e-3."""

    def test_the_stats_are_min_max_and_block_means(self):
        s = fetch_data.stats(np.arange(32 * 48, dtype=np.float32).reshape(1, 32, 48))
        self.assertEqual((s["min"], s["max"]), (0.0, 1535.0))
        self.assertEqual(np.shape(s["block_means"]), (1, 16, 16))
        self.assertEqual(s["block_means"][0][0][0], 25.0)  # rows 0-1, columns 0-2
        self.assertEqual(s["block_means"][0][15][15], 1510.0)

    def test_within_allows_the_tolerance_and_no_more(self):
        s = fetch_data.stats(dem_band(32, 32))
        near, far = json.loads(json.dumps(s)), json.loads(json.dumps(s))
        near["block_means"][0][3][4] += 0.0009
        far["block_means"][0][3][4] += 0.0011
        self.assertTrue(fetch_data.within(near, s))
        self.assertFalse(fetch_data.within(far, s))
        self.assertFalse(fetch_data.within(fetch_data.stats(np.concatenate([dem_band(32, 32)] * 2)), s))  # 2 bands

    def test_a_raster_within_the_stats_on_another_grid_does_not_match(self):
        with tempfile.TemporaryDirectory() as d:
            path = Path(d) / "r.tif"
            path.write_bytes(raster_bytes(dem_band(32, 32), TRANSFORM))
            entry = {"content_sha256": "", "shape": [1, 32, 32], "transform": list(TRANSFORM),
                     "stats": fetch_data.stats(dem_band(32, 32))}
            self.assertEqual(fetch_data.verdict(entry, path),
                             "content within 0.001 of rasters.json's stats, not bit-identical")
            path.write_bytes(raster_bytes(dem_band(32, 32), (TRANSFORM[0] + STEP, *TRANSFORM[1:])))
            self.assertIsNone(fetch_data.verdict(entry, path))
```

- [ ] **Step 2: Run them to see them fail**

```bash
PX="/Users/alarion239/.pixi/bin/pixi run --frozen --manifest-path /Users/alarion239/Desktop/hurc_ws/wt-data/pixi.toml"
cd /Users/alarion239/Desktop/hurc_ws/wt-data/rover_sim/tests && $PX python -m unittest test_fetch_data.Stats 2>&1 | grep -E "AttributeError|^Ran|^FAILED" | sort | uniq -c
```

Expected: `AttributeError: module 'fetch_data' has no attribute 'stats'` (3 times), `Ran 3 tests`, `FAILED (errors=3)`.

- [ ] **Step 3: Implement stats, within, verdict and check**

In `rover_sim/tools/fetch_data.py`, after the line `LINK = "link it from a teammate's clone: ..."`, add:

```python
STATS_BLOCKS = 16  # blocks per side of a raster's stats
# How far the stats of a lidar rebuild that is not bit-identical may be from the manifest's: 1 mm for the DEMs, a
# thousandth of a code for the source map. Rounding differences between platforms (OpenCV's blur, GDAL's bilinear
# kernel) are a few float32 steps (0.1 mm at 1400 m); a wrong tile, shift or step is centimetres (A).
TOLERANCE = 1e-3
```

Replace the functions `content_sha256` and `matches`:

```python
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
```

with:

```python
def content_digest(bands, transform):
    """SHA-256 of a raster's float32 bands (B, H, W) and geotransform, as dem.read_raster returns them."""
    h = hashlib.sha256(np.ascontiguousarray(bands).tobytes())
    h.update(repr(tuple(float(v) for v in transform)).encode())
    return h.hexdigest()


def content_sha256(path):
    """SHA-256 of what the simulation reads from a raster: dem.read_raster's
    float32 bands (B, H, W) and geotransform. Any lossless encoding of the same
    pixels and georeferencing has the same one."""
    return content_digest(*dem.read_raster(path))


def stats(bands):
    """A raster's stats: the min and max of its float32 bands (B, H, W) and the mean of each of STATS_BLOCKS x
    STATS_BLOCKS blocks per band (rows and columns split as np.array_split splits them), rounded to 1e-6."""
    def r(v):
        return round(float(v), 6)
    return {"min": r(bands.min()), "max": r(bands.max()),
            "block_means": [[[r(block.mean(dtype=np.float64)) for block in np.array_split(rows, STATS_BLOCKS, axis=1)]
                             for rows in np.array_split(band, STATS_BLOCKS, axis=0)] for band in bands]}


def within(got, want, tolerance=TOLERANCE):
    """Whether stats `got` are those of `want` within tolerance, number by number."""
    a = np.array([got["min"], got["max"], *np.ravel(got["block_means"])], float)
    b = np.array([want["min"], want["max"], *np.ravel(want["block_means"])], float)
    return a.shape == b.shape and bool(np.all(np.abs(a - b) <= tolerance))


def verdict(entry, path):
    """How the raster at path matches entry's content: "content verified" (the same pixels and georeferencing),
    "content within TOLERANCE ..." (a lidar rebuild that is not bit-identical but has the same shape, the same
    georeferencing and the manifest's stats within TOLERANCE), or None."""
    try:
        bands, transform = dem.read_raster(path)
    except Exception:  # not a raster dem.read_raster accepts
        return None
    if content_digest(bands, transform) == entry["content_sha256"]:
        return "content verified"
    if ("stats" in entry and list(bands.shape) == entry["shape"] and list(transform) == entry["transform"]
            and within(stats(bands), entry["stats"])):
        return f"content within {TOLERANCE} of rasters.json's stats, not bit-identical"
    return None


def matches(entry, path):
    """Whether the raster at path is entry's: the same file, or (rebuilt from
    its official source, so encoded anew) the same content, or a lidar rebuild
    within TOLERANCE of its stats."""
    return file_sha256(path) == entry["sha256"] or verdict(entry, path) is not None


def check(entry, path):
    """--verify's word for the raster at path: verified (with how, when it is within TOLERANCE), DIFFERS, MISSING."""
    if not path.exists():
        return "MISSING"
    if file_sha256(path) == entry["sha256"]:
        return "verified"
    how = verdict(entry, path)
    return "DIFFERS" if how is None else "verified" if how == "content verified" else f"verified, {how}"
```

In `main`, replace:

```python
        elif args.verify:
            state = "verified" if path.exists() and matches(entry, path) else "DIFFERS" if path.exists() else "MISSING"
            print(f"{entry['path']}: {state}")
            failed += state != "verified"
```

with:

```python
        elif args.verify:
            state = check(entry, path)
            print(f"{entry['path']}: {state}")
            failed += not state.startswith("verified")
```

- [ ] **Step 4: Run the tests**

```bash
PX="/Users/alarion239/.pixi/bin/pixi run --frozen --manifest-path /Users/alarion239/Desktop/hurc_ws/wt-data/pixi.toml"
cd /Users/alarion239/Desktop/hurc_ws/wt-data/rover_sim/tests && $PX python -m unittest test_fetch_data 2>&1 | tail -3
cd /Users/alarion239/Desktop/hurc_ws/wt-data && $PX python rover_sim/tools/fetch_data.py --verify --all | awk '{print $NF}' | sort | uniq -c
```

Expected: `Ran 20 tests`, `OK`; `  10 verified`.

- [ ] **Step 5: Commit**

```bash
cd /Users/alarion239/Desktop/hurc_ws/wt-data && git add rover_sim/tools/fetch_data.py rover_sim/tests/test_fetch_data.py && git commit -q -F - <<'EOF'
Lidar from USGS: fetch-data's tolerance for rebuilds that are not bit-identical

A raster with stats in rasters.json (min, max, 16 x 16 block means) whose
content differs but whose shape, georeferencing and stats are within 1e-3
matches, and --verify says so ("verified, content within 0.001 of
rasters.json's stats, not bit-identical"). Platform rounding is a few
float32 steps; a wrong tile or step is centimetres.

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
EOF
git log --oneline -1
```

### Task 9: fetch-data: the USGS files, pinned

**Files:** Modify `rover_sim/tools/fetch_data.py`, `rover_sim/tests/test_fetch_data.py`.

- [ ] **Step 1: Write the tests**

In `rover_sim/tests/test_fetch_data.py`, add after the function `add_unofficial` (before `def run(`):

```python


def sha256(body):
    return hashlib.sha256(body).hexdigest()
```

and insert before `class Help(unittest.TestCase):`:

```python
class Sources(unittest.TestCase):
    """The USGS files a recipe reads: downloaded once, checked against the size and SHA-256 its provenance lists."""

    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.dir = Path(self.tmp.name)
        self.tiles = [{"url": f"https://example.test/{name}", "size_bytes": len(body), "sha256": sha256(body)}
                      for name, body in (("a.laz", b"first"), ("b.tif", b"second"))]
        self.server = Server({"https://example.test/a.laz": b"first", "https://example.test/b.tif": b"second"})

    def test_they_are_downloaded_checked_and_used_again(self):
        said = []
        paths = fetch_data.source_files(self.tiles, self.dir, self.server, said.append)
        self.assertEqual(paths, [self.dir / "a.laz", self.dir / "b.tif"])
        self.assertEqual([p.read_bytes() for p in paths], [b"first", b"second"])
        self.assertEqual(said, ["  a.laz: downloaded, 0.0 MB", "  b.tif: downloaded, 0.0 MB"])
        (self.dir / "b.tif").write_bytes(b"spoiled")
        self.assertEqual(fetch_data.source_files(self.tiles, self.dir, self.server, said.append), paths)
        self.assertEqual(len(self.server.asked), 3)  # a.laz once; b.tif again, as the copy here was not it
        self.assertEqual((self.dir / "b.tif").read_bytes(), b"second")

    def test_a_file_that_is_not_the_listed_one_raises_and_is_not_kept(self):
        self.server.bodies["https://example.test/b.tif"] = b"second, changed"
        with self.assertRaisesRegex(ValueError, "https://example.test/b.tif is not the file the provenance lists "
                                                "[(]15 bytes, SHA-256 [0-9a-f]{12}[.][.][.][)]"):
            fetch_data.source_files(self.tiles, self.dir, self.server, lambda line: None)
        self.assertEqual(sorted(p.name for p in self.dir.iterdir()), ["a.laz"])
```

- [ ] **Step 2: Run them to see them fail**

```bash
PX="/Users/alarion239/.pixi/bin/pixi run --frozen --manifest-path /Users/alarion239/Desktop/hurc_ws/wt-data/pixi.toml"
cd /Users/alarion239/Desktop/hurc_ws/wt-data/rover_sim/tests && $PX python -m unittest test_fetch_data.Sources 2>&1 | grep -E "AttributeError|^Ran|^FAILED" | sort | uniq -c
```

Expected: `AttributeError: module 'fetch_data' has no attribute 'source_files'` (2 times), `Ran 2 tests`, `FAILED (errors=2)`.

- [ ] **Step 3: Implement source_files**

In `rover_sim/tools/fetch_data.py`, after the function `requests_of`, add:

```python


def source_files(tiles, directory, get=download, log=print):
    """The files of `tiles` ({url, size_bytes, sha256}) in directory, in their order: downloaded unless a file with
    that SHA-256 is there, and checked against the size and SHA-256 listed (rockyweb's ETags are no checksums, so
    the provenance pins every file itself)."""
    paths = []
    for tile in tiles:
        path = Path(directory) / tile["url"].rsplit("/", 1)[1]
        if not (path.is_file() and file_sha256(path) == tile["sha256"]):
            body = get(tile["url"])
            digest = hashlib.sha256(body).hexdigest()
            if (len(body), digest) != (tile["size_bytes"], tile["sha256"]):
                raise ValueError(f"{tile['url']} is not the file the provenance lists ({len(body)} bytes, SHA-256 "
                                 f"{digest[:12]}...)")
            part = path.with_name(f".{path.name}.part")
            part.write_bytes(body)
            os.replace(part, path)
            log(f"  {path.name}: downloaded, {len(body) / 1e6:.1f} MB")
        paths.append(path)
    return paths
```

`download` reads a body into memory (the largest file is 61 MB).

- [ ] **Step 4: Run the tests**

```bash
PX="/Users/alarion239/.pixi/bin/pixi run --frozen --manifest-path /Users/alarion239/Desktop/hurc_ws/wt-data/pixi.toml"
cd /Users/alarion239/Desktop/hurc_ws/wt-data/rover_sim/tests && $PX python -m unittest test_fetch_data 2>&1 | tail -3
```

Expected: `Ran 22 tests`, `OK`.

- [ ] **Step 5: Commit**

```bash
cd /Users/alarion239/Desktop/hurc_ws/wt-data && git add rover_sim/tools/fetch_data.py rover_sim/tests/test_fetch_data.py && git commit -q -F - <<'EOF'
Lidar from USGS: fetch-data downloads USGS files pinned by SHA-256

source_files puts the files a recipe reads into a directory: each is
downloaded unless a file with its SHA-256 is already there, and a body
whose size or SHA-256 is not the provenance's raises before it is written.

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
EOF
git log --oneline -1
```

### Task 10: fetch-data: rebuilds by recipe

**Files:** Modify `rover_sim/tools/fetch_data.py`, `rover_sim/tests/test_fetch_data.py`.

- [ ] **Step 1: Write the tests**

In `rover_sim/tests/test_fetch_data.py`:

Replace the module docstring with:

```python
"""pixi run fetch-data (tools/fetch_data.py) without the network: the raster
manifest against what the simulation reads and what is here, the dry run of
a fresh clone, official tiles made into a raster and checked, nothing
installed that does not match, the lidar rebuilt by a recipe from pinned
USGS files and kept within its stats' tolerance, the rasters it cannot
rebuild named with the worlds that need them and how to get them, without
failing the run, and its exit status in --help."""
```

After `from pathlib import Path` add `from unittest import mock`; after `import fetch_data  # noqa: E402` add `import usgs_lidar  # noqa: E402`.

After the function `sha256` (before `def run(`) add:

```python


LIDAR_URL = "https://example.test/lpc/lidar.laz"


def add_lidar(data, offset):
    """Adds to the manifest in data two rasters that a made-up recipe of tools/usgs_lidar.py, "made_up", makes from
    one USGS file: dem/lidar.tif (required) and dem/lidar_map.tif (not required), with their stats. Returns ({url:
    body} of the file, the recipe, the list of its runs); the DEM it makes is the file's plus offset["m"]."""
    body = raster_bytes(dem_band(32, 32) + 7.0, TRANSFORM)
    runs = []

    def make(files, data_dir, work):
        runs.append(files)
        bands, transform = dem.read_raster(files["lidar_tiles"][0])
        made = {"dem/lidar.tif": work / "lidar.tif", "dem/lidar_map.tif": work / "lidar_map.tif"}
        fetch_data.write_geotiff(made["dem/lidar.tif"], bands + offset["m"], fetch_data.geotags(transform))
        fetch_data.write_geotiff(made["dem/lidar_map.tif"], np.full_like(bands, 2.0), fetch_data.geotags(transform))
        return made

    (data / "dem" / "lidar.json").write_text(json.dumps({
        "recipe": "made_up", "lidar_tiles": [{"url": LIDAR_URL, "size_bytes": len(body), "sha256": sha256(body)}]}))
    manifest = json.loads((data / fetch_data.MANIFEST).read_text())
    for name, bands, required in (("lidar", dem_band(32, 32) + 7.0, True),
                                  ("lidar_map", np.full((1, 32, 32), 2.0), False)):
        whole = raster_bytes(bands, TRANSFORM)
        (data / "whole.tif").write_bytes(whole)
        manifest["rasters"].append({
            "path": f"dem/{name}.tif", "provenance": "dem/lidar.json", "required": required, "official": True,
            "bytes": len(whole), "sha256": sha256(whole),
            "content_sha256": fetch_data.content_sha256(data / "whole.tif"),
            "shape": [1, 32, 32], "dtype": "float32", "transform": list(TRANSFORM),
            "stats": fetch_data.stats(dem.read_raster(data / "whole.tif")[0])})
        (data / "whole.tif").unlink()
    (data / fetch_data.MANIFEST).write_text(json.dumps(manifest))
    return {LIDAR_URL: body}, usgs_lidar.Recipe(("lidar_tiles",), ("dem/lidar.tif", "dem/lidar_map.tif"), make), runs
```

Insert before `class Help(unittest.TestCase):`:

```python
class Lidar(unittest.TestCase):
    """A raster whose provenance names a recipe of tools/usgs_lidar.py: rebuilt from the USGS files listed there and
    kept when its content matches or, not bit-identical, its stats are within the tolerance."""

    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        (Path(self.tmp.name) / "data").mkdir()
        self.data, bodies = case(Path(self.tmp.name) / "data", dem_band())
        self.offset = {"m": 0.0}
        lidar_bodies, recipe, self.runs = add_lidar(self.data, self.offset)
        self.bodies = {**bodies, **lidar_bodies}
        self.temp = Path(self.tmp.name) / "temp"  # where the run's temporary directories go
        self.temp.mkdir()
        for patcher in (mock.patch.dict(usgs_lidar.RECIPES, {"made_up": recipe}),
                        mock.patch.object(tempfile, "tempdir", str(self.temp))):
            patcher.start()
            self.addCleanup(patcher.stop)

    def files(self):
        return sorted(p.name for p in (self.data / "dem").iterdir())

    def test_a_recipe_makes_its_rasters_in_one_run_and_they_are_verified(self):
        code, lines = run(self.data, Server(self.bodies), "--all")
        self.assertEqual((code, lines), (0, ["dem/made_up.tif: fetched from its official source, content verified",
                                             "  lidar.laz: downloaded, 0.0 MB",
                                             "dem/lidar.tif: rebuilt from the USGS lidar, content verified",
                                             "dem/lidar_map.tif: rebuilt from the USGS lidar, content verified"]))
        self.assertEqual(len(self.runs), 1)
        self.assertEqual(self.files(), ["lidar.json", "lidar.tif", "lidar_map.tif", "made_up.json", "made_up.tif"])
        self.assertEqual(list(self.temp.iterdir()), [])  # the downloads and the work directory are gone
        self.assertEqual(run(self.data, Server({}), "--all", "--verify"),
                         (0, ["dem/made_up.tif: verified", "dem/lidar.tif: verified", "dem/lidar_map.tif: verified"]))

    def test_the_default_run_makes_only_the_required_raster(self):
        code, lines = run(self.data, Server(self.bodies))
        self.assertEqual((code, lines[-1]), (0, "dem/lidar.tif: rebuilt from the USGS lidar, content verified"))
        self.assertNotIn("lidar_map.tif", self.files())

    def test_the_dry_run_says_what_it_would_download(self):
        server = Server(self.bodies)
        self.assertEqual(run(self.data, server, "--dry-run", "--all"),
                         (0, ["dem/made_up.tif: missing, would fetch from the official source, 4 request(s)",
                              "dem/lidar.tif: missing, would rebuild it from the USGS lidar, 1 file(s), 0 MB",
                              "dem/lidar_map.tif: missing, would rebuild it from the USGS lidar, 1 file(s), 0 MB"]))
        self.assertEqual((server.asked, self.runs), ([], []))

    def test_a_file_that_is_not_the_listed_one_fails_and_installs_nothing(self):
        bad = b"X" + self.bodies[LIDAR_URL][1:]
        self.bodies[LIDAR_URL] = bad
        err = io.StringIO()
        code, lines = run(self.data, Server(self.bodies), err=err)
        self.assertEqual((code, lines[1:]), (1, [
            f"dem/lidar.tif: FAILED, the rebuild from the USGS lidar failed ({LIDAR_URL} is not the file the "
            f"provenance lists ({len(bad)} bytes, SHA-256 {sha256(bad)[:12]}...))"]))
        self.assertEqual((self.runs, self.files()), ([], ["lidar.json", "made_up.json", "made_up.tif"]))
        self.assertIn("1 raster(s) not fetched (FAILED above)", err.getvalue())

    def test_a_rebuild_within_the_tolerance_is_kept_and_says_so(self):
        self.offset["m"] = 0.0002  # what another platform's rounding might give
        code, lines = run(self.data, Server(self.bodies))
        how = "content within 0.001 of rasters.json's stats, not bit-identical"
        self.assertEqual((code, lines[-1]), (0, f"dem/lidar.tif: rebuilt from the USGS lidar, {how}"))
        self.assertEqual(run(self.data, Server({}), "--verify"),
                         (0, ["dem/made_up.tif: verified", f"dem/lidar.tif: verified, {how}"]))

    def test_a_rebuild_beyond_the_tolerance_fails(self):
        self.offset["m"] = 0.01  # a wrong tile or step: centimetres
        code, lines = run(self.data, Server(self.bodies))
        self.assertEqual((code, lines[-1]), (1, "dem/lidar.tif: FAILED, the rebuild from the USGS lidar gives other "
                                                "pixels than rasters.json lists"))
        self.assertNotIn("lidar.tif", self.files())

    def test_sources_keeps_the_downloads_for_the_next_run(self):
        sources = Path(self.tmp.name) / "sources"
        self.assertEqual(run(self.data, Server(self.bodies), "--sources", str(sources))[0], 0)
        self.assertEqual(sorted(p.name for p in sources.iterdir()), ["lidar.laz"])
        (self.data / "dem" / "lidar.tif").unlink()
        server = Server({})
        self.assertEqual(run(self.data, server, "--sources", str(sources)),
                         (0, ["dem/made_up.tif: present",
                              "dem/lidar.tif: rebuilt from the USGS lidar, content verified"]))
        self.assertEqual(server.asked, [])
```

The made-up rasters are 32 x 32 so that each of the 16 x 16 stat blocks holds pixels.

- [ ] **Step 2: Run them to see them fail**

```bash
PX="/Users/alarion239/.pixi/bin/pixi run --frozen --manifest-path /Users/alarion239/Desktop/hurc_ws/wt-data/pixi.toml"
cd /Users/alarion239/Desktop/hurc_ws/wt-data/rover_sim/tests && $PX python -m unittest test_fetch_data.Lidar 2>&1 | grep -E "^(FAIL|ERROR):|^Ran|^FAILED"
```

Expected: `Ran 7 tests`, `FAILED (failures=..., errors=...)`: fetch-data treats the made-up rasters as exportImage rasters with no requests (`FAILED, the official source failed (the requests leave 1024 pixels uncovered)`), the dry run says `0 request(s)`, and `--sources` is an unknown argument (exit 2).

- [ ] **Step 3: Implement the recipe path**

In `rover_sim/tools/fetch_data.py`:

Replace the docstring's opening (from its first line through `drive and the tests' skips say why (world_missing).`):

```python
"""Fetch the git-ignored terrain rasters: pixi run fetch-data [--all] [--dry-run] [--verify].

data/rasters.json lists every raster under data/ that git ignores, with its
size, its SHA-256, the SHA-256 of its content (the pixels and georeferencing
urc.dem.read_raster reads) and the tracked .json that says where it came
from. A missing raster is fetched from its official source when it is one or
more ImageServer exportImage requests (`official`; its .json lists them as
url, urls or requests): each tile is placed on the raster's grid by its own
georeferencing, the mosaic is written as a deflate GeoTIFF and kept only if
its content matches. The others fetch-data cannot rebuild yet (the 0.5 m
lidar DEMs and their source map, made by processing that is not in the
repository; the NAIP 2021 route mosaic, whose .json gives only a URL
template): for each one missing it names the worlds that need it (`worlds`)
and what is planned (`rebuild`), and a teammate's clone lends it
(tools/link_data.sh <that clone>). There is no other copy to fetch from;
without such a raster sim-worlds skips the worlds that need it, and sim,
drive and the tests' skips say why (world_missing).
```

with:

```python
"""Fetch the git-ignored terrain rasters: pixi run fetch-data [--all] [--dry-run] [--verify] [--sources DIR].

data/rasters.json lists every raster under data/ that git ignores, with its
size, its SHA-256, the SHA-256 of its content (the pixels and georeferencing
urc.dem.read_raster reads) and the tracked .json that says where it came
from. A missing raster is rebuilt from its official source (`official`):
from one or more ImageServer exportImage requests (its .json lists them as
url, urls or requests), each tile placed on the raster's grid by its own
georeferencing and the mosaic written as a deflate GeoTIFF; or, for the
0.5 m lidar DEMs and their source map, by the recipe of tools/usgs_lidar.py
that its .json names, from the USGS tiles and point clouds listed there,
each download checked against its size and SHA-256 (--sources DIR keeps
them for the next run). A raster is kept only if its content matches; a
lidar rebuild that is not bit-identical (another platform's floating point)
is kept when its stats (min, max and 16 x 16 block means) are within
TOLERANCE of the manifest's, and fetch-data says so. A raster that is not
official fetch-data cannot rebuild: for each one missing it names the worlds
that need it (`worlds`) and what is planned (`rebuild`), and a clone that
has it lends it (tools/link_data.sh <that clone>); without such a raster
sim-worlds skips the worlds that need it, and sim, drive and the tests'
skips say why (world_missing).
```

The paragraphs from `By default only the rasters the simulation reads` on stay as they are (`main` takes the `Exit status` paragraph as the help's epilog).

Replace the imports:

```python
import argparse
import hashlib
import json
import os
import sys
import tempfile
```

with:

```python
import argparse
import contextlib
import hashlib
import json
import os
import shutil
import sys
import tempfile
```

Replace:

```python
SIM_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(SIM_DIR))
from urc import dem  # noqa: E402
```

with:

```python
SIM_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(SIM_DIR))
sys.path.insert(0, str(SIM_DIR / "tools"))  # usgs_lidar beside this file, also under python -I
from urc import dem  # noqa: E402
import usgs_lidar  # noqa: E402
```

(`python -I` does not put the script's folder on the path; the scripts of Tasks 11 and 12 run fetch-data that way.)

Between the functions `requests_of` and `source_files`, add:

```python


def recipe_of(provenance):
    """The tools/usgs_lidar.py recipe a raster's .json names, or None."""
    return usgs_lidar.RECIPES.get(provenance.get("recipe"))


def inputs_of(provenance):
    """The USGS files a recipe reads, as the raster's .json lists them: [{url, size_bytes, sha256, ...}]."""
    return [tile for key in recipe_of(provenance).inputs for tile in provenance[key]]
```

After the function `source_files`, add:

```python


class Rebuilds:
    """The rasters the recipes of tools/usgs_lidar.py made in this run: a recipe that makes several (the route area's
    DEM and its source map) runs once. The USGS files go to `sources` (kept) or to a temporary directory, the
    recipes work in temporary directories; all of them are removed when `stack` closes."""

    def __init__(self, data_dir, sources, get, stack):
        self.data_dir, self.sources, self.get, self.stack, self.made = Path(data_dir), sources, get, stack, {}

    def temporary(self, prefix):
        return Path(self.stack.enter_context(tempfile.TemporaryDirectory(prefix=prefix)))

    def output(self, entry, provenance, log=print):
        """The file in a work directory that entry's recipe made for it; runs the recipe if this run has not."""
        name = provenance["recipe"]
        if name not in self.made:
            try:
                if self.sources is None:
                    self.sources = self.temporary("fetch-data-sources-")
                Path(self.sources).mkdir(parents=True, exist_ok=True)
                recipe = recipe_of(provenance)
                files = {key: source_files(provenance[key], self.sources, self.get, log) for key in recipe.inputs}
                self.made[name] = recipe.make(files, self.data_dir, self.temporary("fetch-data-work-"))
            except Exception as e:
                self.made[name] = e
        made = self.made[name]
        if isinstance(made, Exception):
            raise made
        return made[entry["path"]]
```

After the function `mosaic`, add:

```python


def plan(entry, data_dir):
    """What the dry run would do for a missing raster fetch-data can fetch."""
    provenance = json.loads((Path(data_dir) / entry["provenance"]).read_text())
    if recipe_of(provenance) is None:
        return f"would fetch from the official source, {len(requests_of(provenance))} request(s)"
    tiles = inputs_of(provenance)
    return (f"would rebuild it from the USGS lidar, {len(tiles)} file(s), "
            f"{sum(t['size_bytes'] for t in tiles) / 1e6:.0f} MB")
```

Replace everything from the line `def fetch(entry, data_dir, get=download, log=print):` to the end of the file with:

```python
def fetch(entry, data_dir, get=download, log=print, rebuilds=None):
    """Fetch one raster from its official source into data_dir; True once it is in place and its content
    verified (rebuilds: the run's Rebuilds, for a raster a recipe makes)."""
    path = data_dir / entry["path"]
    path.parent.mkdir(parents=True, exist_ok=True)
    part = path.with_name(f".{path.name}.part")
    provenance = json.loads((data_dir / entry["provenance"]).read_text())
    lidar = recipe_of(provenance) is not None
    source = "the rebuild from the USGS lidar" if lidar else "the official source"
    with contextlib.ExitStack() as stack:
        try:
            try:
                if lidar:
                    shutil.copyfile((rebuilds or Rebuilds(data_dir, None, get, stack)).output(entry, provenance, log),
                                    part)
                else:
                    mosaic(entry, provenance, get, part)
            except Exception as e:  # any failure of the source
                log(f"{entry['path']}: FAILED, {source} failed ({e})")
                return False
            how = verdict(entry, part)
            if how is None:
                log(f"{entry['path']}: FAILED, {source} {'gives' if lidar else 'now returns'} other pixels than "
                    "rasters.json lists")
                return False
            os.replace(part, path)
            log(f"{entry['path']}: {'rebuilt from the USGS lidar' if lidar else 'fetched from its official source'}, "
                f"{how}")
            return True
        finally:
            part.unlink(missing_ok=True)


def main(argv=None, get=download):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0],
                                     epilog=__doc__[__doc__.index("Exit status"):],  # the docstring's last paragraph
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--all", action="store_true",
                        help="also the rasters only the research and the relief tool read")
    parser.add_argument("--dry-run", action="store_true", help="say what would be fetched; fetch nothing")
    parser.add_argument("--verify", action="store_true",
                        help="check the rasters here against rasters.json; fetch nothing")
    parser.add_argument("--data", type=Path, default=DATA_DIR, help=f"data directory (default {DATA_DIR})")
    parser.add_argument("--sources", type=Path,
                        help="keep the USGS lidar files a rebuild downloads in this directory, and use the ones "
                             "already there (default: a temporary directory, removed afterwards)")
    args = parser.parse_args(argv)
    failed, missing = 0, []
    with contextlib.ExitStack() as stack:
        rebuilds = Rebuilds(args.data, args.sources, get, stack)
        for entry in load_manifest(args.data)["rasters"]:
            if not (args.all or entry["required"]):
                continue
            path = args.data / entry["path"]
            if not (entry["official"] or path.exists()):  # nothing to fetch it from: say so, whatever the mode
                print(f"{entry['path']}: missing, {needed_by(entry)}; {cannot_fetch(entry)}")
                missing.append(entry)
            elif args.verify:
                state = check(entry, path)
                print(f"{entry['path']}: {state}")
                failed += not state.startswith("verified")
            elif path.exists():
                print(f"{entry['path']}: present")
            elif args.dry_run:
                print(f"{entry['path']}: missing, {plan(entry, args.data)}")
            elif not fetch(entry, args.data, get, rebuilds=rebuilds):
                failed += 1
            sys.stdout.flush()  # each line as it comes (a rebuild takes a while), and before the summary on stderr
    if failed and not args.verify:
        print(f"fetch-data: {failed} raster(s) not fetched (FAILED above); a clone that has them can link them "
              "here: rover_sim/tools/link_data.sh <that clone> (README, Simulation)", file=sys.stderr)
    if missing:
        worlds = sorted({world for entry in missing for world in entry["worlds"]})
        print(f"fetch-data: {len(missing)} raster(s) it cannot rebuild yet are not here (above)"
              + (f"; without them sim-worlds skips {', '.join(worlds)} and the tests that need "
                 f"{'it' if len(worlds) == 1 else 'them'} skip" if worlds else ""),
              file=sys.stderr)
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
```

`fetch` now checks with `verdict` (Task 8), so an exportImage raster still has to match its content exactly (it has no stats). The work directory is on the temporary filesystem, so the result is copied next to its place and renamed there (a rename across filesystems would fail).

- [ ] **Step 4: Run the tests**

```bash
PX="/Users/alarion239/.pixi/bin/pixi run --frozen --manifest-path /Users/alarion239/Desktop/hurc_ws/wt-data/pixi.toml"
cd /Users/alarion239/Desktop/hurc_ws/wt-data/rover_sim/tests && $PX python -m unittest test_fetch_data test_usgs_lidar test_urc_unit test_station.WorldArgument test_foundations.Environment.test_run_sh_stops_without_the_world 2>&1 | tail -3
cd /Users/alarion239/Desktop/hurc_ws/wt-data && $PX python rover_sim/tools/fetch_data.py --help | head -3
```

Expected: `OK` (`test_fetch_data` 29 tests among them). These modules run no Gazebo and no server: in WT the rest of `test_foundations` and `test_launcher` would start Gazebo against PR 2's plugins in `hurc_ws/build` (Task 14 runs them in the clone). The help's usage is `usage: fetch_data.py [-h] [--all] [--dry-run] [--verify] [--data DATA]` and, on the next line, `[--sources SOURCES]`.

- [ ] **Step 5: Commit**

```bash
cd /Users/alarion239/Desktop/hurc_ws/wt-data && git add rover_sim/tools/fetch_data.py rover_sim/tests/test_fetch_data.py && git commit -q -F - <<'EOF'
Lidar from USGS: fetch-data rebuilds a raster by the recipe its provenance names

A raster whose .json names a recipe of tools/usgs_lidar.py is rebuilt from
the USGS files listed there (downloaded and checked by source_files) and
installed when it matches rasters.json, exactly or within the stats'
tolerance. A recipe that makes two rasters runs once per run; --sources DIR
keeps the downloads for the next run; the dry run says how many files and
megabytes a rebuild needs. fetch_data.py also finds usgs_lidar under
python -I.

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
EOF
git log --oneline -1
```

## Part D: the live rebuild, the manifest, the documents

### Task 11: The live rebuild of the three rasters, and what reads them

**Files:** none tracked (WT's three lidar symlinks become the rebuilt files, git-ignored). Scratch: `$S/scripts/rebuild_all.py`, `$S/work/`, `$S/rebuilt/`, `$S/downstream/`.

- [ ] **Step 1: Rebuild with both recipes from the downloaded files**

Create `$S/scripts/rebuild_all.py`:

```python
"""Both recipes of tools/usgs_lidar.py on the USGS files in <sources> (fetch_data.source_files checks each against
the provenance), each raster compared with rasters.json and copied to <out>:
python -I rebuild_all.py <rover_sim/tools> <sources> <work> <out>"""
import json
import shutil
import sys
import time
from pathlib import Path

sys.path.insert(0, sys.argv[1])
import fetch_data  # noqa: E402
import usgs_lidar  # noqa: E402

sources, work, out = (Path(a) for a in sys.argv[2:5])
manifest = {e["path"]: e for e in fetch_data.load_manifest()["rasters"]}
for name, provenance in (("mdrs_area", "dem/mdrs_area_lidar2018_0p5m.json"),
                         ("route_area", "dem/route_area_lidar_0p5m.json")):
    tiles = json.loads((fetch_data.DATA_DIR / provenance).read_text())
    recipe = usgs_lidar.RECIPES[name]
    files = {key: fetch_data.source_files(tiles[key], sources) for key in recipe.inputs}
    (work / name).mkdir(parents=True)
    start = time.time()
    made = recipe.make(files, fetch_data.DATA_DIR, work / name)
    print(f"{name}: {time.time() - start:.0f} s")
    for path, file in made.items():
        e = manifest[path]
        same_file = fetch_data.file_sha256(file) == e["sha256"]
        same_content = fetch_data.content_sha256(file) == e["content_sha256"]
        print(f"  {path}: file {'same' if same_file else 'differs'}, content {'same' if same_content else 'DIFFERS'}")
        shutil.copyfile(file, out / Path(path).name)
```

Run it (the route recipe reads WT's `route_area_3dep.tif`, verified in Task 1):

```bash
S=/private/tmp/claude-502/-Users-alarion239-Desktop-Rover/d048031d-a613-49e0-ad24-8cb3b1a40ce3/scratchpad/data
PX="/Users/alarion239/.pixi/bin/pixi run --frozen --manifest-path /Users/alarion239/Desktop/hurc_ws/wt-data/pixi.toml"
rm -rf "$S/work" "$S/rebuilt" && mkdir -p "$S/work" "$S/rebuilt"
cd /Users/alarion239/Desktop/hurc_ws/wt-data && $PX python -I "$S/scripts/rebuild_all.py" rover_sim/tools "$S/sources" "$S/work" "$S/rebuilt"; echo "exit $?"
rm -rf "$S/work"; ls -l "$S/rebuilt"
```

Expected (seconds vary):

```
mdrs_area: 6 s
  dem/mdrs_area_lidar2018_0p5m.tif: file same, content same
route_area: 12 s
  dem/route_area_lidar_0p5m.tif: file same, content same
  dem/route_area_lidar_0p5m_source.tif: file same, content same
exit 0
```

and three files of 58431428, 41457287 and 214914 bytes. No `downloaded` lines: every file is in `$S/sources` with its pinned SHA-256. If a content says `DIFFERS`, stop and debug (superpowers:systematic-debugging) before anything else: compare the environment with Task 2 Step 4 (GDAL 3.13.3, PROJ 9.8.1, OpenCV 4.13.0, numpy 2.5.3, laspy 2.7.0), check that `ls "$HOME/Library/Application Support/proj"` holds no grid files besides `cache.db` (Known 10) and that the code matches Tasks 5 to 7 (`git diff` against the listings), then compare the rebuilt raster with the file pixel by pixel (where the differences lie says which step: the blend zone, the 2020 fill, everything).

- [ ] **Step 2: What the original rasters give downstream**

The Autonomy world (gen_worlds), the relief swatches (`tools/make_relief_swatches.py`, both DEMs) and the roughness targets (`tools/terrain_targets.py`, both DEMs), with WT's rasters still the symlinked originals. Run in the background (about 5 minutes; gen_worlds makes the shared media first if they are not there, and WT gains about 700 MB of git-ignored generated models, as SRC has):

```bash
S=/private/tmp/claude-502/-Users-alarion239-Desktop-Rover/d048031d-a613-49e0-ad24-8cb3b1a40ce3/scratchpad/data
PX="/Users/alarion239/.pixi/bin/pixi run --frozen --manifest-path /Users/alarion239/Desktop/hurc_ws/wt-data/pixi.toml"
mkdir -p "$S/downstream/original" && cd /Users/alarion239/Desktop/hurc_ws/wt-data && {
  $PX python rover_sim/gen_model.py && $PX python rover_sim/gen_worlds.py \
  && (cd rover_sim && find worlds models -type f \( -path 'worlds/urc_*' -o -path 'worlds/proving_ground.*' -o -path 'models/urc_*' \) | sort | xargs shasum -a 256) > "$S/downstream/original/worlds.sha" \
  && $PX python rover_sim/tools/make_relief_swatches.py --out "$S/downstream/original/relief" \
  && $PX python rover_sim/tools/terrain_targets.py --out "$S/downstream/original/terrain_targets.json"; echo "exit $?"; } > "$S/downstream/original.log" 2>&1
tail -3 "$S/downstream/original.log"; wc -l < "$S/downstream/original/worlds.sha"
for f in "$S"/downstream/original/relief/*.npz; do cmp -s "$f" rover_sim/data/relief/$(basename "$f") && echo "$(basename "$f") same as tracked" || echo "$(basename "$f") differs from tracked"; done
cmp -s "$S/downstream/original/terrain_targets.json" rover_sim/data/research/terrain_targets.json && echo "terrain_targets same as tracked" || echo "terrain_targets differs from tracked"
git status --short
```

Expected: `exit 0`; several hundred hashed files; the five swatches and the targets the same as the tracked files (if one differs, the tool or its inputs changed since those files were made: note it for the report; the comparison in Step 4 is against this run, not against git); `git status` clean (gen_worlds' output is git-ignored, the tools wrote to `$S`).

- [ ] **Step 3: Put the rebuilt rasters in place of the symlinks**

```bash
S=/private/tmp/claude-502/-Users-alarion239-Desktop-Rover/d048031d-a613-49e0-ad24-8cb3b1a40ce3/scratchpad/data
cd /Users/alarion239/Desktop/hurc_ws/wt-data/rover_sim/data/dem
for f in mdrs_area_lidar2018_0p5m.tif route_area_lidar_0p5m.tif route_area_lidar_0p5m_source.tif; do
  [ -L "$f" ] && mv -f "$S/rebuilt/$f" "$f"
done
ls -l mdrs_area_lidar2018_0p5m.tif route_area_lidar_0p5m.tif route_area_lidar_0p5m_source.tif route_area_3dep.tif | awk '{print $1, $5, $9, $10, $11}'
```

Expected: the three lidar files are regular files (`-rw-...`) of 58431428, 41457287 and 214914 bytes; `route_area_3dep.tif` is still a symlink. `mv` replaces the links themselves; the originals in SRC and SOURCE are untouched.

- [ ] **Step 4: The same downstream results from the rebuilt rasters**

```bash
S=/private/tmp/claude-502/-Users-alarion239-Desktop-Rover/d048031d-a613-49e0-ad24-8cb3b1a40ce3/scratchpad/data
PX="/Users/alarion239/.pixi/bin/pixi run --frozen --manifest-path /Users/alarion239/Desktop/hurc_ws/wt-data/pixi.toml"
mkdir -p "$S/downstream/rebuilt" && cd /Users/alarion239/Desktop/hurc_ws/wt-data && {
  $PX python rover_sim/gen_worlds.py \
  && (cd rover_sim && find worlds models -type f \( -path 'worlds/urc_*' -o -path 'worlds/proving_ground.*' -o -path 'models/urc_*' \) | sort | xargs shasum -a 256) > "$S/downstream/rebuilt/worlds.sha" \
  && $PX python rover_sim/tools/make_relief_swatches.py --out "$S/downstream/rebuilt/relief" \
  && $PX python rover_sim/tools/terrain_targets.py --out "$S/downstream/rebuilt/terrain_targets.json"; echo "exit $?"; } > "$S/downstream/rebuilt.log" 2>&1
tail -1 "$S/downstream/rebuilt.log"
diff "$S/downstream/original/worlds.sha" "$S/downstream/rebuilt/worlds.sha" && echo "worlds identical"
diff -r "$S/downstream/original/relief" "$S/downstream/rebuilt/relief" && echo "swatches identical"
cmp "$S/downstream/original/terrain_targets.json" "$S/downstream/rebuilt/terrain_targets.json" && echo "targets identical"
$PX python rover_sim/tools/fetch_data.py --verify --all | awk '{print $NF}' | sort | uniq -c
```

Run it in the background. Expected: `exit 0`, `worlds identical`, `swatches identical`, `targets identical`, `  10 verified`. Then `rm -rf "$S/downstream" "$S/rebuilt"`. Nothing to commit.

### Task 12: The manifest: the lidar official

**Files:** Modify `rover_sim/data/rasters.json`, `rover_sim/data/dem/mdrs_area_lidar2018_0p5m.json`, `rover_sim/data/dem/route_area_lidar_0p5m.json`, `rover_sim/tools/fetch_data.py` (`LINK`), `rover_sim/tests/test_fetch_data.py`, `rover_sim/tests/test_urc_unit.py`, `rover_sim/gen_worlds.py` (docstring). Scratch: `$S/scripts/make_official.py`.

- [ ] **Step 1: Write the tests of the manifest as it will be**

In `rover_sim/tests/test_fetch_data.py`:

Replace `LINK = "link it from a teammate's clone: rover_sim/tools/link_data.sh <that clone> (README, Simulation)"` with `LINK = "link it from a clone that has it: rover_sim/tools/link_data.sh <that clone> (README, Simulation)"`.

In `class Manifest`, replace the method `test_each_raster_has_its_provenance_and_an_official_one_its_requests` with:

```python
    def test_each_raster_has_its_provenance_and_an_official_one_its_source(self):
        """An official raster's .json lists its exportImage requests, or names the recipe of tools/usgs_lidar.py that
        makes it and lists every file that recipe reads with its size and SHA-256; a lidar rebuild has its stats."""
        for e in MANIFEST["rasters"]:
            with self.subTest(e["path"]):
                provenance = json.loads((fetch_data.DATA_DIR / e["provenance"]).read_text())
                requests = fetch_data.requests_of(provenance)
                recipe = fetch_data.recipe_of(provenance)
                exported = bool(requests) and all("exportImage" in url for url in requests)
                self.assertEqual(exported or recipe is not None, e["official"])
                self.assertEqual(recipe is not None, "stats" in e)
                if recipe is not None:
                    self.assertIn(e["path"], recipe.outputs)
                    for tile in fetch_data.inputs_of(provenance):
                        self.assertRegex(tile["sha256"], "^[0-9a-f]{64}$")
                        self.assertGreater(tile["size_bytes"], 0)

    def test_the_lidar_comes_from_usgs(self):
        """The recipes read USGS's own files only: the OPR DEM tiles on The National Map's S3 bucket and the point
        clouds on rockyweb (the user, 2026-10-09: no data release; the lidar DEMs rebuilt from USGS's public lidar)."""
        usgs = ("https://prd-tnm.s3.amazonaws.com/StagedProducts/Elevation/OPR/Projects/UT_StateWide_2018_A18/",
                "https://rockyweb.usgs.gov/vdelivery/Datasets/Staged/Elevation/LPC/Projects/"
                "UT_StatewideSouth_2020_A20/")
        counts = {}
        for e in MANIFEST["rasters"]:
            provenance = json.loads((fetch_data.DATA_DIR / e["provenance"]).read_text())
            if fetch_data.recipe_of(provenance) is not None:
                tiles = fetch_data.inputs_of(provenance)
                counts[e["path"]] = len(tiles)
                for tile in tiles:
                    self.assertTrue(tile["url"].startswith(usgs), tile["url"])
        self.assertEqual(counts, {"dem/mdrs_area_lidar2018_0p5m.tif": 11, "dem/route_area_lidar_0p5m.tif": 11,
                                  "dem/route_area_lidar_0p5m_source.tif": 11})
```

In the method `test_the_rasters_it_cannot_rebuild_name_their_worlds_and_what_is_planned`, replace its docstring:

```python
        """A raster that is not official has no copy to fetch, only a
        teammate's clone: the manifest says which worlds need it (the
        required ones, read by the simulation) and what is planned."""
```

with:

```python
        """A raster that is not official has no copy to fetch, only another
        clone: the manifest says which worlds need it (the required ones,
        read by the simulation) and what is planned. None is listed today."""
```

and its last three lines:

```python
        lidar = fetch_data.entry_of(autonomy.DEM_PATH)
        self.assertEqual((lidar["official"], lidar["worlds"]), (False, ["urc_autonomy"]))
        self.assertIn("USGS lidar", lidar["rebuild"])
```

with:

```python
        self.assertEqual([e["path"] for e in MANIFEST["rasters"] if not e["official"]], [])  # none, since the lidar
        lidar = fetch_data.entry_of(autonomy.DEM_PATH)
        self.assertEqual((lidar["official"], lidar["required"]), (True, True))
```

In `class DryRun`, replace:

```python
            self.assertIn("dem/route_area_lidar_0p5m.tif: missing, urc_autonomy needs it; fetch-data cannot rebuild it "
                          "from official sources yet (a script that rebuilds it from the USGS lidar its provenance "
                          f"lists is planned); {LINK}", lines)
```

with:

```python
            self.assertIn("dem/route_area_lidar_0p5m.tif: missing, would rebuild it from the USGS lidar, 11 file(s), "
                          "371 MB", lines)
```

In `rover_sim/tests/test_urc_unit.py`, replace the head of `class MissingRaster`:

```python
class MissingRaster(unittest.TestCase):
    """The terrain rasters are git-ignored: when a world's raster is not here,
    gen_worlds says that pixi run fetch-data fetches it; when fetch-data
    cannot rebuild it yet (rasters.json: not official), it skips that world,
    says how to get the raster and builds the others, failing only if the
    world was named; any other missing file is raised as it is."""

    LIDAR = gen_worlds.DATA_DIR / "dem" / "route_area_lidar_0p5m.tif"  # fetch-data cannot rebuild it yet
```

with:

```python
class MissingRaster(unittest.TestCase):
    """The terrain rasters are git-ignored: when a world's raster is not here,
    gen_worlds says that pixi run fetch-data fetches it; when fetch-data
    cannot rebuild it (rasters.json: not official; none is listed today, so
    a made-up entry stands in), it skips that world, says how to get the
    raster and builds the others, failing only if the world was named; any
    other missing file is raised as it is."""

    LIDAR = gen_worlds.DATA_DIR / "dem" / "route_area_lidar_0p5m.tif"  # fetch-data rebuilds it from USGS lidar
    UNOFFICIAL = gen_worlds.DATA_DIR / "dem" / "made_up_lidar.tif"
    ENTRY = {"path": "dem/made_up_lidar.tif", "official": False, "required": True, "worlds": ["made_up"],
             "rebuild": "a made-up script is planned"}
```

In its `run_main`, replace:

```python
        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        err = io.StringIO()
        with mock.patch.multiple(gen_worlds, MODELS_DIR=Path(tmp.name) / "models",
                                 WORLDS_DIR=Path(tmp.name) / "worlds", MISSIONS={"made_up": World(), "other": Other()},
                                 COURSES={}), \
                mock.patch.object(sys, "argv", ["gen_worlds.py", *names]), contextlib.redirect_stderr(err), \
                contextlib.redirect_stdout(io.StringIO()):
            gen_worlds.main()
        return built, err.getvalue()
```

with:

```python
        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        err = io.StringIO()
        real = gen_worlds.fetch_data.entry_of

        def entry_of(path, data_dir):  # the manifest, with the made-up raster fetch-data cannot rebuild
            return self.ENTRY if Path(path) == self.UNOFFICIAL else real(path, data_dir)

        with mock.patch.multiple(gen_worlds, MODELS_DIR=Path(tmp.name) / "models",
                                 WORLDS_DIR=Path(tmp.name) / "worlds", MISSIONS={"made_up": World(), "other": Other()},
                                 COURSES={}), \
                mock.patch.object(gen_worlds.fetch_data, "entry_of", entry_of), \
                mock.patch.object(sys, "argv", ["gen_worlds.py", *names]), contextlib.redirect_stderr(err), \
                contextlib.redirect_stdout(io.StringIO()):
            gen_worlds.main()
        return built, err.getvalue()
```

and replace the tests `test_a_raster_fetch_data_cannot_rebuild_skips_its_world` and `test_that_world_named_fails_the_run` with:

```python
    def test_the_autonomy_lidar_is_fetched_like_the_other_rasters(self):
        with self.assertRaises(SystemExit) as caught:
            self.run_main(self.LIDAR)
        self.assertEqual(caught.exception.code, "made_up: rover_sim/data/dem/route_area_lidar_0p5m.tif is missing; "
                                                "pixi run fetch-data fetches the terrain rasters (README, Simulation)")

    def test_a_raster_fetch_data_cannot_rebuild_skips_its_world(self):
        built, err = self.run_main(self.UNOFFICIAL)
        self.assertEqual(built, ["other"])
        self.assertEqual(err, "skipped made_up: rover_sim/data/dem/made_up_lidar.tif is missing; fetch-data cannot "
                              "rebuild it from official sources yet (a made-up script is planned); link it from a "
                              "clone that has it: rover_sim/tools/link_data.sh <that clone> (README, Simulation)\n")

    def test_that_world_named_fails_the_run(self):
        with self.assertRaises(SystemExit) as caught:
            self.run_main(self.UNOFFICIAL, "made_up", "other")
        self.assertEqual(caught.exception.code, "not generated: made_up (above)")
```

In the same file, the made-up unofficial raster's helper and test no longer speak of a teammate: replace

```python
    tiles, not exportImage requests), read by `worlds`. Returns its bytes, as a teammate's clone would have it."""
```

with

```python
    tiles, not exportImage requests), read by `worlds`. Returns its bytes, as another clone would have it."""
```

and `    def test_from_a_teammates_clone_it_is_present_and_verified(self):` with `    def test_from_another_clone_it_is_present_and_verified(self):`.

- [ ] **Step 2: Run them to see them fail**

```bash
PX="/Users/alarion239/.pixi/bin/pixi run --frozen --manifest-path /Users/alarion239/Desktop/hurc_ws/wt-data/pixi.toml"
cd /Users/alarion239/Desktop/hurc_ws/wt-data/rover_sim/tests && $PX python -m unittest test_fetch_data test_urc_unit.MissingRaster 2>&1 | grep -E "^(FAIL|ERROR):|^Ran|^FAILED"
```

Expected: failures in `test_the_lidar_comes_from_usgs` (counts `{}`: no provenance names a recipe yet), `test_the_rasters_it_cannot_rebuild_...` (three unofficial rasters), `DryRun` (the old line), the `CannotRebuild` tests and `test_a_raster_fetch_data_cannot_rebuild_skips_its_world` (the old `LINK`), `test_the_autonomy_lidar_is_fetched_like_the_other_rasters` (gen_worlds skips it instead); `Ran 35 tests`, `FAILED`.

- [ ] **Step 3: The manifest and the provenance**

Create `$S/scripts/make_official.py`:

```python
"""Marks the lidar DEMs and their source map official in rasters.json, with their stats, and names their recipe in
their provenance: python -I make_official.py <rover_sim/tools>"""
import datetime
import json
import re
import subprocess
import sys

import cv2
import laspy
import numpy as np

sys.path.insert(0, sys.argv[1])
import fetch_data  # noqa: E402
from urc import dem  # noqa: E402  (fetch_data put rover_sim/ on the path)

DATA = fetch_data.DATA_DIR
RECIPES = {"dem/mdrs_area_lidar2018_0p5m.json": "mdrs_area", "dem/route_area_lidar_0p5m.json": "route_area"}
LIDAR = ("dem/mdrs_area_lidar2018_0p5m.tif", "dem/route_area_lidar_0p5m.tif", "dem/route_area_lidar_0p5m_source.tif")
WHAT = ("The git-ignored rasters under rover_sim/data, which pixi run fetch-data (tools/fetch_data.py) fetches when "
        "missing. sha256: the file; content_sha256: its pixels and georeferencing as urc.dem.read_raster reads them; "
        "shape: bands, rows, columns; transform: the GDAL geotransform; provenance: the tracked .json that says where "
        "it came from; official: rebuilt from the official source its provenance gives, the exportImage requests it "
        "lists or the recipe of tools/usgs_lidar.py it names with the USGS files that recipe reads; stats (the lidar "
        "rebuilds): min, max and 16 x 16 block means per band, which a rebuild that is not bit-identical (another "
        "platform's floating point) must match within fetch_data.TOLERANCE; required: read by the simulation (the "
        "others only by the research and tools/make_relief_swatches.py). A raster that is not official fetch-data "
        "cannot rebuild, and there is no other copy to fetch; for each such raster worlds: the generated worlds that "
        "read it; rebuild: what is planned (none is listed today).")

gdal = " ".join(subprocess.run(["gdalinfo", "--version"], capture_output=True, text=True).stdout.split()[:2])
proj = subprocess.run(["proj"], capture_output=True, text=True).stderr.split(",")[0].replace("Rel. ", "PROJ ")
versions = f"{gdal}, {proj}, OpenCV {cv2.__version__}, numpy {np.__version__}, laspy {laspy.__version__}"
assert versions == "GDAL 3.13.3, PROJ 9.8.1, OpenCV 4.13.0, numpy 2.5.3, laspy 2.7.0", versions
today = datetime.date.today().isoformat()

for name, recipe in RECIPES.items():
    path = DATA / name
    out = {}
    for key, value in json.loads(path.read_text()).items():
        out[key] = value
        if key == "fetched":
            out["recipe"] = recipe
            out["rebuilt"] = (f"{today}: tools/usgs_lidar.py (recipe {recipe}) makes this raster from the files "
                              f"listed here with its content_sha256 in rasters.json, on osx-arm64 with {versions}")
    path.write_text(json.dumps(out, indent=2))

manifest = fetch_data.load_manifest()
manifest["what"] = WHAT
for entry in manifest["rasters"]:
    if entry["path"] in LIDAR:
        raster = DATA / entry["path"]
        assert fetch_data.content_sha256(raster) == entry["content_sha256"], raster
        entry["official"] = True
        del entry["worlds"], entry["rebuild"]
        entry["stats"] = fetch_data.stats(dem.read_raster(raster)[0])
text = json.dumps(manifest, indent=1)
# one line per list of numbers or strings, as rasters.json has them
text = re.sub(r"\[\s*([^\[\]{}]*?)\s*\]", lambda m: "[" + ", ".join(v.strip() for v in m.group(1).split(",")) + "]",
              text)
(DATA / fetch_data.MANIFEST).write_text(text + "\n")
print("official:", [e["path"] for e in manifest["rasters"] if e["official"]])
```

It computes the stats from WT's rasters, which Task 11 rebuilt and verified, and asserts their content first. Run it:

```bash
S=/private/tmp/claude-502/-Users-alarion239-Desktop-Rover/d048031d-a613-49e0-ad24-8cb3b1a40ce3/scratchpad/data
PX="/Users/alarion239/.pixi/bin/pixi run --frozen --manifest-path /Users/alarion239/Desktop/hurc_ws/wt-data/pixi.toml"
cd /Users/alarion239/Desktop/hurc_ws/wt-data && $PX python -I "$S/scripts/make_official.py" rover_sim/tools
git diff --stat; grep -c '' rover_sim/data/rasters.json
```

Expected: `official:` lists all ten rasters; the diff touches `rasters.json` and the two lidar `.json` files; rasters.json has 197 lines (each lidar entry has lost `worlds` and `rebuild` and gained a `stats` block of 16 rows of 16 block means).

- [ ] **Step 4: The neutral link wording and gen_worlds' docstring**

In `rover_sim/tools/fetch_data.py`, replace:

```python
LINK = "link it from a teammate's clone: rover_sim/tools/link_data.sh <that clone> (README, Simulation)"
```

with:

```python
LINK = "link it from a clone that has it: rover_sim/tools/link_data.sh <that clone> (README, Simulation)"
```

In `rover_sim/gen_worlds.py`, replace in the docstring:

```
The terrain rasters are git-ignored (pixi run fetch-data). A world whose
raster fetch-data cannot rebuild yet (tools/fetch_data.py: the Autonomy
world's 0.5 m lidar DEM) is skipped when it is not here, saying how to get
it; the run fails if that world was named, or if any other raster is
missing.
```

with:

```
The terrain rasters are git-ignored (pixi run fetch-data). A world whose
raster fetch-data cannot rebuild (rasters.json: not official; none is
listed today) is skipped when it is not here, saying how to get it; the
run fails if that world was named, or if any other raster is missing.
```

- [ ] **Step 5: Run the tests**

```bash
PX="/Users/alarion239/.pixi/bin/pixi run --frozen --manifest-path /Users/alarion239/Desktop/hurc_ws/wt-data/pixi.toml"
cd /Users/alarion239/Desktop/hurc_ws/wt-data/rover_sim/tests && $PX python -m unittest test_fetch_data test_usgs_lidar test_urc_unit test_station.WorldArgument test_foundations.Environment.test_run_sh_stops_without_the_world test_urc_dem test_landscape 2>&1 | tail -3
cd /Users/alarion239/Desktop/hurc_ws/wt-data && $PX python rover_sim/tools/fetch_data.py --verify --all | awk '{print $NF}' | sort | uniq -c
```

Expected: `OK` (`test_fetch_data` 30 tests, `test_usgs_lidar` 14, `test_urc_unit` 35 among them); `  10 verified`.

- [ ] **Step 6: Commit**

```bash
cd /Users/alarion239/Desktop/hurc_ws/wt-data && git add rover_sim/data/rasters.json rover_sim/data/dem/mdrs_area_lidar2018_0p5m.json rover_sim/data/dem/route_area_lidar_0p5m.json rover_sim/tools/fetch_data.py rover_sim/tests/test_fetch_data.py rover_sim/tests/test_urc_unit.py rover_sim/gen_worlds.py && git commit -q -F - <<'EOF'
Lidar from USGS: every raster is official

The 0.5 m lidar DEMs and the source map name their recipe in their
provenance and are official in rasters.json, with their stats for rebuilds
that are not bit-identical. Rebuilt on this Mac (osx-arm64, GDAL 3.13.3,
PROJ 9.8.1, OpenCV 4.13.0, numpy 2.5.3, laspy 2.7.0) from the pinned USGS
files, all three are the files of 2026-10-06 byte for byte, and the worlds,
relief swatches and roughness targets generated from them are identical.
No raster is unofficial now; the path for one stays, its message names "a
clone that has it", and gen_worlds' test of it uses a made-up raster.

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
EOF
git log --oneline -1
```

### Task 13: Documentation, the pixi comment and CI

**Files:** Modify `README.md`, `rover_sim/README.md`, `docs/sim/manual.md`, `docs/research/README.md`, `pixi.toml`, `.github/workflows/ci.yml`.

- [ ] **Step 1: README.md**

Replace the Simulation item 1 (the line that starts `1. \`pixi run fetch-data\` downloads the terrain rasters`) with:

```markdown
1. `pixi run fetch-data` gets the terrain rasters the simulation reads from their official sources, once, and checks each against `rover_sim/data/rasters.json` (git ignores them): about 90 MB of USGS elevation data and USDA NAIP imagery, and the Autonomy world's 0.5 m lidar DEM, which it rebuilds from USGS's public lidar (371 MB of point clouds and DEM tiles to download, about 1.5 GB of temporary space and 3 GB of memory, a few minutes; `--sources <folder>` keeps the downloads for another run). The tasks below generate every world, so they need these, also for the test ground. When a source is down, `rover_sim/tools/link_data.sh <a clone that has the rasters>` links them from another clone
```

Replace, in the Tests list, the CI item's last sentence `CI has only the rasters fetch-data can fetch, so the Autonomy world's tests skip there until its lidar DEM can be rebuilt` with:

```markdown
The rasters are cached by their manifest, so fetch-data downloads and rebuilds them only when `rasters.json` changes
```

- [ ] **Step 2: rover_sim/README.md**

Replace the paragraph after the quick start (it starts `Every world needs the rasters fetch-data fetches`) with:

```markdown
Every world needs the rasters fetch-data fetches, the test ground too (the tasks generate all worlds). The Autonomy world's 0.5 m lidar DEM (`route_area_lidar_0p5m.tif`) is rebuilt from USGS lidar tiles and point clouds by `tools/usgs_lidar.py` (371 MB to download, a few minutes); like every raster it is checked against `data/rasters.json`.
```

In the Layout table's `tools/`, `patches/` row, replace `Fetching the rasters, the patched rendering media` with ``Fetching the rasters (the lidar DEMs rebuilt from USGS lidar by `usgs_lidar.py`), the patched rendering media``.

- [ ] **Step 3: docs/sim/manual.md**

Replace the table row:

```markdown
| `tools/fetch_data.py`, `data/rasters.json` | `pixi run fetch-data`: fetches the git-ignored rasters from their official sources and checks them against the manifest; names those it cannot rebuild yet (the 0.5 m lidar DEMs, the NAIP 2021 route mosaic), the worlds that need them and how to get them |
```

with:

```markdown
| `tools/fetch_data.py`, `tools/usgs_lidar.py`, `data/rasters.json` | `pixi run fetch-data`: fetches the git-ignored rasters from their official sources (ImageServer requests; the 0.5 m lidar DEMs and their source map rebuilt from USGS lidar tiles and point clouds by `usgs_lidar.py`) and checks them against the manifest |
```

In "Autonomy: the real square mile", replace the paragraph:

```markdown
That processing (the gridding, the feathering, the shift) is not in the
repository, so `pixi run fetch-data` cannot rebuild the file from official
sources yet: a script that rebuilds it from the USGS tiles and point clouds
its `.json` lists is planned. Until then
link it from a teammate's clone that has it
(`rover_sim/tools/link_data.sh <that clone>`) or copy it into `data/dem/`
(`pixi run fetch-data --verify` checks it). Without it `sim-worlds` skips
this world, `pixi run sim urc_autonomy` and `pixi run drive urc_autonomy`
say what is missing, and its tests skip, saying why.
```

with:

```markdown
`pixi run fetch-data` rebuilds the file from the USGS tiles and point clouds
its `.json` lists (`tools/usgs_lidar.py`, recipe `route_area`: 5 OPR DEM
tiles and 6 LAZ tiles, 371 MB, each pinned by its SHA-256), with the steps
that made it on 2026-10-06. On macOS (Apple silicon) the rebuild is the
file byte for byte (`data/rasters.json`); where floating point rounds
differently (OpenCV's blur and GDAL's bilinear kernel may on Linux x86-64),
fetch-data keeps a rebuild whose minimum, maximum and 16 × 16 block means
are within 1 mm of the manifest's, and says it is not bit-identical.
```

In the gen_worlds bullet, replace `not \`official\`; today Autonomy's lidar DEM) is skipped when that raster is` with `not \`official\`; none today) is skipped when that raster is`.

In Troubleshooting, replace the bullet:

```markdown
- **`gen_worlds.py` cannot read a raster** (`route_area_naip2024.tif`,
  `far_dem_3dep_60km.tif`, ...) **or skips `autonomy`**
  (`route_area_lidar_0p5m.tif`): the rasters are git-ignored. `pixi run
  fetch-data` fetches the missing ones from their official sources and
  checks them against `data/rasters.json` (each raster's `.json` says where
  it came from). It cannot rebuild the 0.5 m lidar DEMs and their source map
  yet (their processing is not in the repository; a script that rebuilds
  them from USGS lidar is planned) nor the research's NAIP 2021 route
  mosaic, and there is no other copy to fetch: it names them, the worlds that
  need them, and exits 0 (1 only when a raster it can fetch fails;
  `--help`). For those, when a source fails, or in a git worktree, link them
  from a clone that has them, a teammate's or (by default) the worktree's
  main checkout: `rover_sim/tools/link_data.sh [that clone]`.
```

with:

```markdown
- **`gen_worlds.py` cannot read a raster** (`route_area_naip2024.tif`,
  `route_area_lidar_0p5m.tif`, ...): the rasters are git-ignored. `pixi run
  fetch-data` fetches the missing ones from their official sources (the
  lidar DEMs: rebuilt from USGS lidar, a few minutes) and checks them
  against `data/rasters.json` (each raster's `.json` says where it came
  from); `--all` also the research's rasters. It exits 1 when one was not
  fetched (`--help`). When a source is down, or in a git worktree, link the
  rasters from a clone that has them, by default the worktree's main
  checkout: `rover_sim/tools/link_data.sh [that clone]`.
```

- [ ] **Step 4: docs/research/README.md**

Replace:

```markdown
The lidar and colour measurements were made on the git-ignored rasters that
`rover_sim/data/rasters.json` lists. `pixi run fetch-data --all` fetches
them from their official sources, except four it cannot rebuild yet: the
0.5 m lidar DEMs (`route_area_lidar_0p5m.tif`, which the gate runs used,
its source map and `mdrs_area_lidar2018_0p5m.tif`), made by processing that
is not in the repository, and the NAIP 2021 route mosaic
(`naip2021_route_area.tif`, which `render/prototype/make_ortho.py` reads).
A script that rebuilds the lidar DEMs from USGS's public lidar is planned;
until then link them from a teammate's clone that has them,
`rover_sim/tools/link_data.sh <that clone>`.
```

with:

```markdown
The lidar and colour measurements were made on the git-ignored rasters that
`rover_sim/data/rasters.json` lists. `pixi run fetch-data --all` fetches
them from their official sources: the 0.5 m lidar DEMs
(`route_area_lidar_0p5m.tif`, which the gate runs used, its source map and
`mdrs_area_lidar2018_0p5m.tif`) rebuilt from USGS lidar tiles and point
clouds by `rover_sim/tools/usgs_lidar.py`, and the NAIP 2021 route mosaic
(`naip2021_route_area.tif`, which `render/prototype/make_ortho.py` reads)
from the USGS National Map's NAIP service.
```

- [ ] **Step 5: pixi.toml and ci.yml comments**

In `pixi.toml`, replace:

```toml
# The git-ignored terrain rasters the simulation reads, from their official sources; it names the ones it cannot
# rebuild yet (the Autonomy world's lidar: link it from a teammate's clone, README) (--all: every raster; --dry-run;
# --verify).
```

with:

```toml
# The git-ignored terrain rasters the simulation reads, from their official sources: ImageServer requests, and the
# 0.5 m lidar DEMs rebuilt from USGS lidar (tools/usgs_lidar.py) (--all: every raster; --dry-run; --verify;
# --sources DIR keeps the USGS lidar files).
```

In `.github/workflows/ci.yml`, replace:

```yaml
      # fetch-data gets the rasters it can rebuild from their official sources and fails only if one of those fails; it
      # cannot rebuild the Autonomy world's 0.5 m lidar DEM yet, so sim-worlds skips that world and the tests that
      # need it skip, saying why.
```

with:

```yaml
      # fetch-data gets every raster the simulation reads from its official source and fails if one fails; the
      # Autonomy world's 0.5 m lidar DEM it rebuilds from USGS lidar (371 MB of downloads). Off the Mac that made
      # rasters.json the rebuild may round differently in the last bits: fetch-data then keeps it within the
      # tolerance of the manifest's stats and says that it is not bit-identical.
```

- [ ] **Step 6: Check the wording and the files**

```bash
PX="/Users/alarion239/.pixi/bin/pixi run --frozen --manifest-path /Users/alarion239/Desktop/hurc_ws/wt-data/pixi.toml"
cd /Users/alarion239/Desktop/hurc_ws/wt-data && git grep -n -i "teammate" -- ':!docs/superpowers'; echo "---"
git grep -n -i "cannot rebuild" -- ':!docs/superpowers' | cut -c1-110; echo "---"
git grep -n -i "release" -- ':!docs/superpowers' ':!docs/research' ':!pixi.lock' | cut -c1-110; echo "---"
/Users/alarion239/.pixi/bin/pixi exec actionlint .github/workflows/ci.yml && echo "actionlint ok"
$PX python -c "import tomllib; tomllib.load(open('pixi.toml', 'rb')); print('pixi.toml ok')"
awk 'length > 120 {print FILENAME": "FNR}' rover_sim/tools/usgs_lidar.py rover_sim/tools/fetch_data.py rover_sim/tests/test_usgs_lidar.py rover_sim/tests/test_fetch_data.py rover_sim/tests/test_urc_unit.py .github/workflows/ci.yml
```

Expected: no `teammate` outside `docs/superpowers` (the plans and specs keep their history); `cannot rebuild` only in the generic path (`fetch_data.py`'s messages and docstring, `gen_worlds.py`, `worldfiles.py`, `station/__main__.py`, `run.sh`, their tests and the manual's gen_worlds bullet); `release` only in the uses PR 1 listed as unrelated (CMake Release, "released to autonomy", URC's DEM, Actions versions, station.js); `actionlint ok`; `pixi.toml ok`; no line over 120 columns (pixi.toml's longer task lines are older than this plan).

- [ ] **Step 7: Commit**

```bash
cd /Users/alarion239/Desktop/hurc_ws/wt-data && git add README.md rover_sim/README.md docs/sim/manual.md docs/research/README.md pixi.toml .github/workflows/ci.yml && git commit -q -F - <<'EOF'
Lidar from USGS: the documents and CI say fetch-data rebuilds every raster

README, rover_sim/README, the manual and docs/research/README no longer
send anyone to a teammate's clone for the lidar DEMs: fetch-data rebuilds
them from USGS lidar (371 MB for the Autonomy DEM, --sources keeps the
downloads), bit-identical on the Mac and within a stated tolerance where
floating point differs. link_data.sh remains for worktrees and for sources
that are down. CI's jobs and raster cache are unchanged; its comment says
what fetch-data now does on a cache miss.

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
EOF
git log --oneline -1
```

### Task 14: A fresh clone: fetch-data and the full suites

**Files:** none tracked. Scratch: `$S/ws/src` (a clone of the branch, its own environment and build), `$S/tmp`.

- [ ] **Step 1: Clone the branch into a workspace's src/ and install**

```bash
S=/private/tmp/claude-502/-Users-alarion239-Desktop-Rover/d048031d-a613-49e0-ad24-8cb3b1a40ce3/scratchpad/data
rm -rf "$S/ws" && mkdir -p "$S/ws" "$S/tmp"
git clone -q --branch data/lidar-from-usgs /Users/alarion239/Desktop/hurc_ws/src "$S/ws/src" && git -C "$S/ws/src" log --oneline -1
cd "$S/ws/src" && /Users/alarion239/.pixi/bin/pixi install --locked > "$S/ws_install.log" 2>&1; tail -1 "$S/ws_install.log"
```

Run the install in the background. Expected: the clone's last commit is Task 13's; `✔ The default environment has been installed.` (`--locked` fails if the lock does not match the manifest).

- [ ] **Step 2: fetch-data on the fresh clone**

The ImageServer rasters are downloaded (about 90 MB); the lidar files come from `$S/sources` (`--sources`), so nothing is downloaded from rockyweb again.

```bash
S=/private/tmp/claude-502/-Users-alarion239-Desktop-Rover/d048031d-a613-49e0-ad24-8cb3b1a40ce3/scratchpad/data
cd "$S/ws/src" && /Users/alarion239/.pixi/bin/pixi run fetch-data --dry-run | grep lidar
{ TMPDIR="$S/tmp" /Users/alarion239/.pixi/bin/pixi run fetch-data --sources "$S/sources"; echo "exit $?"
  TMPDIR="$S/tmp" /Users/alarion239/.pixi/bin/pixi run fetch-data --all --sources "$S/sources"; echo "exit $?"
  /Users/alarion239/.pixi/bin/pixi run fetch-data --all --verify; echo "exit $?"; } > "$S/ws_fetch.log" 2>&1
cat "$S/ws_fetch.log"; ls "$S/tmp" | wc -l
```

Run the second block in the background. Expected: the dry run says `dem/route_area_lidar_0p5m.tif: missing, would rebuild it from the USGS lidar, 11 file(s), 371 MB`; then

```
dem/far_dem_3dep_60km.tif: fetched from its official source, content verified
dem/mdrs_area_3dep.tif: fetched from its official source, content verified
dem/route_area_3dep.tif: fetched from its official source, content verified
dem/route_area_lidar_0p5m.tif: rebuilt from the USGS lidar, content verified
imagery/naip2021_far_60km.tif: fetched from its official source, content verified
imagery/route_area_naip2024.tif: fetched from its official source, content verified
exit 0
```

then the `--all` run: the six `present`, `dem/mdrs_area_lidar2018_0p5m.tif` and `dem/route_area_lidar_0p5m_source.tif` `rebuilt from the USGS lidar, content verified`, `imagery/mdrs_area_naip2024.tif` and `imagery/naip2021_route_area.tif` `fetched from its official source, content verified`, `exit 0`; then ten `verified`, `exit 0`; `0` (no temporary directory left). If a 3DEP or NAIP 2024 raster says `FAILED, the official source now returns other pixels`, that is PR 1's path and a change at the service, not this PR's: report it, and link that raster from WT (`ln -s`) to go on.

- [ ] **Step 3: The full suites (background, about 20 minutes)**

```bash
S=/private/tmp/claude-502/-Users-alarion239-Desktop-Rover/d048031d-a613-49e0-ad24-8cb3b1a40ce3/scratchpad/data
cd "$S/ws/src" && GZ_IP=127.0.0.1 GZ_PARTITION=data_suite_$$ /Users/alarion239/.pixi/bin/pixi run test > "$S/ws_test.log" 2>&1; echo "exit $?" >> "$S/ws_test.log"
```

Then:

```bash
S=/private/tmp/claude-502/-Users-alarion239-Desktop-Rover/d048031d-a613-49e0-ad24-8cb3b1a40ce3/scratchpad/data
grep -E "tests passed|^Summary" "$S/ws_test.log"; grep -E "^Ran |^OK|^FAILED|^exit" "$S/ws_test.log"
grep -o "skipped '[^']*'" "$S/ws_test.log" | sort | uniq -c
grep -c "is not here" "$S/ws_test.log"
```

Expected: colcon's summary with every package finished and its tests passed; `Ran 553 tests` (525 + 14 in `test_usgs_lidar` + 13 in `test_fetch_data` + 1 in `test_urc_unit`), `OK` with the skips of a Mac with a GPU (the opt-in performance and slow tests) and PR 1's 2 expected failures, `exit 0`; no skip reason that says a raster or world `is not here` (`0`). A failure here: run that test alone in the clone (`cd "$S/ws/src/rover_sim/tests" && pixi run python -m unittest -v <test id>`); if it also fails on `import/simulation` (a clone of that branch in `$S/ws_base/src`, same steps), it is not this PR's; report it.

### Task 15: Hand-off: cleanup and report

**Files:** none.

- [ ] **Step 1: The branch as it will be pushed; nothing pushed; SRC and SOURCE untouched**

```bash
cd /Users/alarion239/Desktop/hurc_ws/wt-data && git status --short && git log --oneline import/simulation..data/lidar-from-usgs
git ls-remote --heads origin data/lidar-from-usgs | wc -l; git rev-parse --short origin/import/simulation
git -C /Users/alarion239/Desktop/hurc_ws/src status --short | head -5; git -C /Users/alarion239/Desktop/Rover status --short | head -5
```

Expected: WT clean; the commits of Tasks 2 to 13 (pixi, provenance pins, NAIP 2021, three usgs_lidar commits, three fetch-data commits, every raster official, documents) on top of this plan's commit; `0` (the branch is not on GitHub); `b496057`; SRC's and SOURCE's status as before this plan (SRC may show PR 2's own work in progress, which this plan did not touch).

- [ ] **Step 2: Remove the temporary files**

```bash
S=/private/tmp/claude-502/-Users-alarion239-Desktop-Rover/d048031d-a613-49e0-ad24-8cb3b1a40ce3/scratchpad/data
rm -rf "$S/ws" "$S/ws_base" "$S/sources" "$S/tmp" "$S/scripts" "$S/work" "$S/rebuilt" "$S/downstream" "$S/naip" \
  "$S"/*.log "$S"/*.json "$S"/*.txt "$S/pixi.lock.before"
ls "$S"; /Users/alarion239/.pixi/bin/pixi clean cache --exec 2>/dev/null; true
```

Expected: only `prov` is left. WT keeps its environment, its rebuilt lidar rasters (real files) and the other rasters' symlinks, and the generated worlds.

- [ ] **Step 3: Report to the user**

Report: the branch and its commits; that the three lidar rasters rebuilt here are the 2026-10-06 files byte for byte and the worlds, swatches and targets made from them identical (Task 11); the fresh-clone fetch-data run and the suite counts (Task 14); that nothing is pushed; and what is open: the push of `data/lidar-from-usgs` and its pull request (after PR 1, which it is based on). The pull request's description also says:

- Linux was not run here (no Docker). CI's Ubuntu job rebuilds the Autonomy DEM on its first run: bit-identical or, if OpenCV's blur or GDAL's bilinear kernel round differently on x86-64, within the stated tolerance (fetch-data says which). The macOS job should match byte for byte.
- The lock changes by four packages (laspy, lazrs-python and their m2r2 and mistune), insertions only.
- CI's raster cache key changes with rasters.json, so the first run of the pull request fetches and rebuilds every raster (about 460 MB from USGS and USDA, a few minutes).
- The recipe keeps the measured datum shift (-0.8925 m E, +0.6175 m N): an empirical alignment to the 3DEP and NAIP services, not a derivation.

---

## Acceptance

| Criterion | Where it is shown |
|---|---|
| fetch-data rebuilds each raster it could not rebuild: the two lidar DEMs and the source map from USGS lidar, the NAIP 2021 mosaic from USGS's ImageServer | Task 4 Step 5, Task 11 Step 1, Task 14 Step 2 |
| Official public sources only, USGS (no USDA or UGRC needed) | Decision 10, `test_the_lidar_comes_from_usgs` (Task 12) |
| Deterministic and verifiable: inputs pinned by SHA-256, output checked against rasters.json | Task 3, Task 9, Task 10 |
| Bit-identical where realistic (macOS), else a stated tolerance against manifest stats | Task 11 Step 1 (byte for byte), Task 8, Task 10 (tolerance tests), Task 12 (stats) |
| The downstream worlds checked | Task 11 Steps 2-4 (worlds, relief swatches, roughness targets identical), Task 14 Step 3 (the suite with the Autonomy world) |
| pixi dependencies for osx-arm64 and linux-64 only, only what is needed | Task 2 |
| Offline unit tests with small fixtures | Tasks 5-10, 12 (`test_usgs_lidar` 14, `test_fetch_data` +13) |
| A live reproduction run compared with the existing files, within the disk budget | Task 11, Task 14 (`$S` under 2.5 GB) |
| CI uses it with caching, no gigabytes per run | Decision 8, Task 13 Step 5 (cache keyed by rasters.json, 371 MB on a miss only) |
| No more "link from a teammate" in the documents | Task 13 Step 6 |
| Nothing pushed, SRC and SOURCE untouched | Rules, Task 15 Step 1 |

## Risks

| Risk | Handling |
|---|---|
| Linux x86-64 (CI) rounds OpenCV's GaussianBlur, the distance transforms or GDAL's bilinear kernel differently, so the rebuild is not bit-identical there | The stats tolerance (Decision 5) keeps it and says so; the macOS job stays exact. If even the stats fail on Linux (more than 1 mm), look at where: a systematic difference points to a library version, not rounding. A world test that pins Autonomy numbers more tightly than the terrain's rounding would show on Linux only; it is fixed in that test |
| PROJ's network or a local geoid grid changes the warp (the environment sets PROJ_NETWORK=ON; `projsync` or a proj-data package would add grids) | Every warp names its operation (`-ct`) and runs with PROJ_NETWORK=OFF (Decision 2, Known 4) |
| A newer GDAL, PROJ, OpenCV, numpy or laspy changes the last bits | GDAL and PROJ pinned to their minor versions (Task 2); the others fall under the tolerance; the content check catches anything larger |
| USGS changes or removes a file (rockyweb is a staging server; the LAZ tiles are not on the S3 bucket) | The SHA-256 pins fail the rebuild before processing, with the URL. Fallback (not code): the AWS EPT copy `usgs-lidar-public/UT_StatewideSouth_3_2020` holds the same points (Z and class identical, XY within 0.4 mm after a plain 2D EPSG:3857 to EPSG:6341 transform), which would give a close rebuild within the tolerance at best, not a bit-identical one |
| The USGS NAIP Plus service is rebuilt and raster 195681 gets another id, or answers HTTP 500 | The lock in the request fails or returns other pixels, which fetch-data reports (exit 1); it retries HTTP errors three times |
| The route recipe needs `route_area_3dep.tif` (a 3DEP ImageServer raster, a dynamic mosaic) | It is fetched first in the same run; if the 3DEP service changes, that raster fails first (PR 1's behaviour) and the lidar rebuild says the fill is not here. Its pixels reach no pixel inside the route DEM's bounds today (source map: no 3DEP code) |
| Memory: the route recipe peaks at 2.9 GB (8000 x 6000 float32 and float64 grids) | Fine on the Macs and CI runners (7 GB on macOS, 16 GB on Ubuntu); stated in the README |
| The default `fetch-data` now downloads 371 MB more and takes minutes | Said in the README; `--sources` keeps the downloads; CI caches the result |
| The recovered recipe could hide a detail of 2026-10-06 that the probes did not exercise | Task 11 compares all three files byte for byte before the manifest calls them official (Known 3 already did, with this plan's code) |
