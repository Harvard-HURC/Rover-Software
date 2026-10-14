# Research

The measurements, prototypes and gate runs behind the simulation's design
(`docs/superpowers/specs/2026-10-06-urc-realism-design.md`), moved here from
`sim/data/research/` when the simulation joined this repository
(2026-10-09). The simulation reads none of it: the four research files its
code reads or writes stay in `rover_sim/data/research/`
(`terrain_targets.json`, `mdrs_terrain_measurements.json`,
`realism_report.json`, `realism_contact_sheet.jpg`).

- `gates.json`, `gates/`: the WS-0 gate runs G1 to G8 (the worlds' physics
  speed per resolution and solver, meshes and textures, the drivetrain
  prototype and its cost, the station's frame rate) and their scripts
- `drive/`: the drivetrain prototype (motor, contact and dig-in models) and
  its experiments
- `flycam/`: the fly camera prototype and its measurements
- `render/`: the rendering prototype, its steps and before/after pictures
- the rest: terrain, colour and micro-relief measurements of the MDRS and
  route areas (`lidar_microrelief.json`, `colour_stats.json`,
  `strata_colour_ramps.json`, the soil survey, close-ups and overviews)

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

Code comments cite these files as `sim/data/research/<file>`: read
`docs/research/<file>`. The gate and prototype scripts expect the tree as it
was before the move (they look for `sim/` beside them); run them from that
tree, the parent of the commit that moved it:
`git checkout $(git rev-list -1 HEAD -- sim/gen_model.py)^`.
