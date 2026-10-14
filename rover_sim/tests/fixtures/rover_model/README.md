# The rover as gen_model.py wrote it

These SDF files are the rover model and the variants of it that the tests build, as `rover_sim/gen_model.py` wrote them at commit 9208047 (`import/simulation`, 2026-10-09), before `rover_description` described the rover. `tests/test_equivalence.py` builds the same rovers from `rover_description/config/rover.yaml`, `rover_sim/config/sim.yaml` and `urdf/rover_sim.urdf.xacro` (`tools/gen_rover.py`) and checks that each is its file here, number for number (`tests/sdf_equivalence.py`), and that the default and two variants drive bit for bit alike in Gazebo.

| File | gen_model.py's parameters | The same rover now (`rover_config.Variant().replace(...)`) |
|---|---|---|
| `default.sdf` | `Params()`; byte for byte the tracked `models/rover/model.sdf` of 9208047 | `Variant()` |
| `diffdrive.sdf` | `Params(drive=DriveParams(mode="diffdrive"))` | `drivetrain={"mode": "diffdrive"}` |
| `diffdrive_square.sdf` | `Params(drive=DriveParams(mode="diffdrive"), mu_lateral=1.0)` | `drivetrain={"mode": "diffdrive"}, diffdrive={"mu_lateral": 1.0}` |
| `diffdrive_dust.sdf` | `Params(drive=DriveParams(mode="diffdrive", dust=True))` | `drivetrain={"mode": "diffdrive"}, dust={"enabled": True}` |
| `dust.sdf` | `Params(drive=DriveParams(dust=True))` | `dust={"enabled": True}` |
| `cues_off.sdf` | `Params(tread_tyre=False, drive=DriveParams(dig_sink=False, ruts=False))` | `tread_tyre=False, dig_sink={"enabled": False}, ruts={"enabled": False}` |
| `tyre_compliance.sdf` | `Params(tire_compliance=True)` | `tyre_compliance={"enabled": True}` |
| `test_physical.sdf` | `Params(drive=DriveParams(cmd_timeout_clock="sim", dig=False, mu_noise=0.0, backlash=0.0, current_limit=10.0, accel=0.0, state_rate=1000.0, cmd_timeout=0.0, default_surface="sand", tracks=TrackParams(report=1.0)))`: every number the tests change | `drivetrain={"cmd_timeout_clock": "sim", "accel": 0.0, "state_rate": 1000.0, "cmd_timeout": 0.0}, contact={"dig": False, "mu_noise": 0.0, "default_surface": "sand"}, driveline={"backlash": 0.0}, motor={"current_limit": 10.0}, ruts={"tracks": {"report": 1.0}}` |
| `same_surfaces.sdf` | `Params(drive=DriveParams(default_surface="manmade"))`: one surface row | `contact={"default_surface": "manmade"}` |
| `unknown_surface.sdf` | `Params(drive=DriveParams(default_surface="lunar_dust"))`: no row for a key the catalogue lacks | `contact={"default_surface": "lunar_dust"}` |

When the rover changes on purpose (real numbers from the mechanical team, a new sensor), the test fails as it should. Then write the fixtures again from the new generator in the same commit, and say in the commit why the rover changed:

```bash
cd rover_sim/tests && pixi run python -c "
import worldfiles, gen_rover, test_equivalence as t
for name, variant in t.VARIANTS.items():
    (t.FIXTURES / f'{name}.sdf').write_text(gen_rover.build_sdf(variant))"
```

The same commit updates what says where the fixtures come from: this README's heading, first paragraph (the generator and the commit) and table, and the docstrings that say gen_model.py wrote them (`tests/test_equivalence.py`, `tests/sdf_equivalence.py`, `tests/test_rover_model.py`, `tools/gen_rover.py`; `git grep gen_model rover_sim/tests rover_sim/tools` finds them).
