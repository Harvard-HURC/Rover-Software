## What changed

<!-- What this pull request changes and why, in a few sentences or a list. Link the issue it closes ("Closes #12"). -->

## How it was tested

<!-- Say what you ran and what happened; delete the lines that do not apply. -->

- **Simulation:** <!-- the world and the commands, e.g. `pixi run sim urc_delivery`, drove the course from the station; `pixi run test`: OK -->
- **Hardware:** <!-- the rover or computer, the branch's commit, what you ran and saw -->
- **Not tested:** <!-- what, and why -->

## Checklist

- [ ] `pixi run test` passes on my computer (or: the parts I ran, named above)
- [ ] `git status` is clean after the tests: generated files that are tracked (`rover_sim/models/rover/model.sdf`, the station's camera models) are committed with their source
- [ ] A changed rover (`rover.yaml`, the xacro files): the equivalence fixtures are written again and the commit says why (`rover_sim/tests/fixtures/rover_model/README.md`)
- [ ] New numbers carry their unit and source ((A), (M) or (R))
- [ ] Documents updated: the package's README, `docs/topics.md` for a topic, the README's package list for a new package
- [ ] No terrain rasters, build output or other large files committed

<!-- Merging: "Create a merge commit" when the commits are history worth keeping, "Squash and merge" for a small change
     (CONTRIBUTING.md, Branches and pull requests). -->
