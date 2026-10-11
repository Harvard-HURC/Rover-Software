# PR 4: Contributor documents Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** A student who has never seen the repository can set it up on a Mac, on Linux or on Windows (the rover code natively, the simulation in WSL2), change the rover, add a world or a sensor, swap or write a drive controller, run the tests and open a pull request the team can review, from `CONTRIBUTING.md`, the `.github` templates and the package READMEs.

**Architecture:** Design spec `docs/superpowers/specs/2026-10-09-rover-software-integration-design.md`, section 8 (Documents) and 9.2 row 4. Documents only, on the tree PR 3 left: no code, configuration or test changes. Every command and path the documents name is run or checked on `pr4/contributing`; the steps that cannot be run here (Windows, WSL2, Ubuntu, the rover's computer) follow Microsoft's, RoboStack's, pixi's and Gazebo's published guidance and say in the document that no team machine has run them yet.

**Tech Stack:** Markdown (GitHub-flavoured), GitHub's CODEOWNERS, pull request and issue templates; pixi 0.81, ROS 2 Jazzy (RoboStack), colcon, Gazebo Harmonic, for the commands the documents name.

---

## Places and rules

| Name | Path |
|---|---|
| SRC | `/Users/alarion239/Desktop/hurc_ws/src` (the team's clone; branch `pr4/contributing`, made from `pr3/ros2-control` at `16b1523`) |
| WS | `/Users/alarion239/Desktop/hurc_ws` (the colcon workspace) |
| T | `<scratchpad>/pr4` (the session's scratchpad: the example program, logs; deleted at the end) |
| P | `/Users/alarion239/.pixi/bin/pixi` (0.81.0, the version CI installs) |

- Never push, never change anything on GitHub. No worktree or clone inside WS. `/Users/alarion239/Desktop/Rover` is read-only.
- Every ROS or Gazebo run sets `GZ_IP=127.0.0.1`, its own `GZ_PARTITION` and `ROS_DOMAIN_ID` (71 to 74 here) and `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`; stop only the processes this work starts.
- Commits start `PR 4: ` and end with `Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>`.
- Style: the repository's: short declarative sentences, units in brackets, numbers with their source label ((A) assumed, (M) measured or tuned, (R) from a reference), dated user decisions.

## Decisions this plan makes

1. **The WSL2 guide is a section of `CONTRIBUTING.md`** (spec 8: "setup per platform including WSL2"), not a file of its own; the README's Windows line links to it.
2. **The Autonomy lidar caveat stays in one place**, the README's Simulation section, which `CONTRIBUTING.md` and `rover_sim/README.md` link to. The data pull request (`data/lidar-from-usgs`, fetch-data rebuilding that raster) then changes one paragraph, not three.
3. **What no one ran is marked where it is written.** No Windows or Linux machine was available (2026-10-10): `CONTRIBUTING.md` says that the Windows and WSL2 steps follow the vendors' guidance and that no team machine has run them, and that Linux is the macOS steps run by CI's Ubuntu job, whose first run is the first push. Whoever goes through them first fixes what differs.
4. **CODEOWNERS** (the task's names; the team confirms or changes them, as the file's comment says): `*` (the root files and everything not named below: `.github/`, `docs/` outside `docs/sim/`) and `/rover_description/`, `/rover_control/` to @BeckettOBrien; `/rover_sim/` and `/docs/sim/` to @Alarion239. GitHub takes the last matching line, so the specific lines follow `*`. Owners only get review requests until an admin turns on "Require review from Code Owners" (spec 8: the user's account has no admin rights).
5. **Merging**: a merge commit for a pull request that carries history (the integration series PR 1 to PR 4, the data pull request, any branch whose commits each say what changed); squash allowed for a small one whose commits are work in progress; never rebase-merge a history branch, because the documents cite commits by hash (`rover_sim/tests/fixtures/rover_model/README.md` cites `9208047`).
6. **Example code lives in the documents only** (the task: no code changes). `CONTRIBUTING.md` carries a 20-line rclpy program that drives the rover through `/cmd_vel`; it was run against `rover.launch.py` with the hardware stub, whose log showed the wheel commands it would send (2.00 rad/s for 0.3 m/s). A ros2_control controller gets the steps, the interface names and the reference implementation (`diff_drive_controller`), no C++ skeleton, which could not be checked without building a plugin.
7. **Issue templates are Markdown templates** (`bug_report.md`, `feature_request.md`, with the labels `bug` and `enhancement`, which GitHub creates in every new repository), not YAML forms: they render the same in every client and need no schema.
8. **Commit messages** follow the history: a subject that says what changed, the area first; a body with why and how it was tested; a `Co-Authored-By:` trailer when an assistant wrote part of it.

## File structure

Created: `CONTRIBUTING.md`; `.github/CODEOWNERS`; `.github/pull_request_template.md`; `.github/ISSUE_TEMPLATE/bug_report.md`, `.github/ISSUE_TEMPLATE/feature_request.md`; this plan.

Modified: `README.md` (Setup and Contributing point at `CONTRIBUTING.md`, the Windows line at its WSL2 guide, the package list links each package's README), `rover_description/README.md`, `rover_control/README.md`, `rover_sim/README.md` (each: its tests, how to work on it, a link back to `CONTRIBUTING.md`).

### Task 1: The plan

- [ ] **Step 1:** Create the branch: `cd SRC && git switch -c pr4/contributing pr3/ros2-control`.
- [ ] **Step 2:** Write this file; commit `PR 4: plan`.

### Task 2: CONTRIBUTING.md

**Files:** Create `CONTRIBUTING.md`.

- [ ] **Step 1:** Write the sections: before you start (the packages, the dependency rule); setup (what every platform needs; macOS; Linux; Windows natively; Windows with the simulation in WSL2, step by step, with the GPU notes and a troubleshooting list; the rover's computer); building and running; the workflow (branch, pull request, review, CI, merging, stacked pull requests, testing on the rover); how to change the rover, add a world, add a sensor, swap the drive controller, write a program that drives the rover, write a drive controller, add a package, fetch the terrain data, run the tests and the smoke test; code style; commit messages.
- [ ] **Step 2:** Run every command it names that runs on this Mac (Verification, below), and check every path (Task 4).
- [ ] **Step 3:** Commit `PR 4: CONTRIBUTING.md`.

### Task 3: The GitHub templates

**Files:** Create `.github/CODEOWNERS`, `.github/pull_request_template.md`, `.github/ISSUE_TEMPLATE/bug_report.md`, `.github/ISSUE_TEMPLATE/feature_request.md`.

- [ ] **Step 1:** CODEOWNERS as Decision 4, with the comment. The pull request template: what changed and why, how it was tested (simulation: which world and tasks; hardware: which rover or computer; not tested, and why), a checklist (tests, generated files committed, documents, `rover.yaml` sources, no rasters). The issue templates: what happened, expected, steps, where (platform, branch and commit, simulation or hardware, world), logs; the problem, the proposal, the package, how it would be tested.
- [ ] **Step 2:** Commit `PR 4: .github: CODEOWNERS, the pull request template, issue templates`.

### Task 4: The package READMEs and the README

**Files:** Modify `README.md`, `rover_description/README.md`, `rover_control/README.md`, `rover_sim/README.md`.

- [ ] **Step 1:** Each package README says how to run its tests and links `CONTRIBUTING.md`; `rover_sim/README.md` adds a world, runs one test module and the smoke test; `rover_control/README.md` builds the rover's two packages on the rover's computer; the README's package list links each README, its Contributing section and Setup point at `CONTRIBUTING.md`, its Windows line at the WSL2 guide.
- [ ] **Step 2:** Check every relative link and every repository path in the changed Markdown files exists (a script in T: each `](path)` link and each backticked path that starts with a top-level folder, resolved against the file's folder or SRC).
- [ ] **Step 3:** `git diff --stat pr3/ros2-control` lists only Markdown files and `.github/CODEOWNERS`. Commit `PR 4: the package READMEs and the README: tests, links, CONTRIBUTING.md`.

## Verification

Filled in as the work ran (Task 2, Step 2; Task 4, Steps 2 and 3).
