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

## Project Structure
Most of our work is structured into folders called "packages". Whenever you create a new package, please add a description of it here:
- `rover_description`: Contains the URDFs that define the rover's structure, joints, actuators, sensors, etc
- `rover_sim`: Contains SDFs, URDFs, launch configurations, and any other tools that help with simulating the rover in Gazebo

## Contributing
Please do not push directly to the main branch. Any time you want to do work locally, please create a new branch with a descriptive name for that feature. To merge your work into the main branch, please create a pull request that can be reviewed by the team. (You can pull from your branch on the rover or base station any time you need to test with real hardware).
