# HURC Rover Software

Welcome to the central repository for the HURC Rover's control software!

### Setup
We use [pixi](https://pixi.prefix.dev/latest/#installation) to manage ROS packages with [RoboStack](https://robostack.github.io/GettingStarted.html)

1. Install pixi
2. Create a new folder to be your root ROS workspace (ex: `/home/username/hurc_ws/`)
3. Clone this repo into a subfolder called `src` (ex: `/home/username/hurc_ws/src/`)
4. Navigate to the `src` folder and run `pixi install`
5. Run `pixi shell` from this folder anytime you need to interact with installed packages (like ROS)

## Project Structure
Most of our work is structured into folders called "packages". Whenever you create a new package, please add a description of it here:
- `rover_description`: Contains the URDFs that define the rover's structure, joints, actuators, sensors, etc
- `rover_sim`: Contains SDFs, URDFs, launch configurations, and any other tools that help with simulating the rover in Gazebo

## Contributing
Please do not push directly to the main branch. Any time you want to do work locally, please create a new branch with a descriptive name for that feature. To merge your work into the main branch, please create a pull request that can be reviewed by the team. (You can pull from your branch on the rover or base station any time you need to test with real hardware).
