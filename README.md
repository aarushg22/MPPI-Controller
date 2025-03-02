# MPPI Controller in C++ without ROS

This repository contains the code for a TurtleBot3 robot to navigate and avoid obstacles using a Model Predictive Path Integral (MPPI) controller. This is written purelu in C++ without ROS, because I couldn't find any other non-ROS implementation. This has been tested in the Webots simulator, hence the controller code is also commited together.

## Overview

This project implements an MPPI controller for the TurtleBot3 Burger robot in the Webots simulator. The robot is equipped with a lidar sensor to perceive the environment and build a local costmap. The MPPI controller uses this costmap to calculate optimal velocity commands to navigate the environment while avoiding obstacles. The goal of this controller is to go to a defined path.

## Features

*   **MPPI Controller:** Implements a Model Predictive Path Integral controller for motion planning.
*   **Costmap:** Creates a local costmap based on lidar data for obstacle detection.
*   **Obstacle Avoidance:** Generates safe trajectories to avoid obstacles in the environment.
*   **Trajectory Optimization:** Optimizes trajectories considering path following, obstacle avoidance, goal position, and smoothness.
* **Simple Motion model:** The motion model is a simple differential drive model.
*   **Webots Simulation:** Utilizes the Webots simulator for robot simulation.
* **Default control:** A default control using Braitenberg is implemented but commented.


## Dependencies

*   **Webots:** The project requires the Webots robot simulator to be installed. You can download it from the official website: [https://cyberbotics.com/](https://cyberbotics.com/)
*   **CMake:** CMake is used to build the project.
* **C++17:** The code uses C++17.

## Building the Project

1.  **Clone the repository:**
    ```bash
    git clone <repository_url>
    cd turtlebot3_obstacle_avoidance
    ```

2.  **Create a build directory:**
    ```bash
    mkdir build
    cd build
    ```

3.  **Configure the project with CMake:**
    ```bash
    cmake ..
    ```
    Make sure to have set the correct value of `WEBOTS_HOME` in the `CMakeLists.txt` file (line 22). In the file provided, it is set to `/usr/local/webots`.
4.  **Compile the project:**
    ```bash
    make
    ```
The generated binary should be copied out to the parent directory containing the CmakeLists.txt file. This is done because WeBots simulator looks for the controller binary in the parent directory.

### Running the Simulation

1. Open Webots: Launch the Webots simulator.
2. Open a TurtleBot3 world: You'll need to have a Webots world with a TurtleBot3 robot and obstacles. You can create one from scratch or use an existing example world. If you use the example world, then you'll have to copy it out to a place in your filesystem.
3. Set the controller: Go to the controller folder of where you created your new world in the previous step and clone this repo here.
4. Start the simulation: Run the simulation in Webots.


### Future Improvements
1. Dynamic Path Planning: Add the ability for the robot to replan its path dynamically.
2. Global Costmap: Implement a global costmap for more consistent navigation.
3. Better Motion model: Implement a more advanced motion model.
4. More Advanced Cost Functions: Explore more sophisticated cost functions, such as considering the robot's orientation error and distance to the goal.
5. Tune the parameters: Adapt the parameters to have a better behavior.
6. Advanced Obstacle Detection: Implement techniques to classify and handle different types of obstacles.