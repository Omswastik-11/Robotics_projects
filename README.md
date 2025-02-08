# Husky Autonomous Lap Simulation(PS 1)

## Overview
This simulation is designed to make the Husky robot move inside a track and complete one full lap without crossing lane boundaries. The challenge is to balance speed and accuracy using a **PID controller** while processing the camera feed with **OpenCV** to detect the track boundaries.


## How It Works

1. The **Husky robot** starts from a predefined location inside the track.
2. A **camera feed** is processed using OpenCV to detect the track boundaries.
3. A **PID controller** is used to adjust the robot's movement to stay within the track.
4. The simulation stops after completing one full lap.

## Setting Up the Environment

- The simulation is implemented using the `LaRoboLiga24` gym environment.
- The **URDF files** for the robot and environment are loaded from the `urdf` folder.
- The **camera settings** are configured to provide an optimal top-down view for track detection.

## PID Controller

The **PID (Proportional-Integral-Derivative) controller** helps keep the Husky on track by adjusting its wheel velocities based on the detected error.

- **Proportional Term (P)**: Adjusts based on the current error (how far the robot is from the center).
- **Integral Term (I)**: Corrects past accumulated errors.
- **Derivative Term (D)**: Predicts future errors to avoid overshooting.

The PID parameters used are:

```python
kp = 0.037   # Proportional gain
ki = 0.000095  # Integral gain
kd = 0.000034  # Derivative gain
```

## Simulation Steps

1. Start the simulation and **press 's'** immediately to turn off the sun.
2. The robot's camera captures the track image.
3. The image is processed using OpenCV to detect lane boundaries.
4. The PID controller calculates the required speed adjustments.
5. The robot moves forward while staying within the lane.
6. Once a full lap is completed, the robot stops.
7. The simulation can be exited by pressing 'q'.

## Running the Simulation

Run the following Python script:

```bash
python PS1_soln.py.py
```

## Visual Debugging
The script includes OpenCV windows to visualize the processed track image:
- **Thresholded track image**: Displays the detected track boundaries.
- **Raw camera feed**: Shows the robot’s current view.

## Stopping the Simulation
- The simulation stops automatically after **one full lap**.
- You can manually stop it by pressing **'q'** in the OpenCV window.

## Future Improvements
- **Fine-tuning PID parameters** for smoother movement.
- **Adding obstacle detection** using depth cameras.
- **Enhancing lane detection** with more robust image processing.

# RoboLeague Challenge - Husky Ball Scoring(PS2)

## Overview
The RoboLeague challenge involves controlling a Husky robot to pick up balls and shoot them into goalposts while avoiding a goalkeeper. The goal is to develop an autonomous strategy for efficient navigation, ball collection, and scoring.

## Objectives
- Detect and navigate towards balls.
- Pick up the ball using the robotic gripper.
- Identify goalposts and align for an accurate shot.
- Avoid obstacles, including a moving goalkeeper.
- Optimize the scoring process for efficiency.

## Environment
- **Robot:** Husky (UGV with a robotic gripper)
- **Sensors:** Camera (for ball and goal detection), Lidar (for obstacle detection)
- **Arena:** Multiple balls and two goalposts
- **Opponent:** A  goalkeeper trying to block shots

## Implementation Details
- **Ball Detection:** Uses computer vision techniques to locate balls in the environment.
- **Goalpost Detection:** Identifies goalposts and determines the best shooting angle.
- **Navigation:** Uses path planning algorithms to move towards the ball and goal.
- **Gripping & Shooting:** Controls the robotic arm to pick up and shoot balls.
- **Avoidance Strategy:** Detects obstacles and dynamically adjusts path to avoid collisions.

## Technologies Used
- **ROS (Robot Operating System)** for robot control
- **OpenCV** for computer vision
- **Gazebo** for simulation
- **Python** for scripting robotic behaviors
- **MoveIt!** for robotic arm control


## Future Improvements
- Enhance ball detection using deep learning.
- Implement a reinforcement learning-based strategy.
- Improve obstacle avoidance with advanced path planning.



