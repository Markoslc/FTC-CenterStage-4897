# FTC Center Stage - Team 4906 + 4897 2023-2024

This repository contains the code for FTC Team 4897's robot for the Center Stage competition. The code is written in Java and built with Gradle.

## Project Overview

The project aims to provide support for every situation that a robot may encounter during any competition, while maintaining simplicity. The code is designed to be easily understood and modified, allowing for quick adjustments between matches.

## Features

- Orientation: The repository contains code and Parameters for getting the robot's orientation, using the IMU and/or april tags, that are seen by one camera (multiple cameras are not supported yet).
- Color Sensing: The autonomous part of the code provides examples for the use of the camera to detect and compare the average color in a rectangle.
- Telemetry: The robot provides real-time feedback on its status and actions.
- Controller: The repository contains a class for the controller that allows for easy access to the controller's inputs.
- The Constructor of the robot uses a modular design, allowing for easy adjustments, adding, or removing of components.

## Getting Started

To get started with this project, clone the repository and open it in Android Studio or any other IDE that supports Java and Gradle.
A good starting point to look at is the Robot.java file, which contains the main code for the robot. The robot's components are initialized in the constructor, and the main functionality for driving and actions is implemented in the methods below.
It is important to mention that this repository cannot be used as-is, as it is designed for a specific robot. However, it can be used as a reference for creating a new project.
after understanding the robot.java you can start creating your first TeleOp or Autonomous program. 

## Contributing
We welcome contributions from everyone. If you have a suggestion or improvement, please open an issue to discuss it or submit a pull request.

## License
This project is licensed under the MIT License. See the LICENSE file for more details.