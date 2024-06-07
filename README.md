# Robot Arm with Object Identification

Welcome to the Robot Arm with Object Identification project! This repository contains the code and resources for controlling a robotic arm to identify and manipulate objects using computer vision.

Demo: https://www.youtube.com/watch?v=N-pMKWJekiM&t=99s&ab_channel=BearBot5000

## Table of Contents

- [Overview](#overview)
- [Features](#features)
- [Requirements](#requirements)
- [Installation](#installation)
- [Usage](#usage)
- [Project Structure](#project-structure)
- [Contributing](#contributing)
- [License](#license)

## Overview

This project combines robotic arm control with object identification using computer vision. The goal is to enable a robotic arm to detect, identify, and interact with various objects within its environment.

## Features

- Control a robotic arm using ROS 2.
- Object identification using OpenCV and machine learning.
- Integration with Gazebo for simulation.
- Real-time feedback and visualization.

## Requirements

- ROS 2 Humble LTS
- Gazebo
- OpenCV
- Python 3.8 or later
- TensorFlow Lite (for running machine learning models)

## Installation

### ROS 2 and Gazebo

Follow the official installation guides for [ROS 2 Humble](https://docs.ros.org/en/humble/Installation.html) and [Gazebo](http://gazebosim.org/tutorials?tut=install_ubuntu&cat=install).

### Python Dependencies

Create a virtual environment and install the necessary Python packages:

python3 -m venv env
source env/bin/activate
pip install -r requirements.txt

### Cloning the Repository
Clone this repository and navigate to its directory:

git clone https://github.com/BearBot5000/Robot_arm_with_object_identification.git
cd Robot_arm_with_object_identification

### Commands

#### Launch the Gazebo simulation:
ros2 launch my_gazebo_worlds world.launch.py

#### Run the object identification node:
ros2 run object_identification identify_objects.py

#### Control the robotic arm:
ros2 run robot_arm_control arm_controller.py

License
This project is licensed under the MIT License. See the LICENSE file for details.
