Cornbot
by: Adam Gronewold at Dartmouth College
email: Adam.M.Gronewold.TH@Dartmouth.edu
last updated: July 3rd, 2024
version: 2.1
This is a ROS2 package built for a modified Pioneer 3-AT robot to navigate through a cornfield
----------------------------
PROJECT DESCRIPTION
This package is the software associated with a modified Pioneer 3-AT robot retrofitted and upgraded to 2024 standards, such that it can navigate through a cornfield. The package is built for ROS2 and can be utilized as a ROS2 package by including it in a standard ROS2 workspace, sourcing the environment and building the package using "colcon build --packages-select cornbot". The package is built as a standalone package that will only work with the Pioneer robot in question. Functionality is a reflection of the engieering and design of the hardware systems on the robot.

HARDWARE
![alt text](https://github.com/[username]/[reponame]/blob/[branch]/image.jpg?raw=true)
The robot has a hardware architecture (at the time of this writing) comprised of a central NVidia Jetson Orin nano, from which all ROS2 nodes are activated. The robot is controlled by communicating serially to peripheral microcontrollers (USB, UART, I2C, SDL/SDA, etc.). In general each subsystem of the robot has an associated microcontroller. Wheel speeds are driven by a Teensy 4.1 paired with 2 L298N Dual H bridge motor drivers. This device has 





EXECUTEABLES







QUIRKS
![Hardware Architecture](https://github.com/AdamGronewold/cornbot/assets/83837448/ce40096b-f07d-4a90-a36f-48c07659a3fa)
