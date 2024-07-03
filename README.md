Cornbot
by: Adam Gronewold at Dartmouth College
email: Adam.M.Gronewold.TH@Dartmouth.edu
last updated: July 3rd, 2024
version: 2.1
This is a ROS2 package built for a modified Pioneer 3-AT robot to navigate through a cornfield
----------------------------
PROJECT DESCRIPTION
This package is the software associated with a modified Pioneer 3-AT robot retrofitted and upgraded to 2024 standards, such that it can navigate through a cornfield. The package is built for ROS2 and can be utilized as a ROS2 package by including it in a standard ROS2 workspace, sourcing the environment and building the package using "colcon build --packages-select cornbot". The package is built as a standalone package that will only work with the Pioneer robot in question. Functionality is a reflection of the engieering and design of the hardware systems on the robot.

HARDWARE ARCHITECTURE
![Hardware Architecture](https://github.com/AdamGronewold/cornbot/assets/83837448/ce40096b-f07d-4a90-a36f-48c07659a3fa)

The robot has a hardware architecture (at the time of this writing) comprised of a central NVidia Jetson Orin nano, from which all ROS2 nodes are activated. The robot is controlled by communicating serially to peripheral microcontrollers (USB, UART, I2C, SDL/SDA, etc.). In general each subsystem of the robot has an associated microcontroller. Wheel speeds are driven by a Teensy 4.1 paired with 2 L298N Dual H bridge motor drivers. This device has Arduino code built for driving embedded PI wheel speed based on received wheel speed reference commands, under "backup_arduino_code/Teensy_Motor_Driver_2024". Similar embedded files exist for other devices like sensors in the same directory. 

EXECUTEABLES
On the ROS2 end, communication occurs through launching a series of ROS2 nodes. In the wheel speed control case, the user will launch "Motor_Control_Function_2024.py" using "ros2 run cornbot motor_control_node2". This makes the subsystem active to ROS2. The node is subscribed to the "cornbot/speed_ref_topic" to watch for wheel speed reference commands arriving from any other ROS2 node. It then sends these reference speeds along to the microcontroller (the Teensy 4.1) for PI control. 

Similar ROS2 side nodes exist for other sensors. The "gps_gnss_function_2024.py" can be launched with "ros2 run cornbot gnss_node2". This code watches the serial line associated with an RTK-fixed RTK Facet piping NMEA data over serial at a rate of 4Hz. It then parses the data and publishes it to a variety of ROS2 topics to be utilized in navigation and control of the robot. There are currently some files in the package associated with an IMU that is not longer on the robot (BNO005, Arduino Nine Axis Motion Shield).

The robot can also be controlled using an Xbox controller with the proper for of Bluetooth (not all Xbox controllers work, see Model 1708). The Xbox controller is currently set up to automatically connect to the robot when turned on in range of the robot's bnluetooth receiver. The user must then launch the motor control node (to prepare to drive) and then launch the xbox controller node using "ros2 run cornbot xbox_teleop_node". If the controller properly connects with the system the terminal window report this and the controller will vibrate for a 5 seconds to let you know it's working. The xbox_teleop_node is currently set up for basic operation. It has two driving modes, a tank steer mode, and a throttle steer mode. In tank steer mode the robot is driven by using the joysticks, left for the left side speed, right for the right side speed. This is the default steer mode when the node is launched. Throttle steer is performed using the right trigger and the left joystick to steer. This driving mode can be toggled by pressing the "Y" button. Other buttons are no longer working at the time of this writing.
