This project simulates a **line-following patrol robot** using **ROS2, Gazebo, RViz2, and OpenCV (cv2)**. The robot is equipped with a camera sensor to detect a predefined path (a colored or black line) and autonomously follow it using computer vision and control algorithms.

## ✨ Features
- 🚗 **Autonomous Navigation**: The robot follows a predefined line using camera vision.
- 🎥 **Camera-based Line Detection**: Utilizes OpenCV (`cv2`) for real-time path detection.
- 🏗 **ROS2 Integration**: Handles robot control and sensor data processing.
- 🌍 **Simulation in Gazebo**: Virtual testing environment with physics simulation.
- 📡 **Visualization in RViz2**: Displays real-time robot state and path tracking.

## 📂 Project Structure
📦 virtual-line-following-robot ├── src/ # Source code for robot control & vision processing ├── launch/ # Launch files for ROS2 ├── models/ # Robot models and world files for Gazebo ├── config/ # Configuration files (camera settings, PID tuning, etc.) ├── scripts/ # Python scripts for image processing and control ├── README.md # Project documentation

## 🚀 Installation & Setup
### 1️⃣ Prerequisites
Ensure you have the following installed:
- [ROS2 (Foxy or later)](https://docs.ros.org/en/foxy/Installation.html)
- [Gazebo](http://gazebosim.org/tutorials?tut=install_ubuntu)
- OpenCV (`cv2`):  
  ```bash
  sudo apt install python3-opencv
ROS2 Dependencies:

sudo apt install ros-foxy-vision-msgs
2️⃣ Clone the Repository
git clone https://github.com/yourusername/virtual-line-following-robot.git
cd virtual-line-following-robot
3️⃣ Build and Source the Package

colcon build
source install/setup.bash

4️⃣ Launch the Simulation
ros2 launch originbot_gazebo originbot_follow_line_gazebo.launch.py
ros2 run rviz2 rviz2
ros2 run originbot_demo line_follower

🎯 How It Works
Camera Captures Image – The robot’s camera captures frames and detects the line.
Image Processing (cv2) – OpenCV processes the frame, detecting the line’s position.
Control Algorithm – The robot adjusts its steering and speed to follow the line.
ROS2 Nodes – The control signals are sent to the robot using ROS2 topics

