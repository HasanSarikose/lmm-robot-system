# LMM Multi-Robot System

This project implements a multi-robot simulation system in a single Gazebo environment. The system includes a drone, an intelligent ground vehicle (IKA/UGV), a robot arm, obstacles, and red geometric targets. A dashboard sends natural-language mission commands to an LLM layer using Llama 3.1. The backend parses the mission and coordinates the robots.

## System Overview

The main mission scenario is:

1. The user sends a natural-language command from the dashboard.
2. The backend sends the command to Llama 3.1.
3. The LLM returns a structured mission plan.
4. The drone scans the Gazebo environment using its camera.
5. Red geometric targets are detected.
6. The IKA navigates to the detected target locations.
7. The IKA camera performs final visual alignment.
8. The robot arm performs pick/place behavior.
9. The IKA returns to the home position and releases the target.

## Main Components

- **Gazebo / Ignition Gazebo**: Simulation environment.
- **ROS 2 Humble**: Communication layer.
- **ros_gz_bridge**: Bridges Gazebo topics to ROS 2 topics.
- **FastAPI backend**: Mission execution, camera streaming, LLM interface.
- **React dashboard**: User interface and manual control.
- **Llama 3.1 via Ollama**: Natural-language mission planning.
- **OpenCV**: Camera-based red target detection.

## Active Code Structure

```text
backend/
  main.py
  ros_node.py
  frame_buffer.py
  state_buffer.py
  mission_executor.py
  robot_controllers.py
  vision/
    ball_detector.py
    red_ball_detector.py

dashboard/
  React dashboard files

src/system_bringup/launch/
  full_world.sdf
ROS 2 Topics Used

Camera topics:

/drone/camera
/ika/camera
/arm/camera

IKA state topics:

/ika/odom
/ika/lidar

Command topic:

/ugv_cmd_vel
Important Implementation Notes

Earlier versions used ign topic subprocess calls and string parsing for IKA odometry and LiDAR. This has been replaced with a ROS 2 subscriber-based state system:

ros_node.py subscribes to /ika/odom and /ika/lidar.
state_buffer.py stores the latest odometry and LiDAR data.
robot_controllers.py reads IKA state from the shared state buffer.

This makes the system more reliable and closer to a standard ROS 2 architecture.

Requirements

Python packages:

pip install -r requirements.txt

System dependencies include:

sudo apt install ros-humble-ros-gz-bridge

Ollama and Llama 3.1 are required for the LLM layer:

ollama run llama3.1
Running the Demo

Start the full system with:

./start_demo.sh

Or start manually with four terminals.

Terminal 1: Gazebo
export __NV_PRIME_RENDER_OFFLOAD=1
export __GLX_VENDOR_LIBRARY_NAME=nvidia
ign gazebo ~/lmm_robot_ws/src/system_bringup/launch/full_world.sdf -r
Terminal 2: ROS-Gazebo Bridge
source /opt/ros/humble/setup.bash
ros2 run ros_gz_bridge parameter_bridge \
  /drone/camera@sensor_msgs/msg/Image@ignition.msgs.Image \
  /ika/camera@sensor_msgs/msg/Image@ignition.msgs.Image \
  /arm/camera@sensor_msgs/msg/Image@ignition.msgs.Image \
  /ika/lidar@sensor_msgs/msg/LaserScan@ignition.msgs.LaserScan \
  /ika/odom@nav_msgs/msg/Odometry@ignition.msgs.Odometry
Terminal 3: Backend
cd ~/lmm_robot_ws/backend
source /opt/ros/humble/setup.bash
uvicorn main:app --host 0.0.0.0 --port 8000
Terminal 4: Dashboard
cd ~/lmm_robot_ws/dashboard
npm start
Current Limitations
Drone movement currently uses simulated waypoint positioning in Gazebo.
PX4 offboard integration was investigated, and DDS communication was tested, but Gazebo/PX4 sensor plugin issues prevented stable integration in the current demo.
Robot arm pick/place behavior is represented at simulation level.
Quantitative mission evaluation is planned as the next step.
Planned Evaluation Metrics

The system will be evaluated using:

Target detection success rate
Navigation success rate
Pick/place success rate
Mission completion time
Failure modes
Average target localization error