#!/bin/bash

echo "======================================"
echo " LMM Multi-Robot Demo Starting"
echo "======================================"

pkill -f "uvicorn" 2>/dev/null
killall -9 ign gazebo gz sim ruby parameter_bridge 2>/dev/null

sleep 2

echo "[1/4] Starting Gazebo world..."
gnome-terminal -- bash -c "
export __NV_PRIME_RENDER_OFFLOAD=1
export __GLX_VENDOR_LIBRARY_NAME=nvidia
ign gazebo ~/lmm_robot_ws/src/system_bringup/launch/full_world.sdf -r
exec bash
"

sleep 6

echo "[2/4] Starting ROS-Gazebo bridge..."
gnome-terminal -- bash -c "
source /opt/ros/humble/setup.bash
ros2 run ros_gz_bridge parameter_bridge \
  /drone/camera@sensor_msgs/msg/Image@ignition.msgs.Image \
  /ika/camera@sensor_msgs/msg/Image@ignition.msgs.Image \
  /arm/camera@sensor_msgs/msg/Image@ignition.msgs.Image \
  /ika/lidar@sensor_msgs/msg/LaserScan@ignition.msgs.LaserScan \
  /ika/odom@nav_msgs/msg/Odometry@ignition.msgs.Odometry
exec bash
"

sleep 4

echo "[3/4] Starting backend..."
gnome-terminal -- bash -c "
cd ~/lmm_robot_ws/backend
source /opt/ros/humble/setup.bash
uvicorn main:app --host 0.0.0.0 --port 8000
exec bash
"

sleep 3

echo "[4/4] Starting dashboard..."
gnome-terminal -- bash -c "
cd ~/lmm_robot_ws/dashboard
npm start
exec bash
"

echo "======================================"
echo " Demo started."
echo " Backend:   http://localhost:8000"
echo " Dashboard: http://localhost:3000"
echo "======================================"