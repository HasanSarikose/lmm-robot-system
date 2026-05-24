#!/bin/bash

echo "======================================"
echo " LMM Multi-Robot Demo Starting"
echo "======================================"

# Eski prosesleri temizle
pkill -f "uvicorn" 2>/dev/null
pkill -f "npm start" 2>/dev/null
killall -9 ign gazebo gz sim ruby parameter_bridge 2>/dev/null

sleep 2

echo "[1/5] Starting Gazebo GUI world in paused mode..."
gnome-terminal -- bash -c "
cd ~/lmm_robot_ws

# NVIDIA offload kapalı. Bu sistemde GUI bu şekilde daha stabil çalışıyor.
unset __NV_PRIME_RENDER_OFFLOAD
unset __GLX_VENDOR_LIBRARY_NAME

# DİKKAT:
# -r kullanmıyoruz.
# Çünkü DetachableJoint hedefleri başlangıçta attached başlatabiliyor.
# Önce detach gönderilecek, sonra world unpause edilecek.
ign gazebo ~/lmm_robot_ws/src/system_bringup/launch/full_world.sdf

exec bash
"

# Gazebo GUI ve world pluginlerinin açılması için bekle
sleep 10

echo "[2/5] Detaching targets and starting simulation..."
gnome-terminal -- bash -c "
echo '[INIT] Detaching all grasp targets...'

# DetachableJoint başlangıçta attached gelebilir.
# Bu yüzden birkaç kez detach gönderiyoruz.
for i in 1 2 3
do
  echo \"[INIT] Detach attempt \$i...\"

  ign topic -t '/red_circle/detach' -m ignition.msgs.Empty -p 'unused: true'
  sleep 0.3

  ign topic -t '/red_square/detach' -m ignition.msgs.Empty -p 'unused: true'
  sleep 0.3

  ign topic -t '/red_triangle/detach' -m ignition.msgs.Empty -p 'unused: true'
  sleep 0.5
done

echo '[INIT] Detach commands sent.'

echo '[INIT] Starting simulation...'
ign service -s /world/lmm_world/control \
  --reqtype ignition.msgs.WorldControl \
  --reptype ignition.msgs.Boolean \
  --timeout 1000 \
  --req 'pause: false'

echo '[INIT] Simulation unpaused.'

exec bash
"

sleep 3

echo "[3/5] Starting ROS-Gazebo bridge..."
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

echo "[4/5] Starting backend..."
gnome-terminal -- bash -c "
cd ~/lmm_robot_ws/backend
source /opt/ros/humble/setup.bash
uvicorn main:app --host 0.0.0.0 --port 8000
exec bash
"

sleep 3

echo "[5/5] Starting dashboard..."
gnome-terminal -- bash -c "
cd ~/lmm_robot_ws/dashboard
npm start
exec bash
"

echo "======================================"
echo " Demo started."
echo " Gazebo:    GUI mode, paused start, detach, then unpause"
echo " Backend:   http://localhost:8000"
echo " Dashboard: http://localhost:3000"
echo "======================================"
echo ""
echo "Manual checks:"
echo "  curl http://localhost:8000/status"
echo "  ign topic -l | grep -E 'red_circle|red_square|red_triangle|state'"
echo ""
echo "Manual detach if needed:"
echo "  ign topic -t '/red_circle/detach' -m ignition.msgs.Empty -p 'unused: true'"
echo "  ign topic -t '/red_square/detach' -m ignition.msgs.Empty -p 'unused: true'"
echo "  ign topic -t '/red_triangle/detach' -m ignition.msgs.Empty -p 'unused: true'"
echo "======================================"