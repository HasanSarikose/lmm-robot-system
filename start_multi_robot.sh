#!/bin/bash
echo "🤖 Multi-Robot Sistemi Başlatılıyor..."

# 1) Temizle
find $HOME/PX4-Autopilot -name "parameters.bson" -delete 2>/dev/null
find $HOME/PX4-Autopilot -name "parameters_backup.bson" -delete 2>/dev/null
killall -9 gz sim ruby parameter_bridge px4 2>/dev/null
sleep 2

# 2) PX4 başlat — logları dosyaya yönlendir
echo "[1/4] PX4 + Gazebo başlatılıyor..."
cd $HOME/PX4-Autopilot
export IGN_GAZEBO_RESOURCE_PATH=$HOME/PX4-Autopilot/Tools/simulation/gz/models
export GZ_SIM_RESOURCE_PATH=$HOME/PX4-Autopilot/Tools/simulation/gz/models
make px4_sitl gz_x500 > /tmp/px4_log.txt 2>&1 &

# 3) Gazebo hazır olana kadar bekle
echo "[2/4] Gazebo bekleniyor..."
for i in $(seq 1 60); do
    if gz topic -l 2>/dev/null | grep -q "/world/default/clock"; then
        echo "  Gazebo hazır!"
        break
    fi
    sleep 2
done
sleep 10

# 4) Robotları ekle
echo "[3/4] TurtleBot3 ekleniyor..."
gz service -s /world/default/create --reqtype gz.msgs.EntityFactory --reptype gz.msgs.Boolean --timeout 5000 --req "sdf_filename: \"$HOME/lmm_robot_ws/src/system_bringup/launch/turtlebot3_gz8.sdf\""

echo "[4/4] Robot Kol ekleniyor..."
gz service -s /world/default/create --reqtype gz.msgs.EntityFactory --reptype gz.msgs.Boolean --timeout 5000 --req "sdf_filename: \"$HOME/lmm_robot_ws/src/system_bringup/launch/robot_arm_gz8.sdf\""

# 5) Drone'u otomatik kaldır
echo "Drone arm + takeoff bekleniyor..."
sleep 5
gz topic -t /world/default/model/x500_0/link/base_link/sensor/imu_sensor/imu -e --num 1 > /dev/null 2>&1
cd $HOME/PX4-Autopilot/build/px4_sitl_default
echo "commander arm -f" | ./bin/px4 -d 2>/dev/null &
sleep 3

echo ""
echo "✅ Tüm robotlar hazır!"
echo "PX4 logları: tail -f /tmp/px4_log.txt"
echo ""
echo "Drone arm/takeoff için ayrı terminal aç:"
echo "  cd ~/PX4-Autopilot/build/px4_sitl_default"
echo "  pxh komutu yaz: (PX4 loglarına bak)"
echo ""
echo "Test komutları:"
echo "  TurtleBot:  gz topic -t /ugv_cmd_vel -m gz.msgs.Twist -p 'linear: {x: 0.2}'"
echo "  Robot Kol:  gz topic -t /arm_j2 -m gz.msgs.Double -p 'data: 0.5'"
echo ""
echo "PX4 konsoluna bağlanmak için: screen -r px4 (veya tail -f /tmp/px4_log.txt)"
