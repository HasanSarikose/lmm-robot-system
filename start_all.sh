
#!/bin/bash
echo "🤖 Multi-Robot Sistemi Başlatılıyor..."

killall -9 gz sim ruby parameter_bridge px4 2>/dev/null
find $HOME/PX4-Autopilot -name "parameters.bson" -delete 2>/dev/null
find $HOME/PX4-Autopilot -name "parameters_backup.bson" -delete 2>/dev/null
sleep 2

# PX4'ü screen oturumunda başlat
cd $HOME/PX4-Autopilot
export IGN_GAZEBO_RESOURCE_PATH=$HOME/PX4-Autopilot/Tools/simulation/gz/models
export GZ_SIM_RESOURCE_PATH=$HOME/PX4-Autopilot/Tools/simulation/gz/models
screen -dmS px4 bash -c "make px4_sitl gz_x500; exec bash"

echo "PX4 başlatıldı. Gazebo bekleniyor..."
for i in $(seq 1 60); do
    if gz topic -l 2>/dev/null | grep -q "/world/default/clock"; then
        echo "Gazebo hazır!"
        break
    fi
    sleep 2
done
sleep 10

echo "TurtleBot3 ekleniyor..."
gz service -s /world/default/create --reqtype gz.msgs.EntityFactory --reptype gz.msgs.Boolean --timeout 5000 --req "sdf_filename: \"$HOME/lmm_robot_ws/src/system_bringup/launch/turtlebot3_gz8.sdf\""

echo "Robot Kol ekleniyor..."
gz service -s /world/default/create --reqtype gz.msgs.EntityFactory --reptype gz.msgs.Boolean --timeout 5000 --req "sdf_filename: \"$HOME/lmm_robot_ws/src/system_bringup/launch/robot_arm_gz8.sdf\""

echo ""
echo "✅ Tüm robotlar hazır!"
echo ""
echo "PX4 konsoluna bağlanmak için: screen -r px4"
echo "  Orada yaz: commander arm -f"
echo "  Sonra:     commander takeoff"
echo "  Çıkmak için: Ctrl+A sonra D"
echo ""
echo "Test komutları:"
echo "  TurtleBot:  gz topic -t /ugv_cmd_vel -m gz.msgs.Twist -p 'linear: {x: 0.2}'"
echo "  Robot Kol:  gz topic -t /arm_j2 -m gz.msgs.Double -p 'data: 0.5'"
