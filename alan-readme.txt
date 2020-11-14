Freight-Lite instructions:

Make sure roscore is running.
cd ~/catkin_ws/src
./pc_485net_raw/src/pc_485net_raw.py &
./net_485net_packet_handler/src/485net_packet_sorter.py &
./roboclaw_node/nodes/roboclaw_node.py &
roslaunch freight_lite freight_lite.launch &

-------------------------------------------
Old notes that sometimes come in useful:

wait 20 seconds

roscore &

For testing
./prlite_base/send_vel.sh 0 0 5
./prlite_base/send_vel.sh 0 0 0


Fix firmware (ONLY ONE DEVICE ON CHAIN)
./net_485net_firmware_utils/src/485net_firmware_tool.py 0e
where 0e is the address
Run it until it works (up to 2 times)

old:
#ONLY WORKS IN /prlite directory
./net_485net_id_handler/src/id_server.py &
#ONLY WORKS IN /prlite directory
./prlite_base/bin/base_controller &
old2: (no longer required)
rosrun net_485net_id_handler id_server.py &
new:
source devel_isolated/setup.bash
rosrun net_485net_id_handler id_server.py

