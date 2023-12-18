cd ./microros_ws
source install/setup.bash
sudo chmod 777 /dev/ttyUSB0
sudo ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -b 115200
