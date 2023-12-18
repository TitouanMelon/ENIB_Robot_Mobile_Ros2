DIR=$BASH_SOURCE

if [ -L $DIR ]; then
  DIR=$(dirname $(readlink $DIR))
else
  DIR=$(pwd)
fi

cd $DIR/microros_ws
source install/setup.bash
sudo chmod 777 /dev/ttyUSB0
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -b 115200
