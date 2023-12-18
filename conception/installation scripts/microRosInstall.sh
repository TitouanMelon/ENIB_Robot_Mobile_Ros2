# Source the ROS 2 installation
source /opt/ros/$ROS_DISTRO/setup.bash

# Create a workspace and download the micro-ROS tools
cd $DEST_DIR
mkdir microros_ws
cd microros_ws
sudo apt install git -y
git clone -b $ROS_DISTRO https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup

# Update dependencies using rosdep
sudo apt update && sudo rosdep init && rosdep update
rosdep install --from-paths src --ignore-src -y

# Install pip
sudo apt-get install python3-pip -y

# Build micro-ROS tools and source them
colcon build
source /opt/ros/$ROS_DISTRO/setup.bash
source install/local_setup.bash
# Create the MICRO_ROS agent
# Download micro-ROS-Agent packages
ros2 run micro_ros_setup create_agent_ws.sh
# Build step
ros2 run micro_ros_setup build_agent.sh
