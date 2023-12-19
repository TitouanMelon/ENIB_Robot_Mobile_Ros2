#sed -i 's/\r$//' ./ROS.sh && ./ROS.sh
######### CONFIG ############################
BASE_DIR=$(pwd)'/' #DO NOT CHANGE THIS
ROS_DISTRO=humble #DO NOT CHANGE THIS
DEST_DIR=~/robot/ #CHANGE IF YOU WANT CHANGE THE PROJECT DIRECTORY

######### CREATE PROJECT DIRECTORY ##########
mkdir -p $DEST_DIR
if [ $? -ne 0 ]; then
    echo "Can't create "$DEST_DIR" folder. Abort installation"
	exit
fi
######### STM32.sh ##########################
echo 'cd $DEST_DIR' >> STM32.sh
echo 'DIR=$BASH_SOURCE' >> STM32.sh
echo 'if [ -L $DIR ]; then' >> STM32.sh
echo '  DIR=$(dirname $(readlink $DIR))' >> STM32.sh
echo 'else' >> STM32.sh
echo '  DIR=$(pwd)' >> STM32.sh
echo 'fi' >> STM32.sh
echo 'cd $DIR/microros_ws' >> STM32.sh
echo 'source install/setup.bash' >> STM32.sh
echo 'sudo chmod 777 /dev/ttyUSB0' >> STM32.sh
echo 'ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -b 115200' >> STM32.sh

######## SET_ROS_DOMAIN_ID.sh ###############
cd $DEST_DIR
echo 'if [ $# -eq 0 ]; then' >> SET_ROS_DOMAIN_ID.sh
echo '	echo usage $0 <new_id>' >> SET_ROS_DOMAIN_ID.sh
echo "	exit" >> SET_ROS_DOMAIN_ID.sh
echo 'fi' >> SET_ROS_DOMAIN_ID.sh
echo 'sudo sed -i "s/ROS_DOMAIN_ID=.*/ROS_DOMAIN_ID=$1/g" /etc/environment' >> SET_ROS_DOMAIN_ID.sh

echo "while true; do" >> SET_ROS_DOMAIN_ID.sh
echo "	read -p 'You need to reboot to apply change. Do you want reboot now ? [y/N]' yn" >> SET_ROS_DOMAIN_ID.sh
echo '	case $yn in' >> SET_ROS_DOMAIN_ID.sh
echo "		[Yy]* ) sudo reboot; break;;" >> SET_ROS_DOMAIN_ID.sh
echo "		[Nn]* ) exit;;" >> SET_ROS_DOMAIN_ID.sh
echo "		* ) exit;;" >> SET_ROS_DOMAIN_ID.sh
echo " esac" >> SET_ROS_DOMAIN_ID.sh
echo "done" >> SET_ROS_DOMAIN_ID.sh

echo "ROS_DOMAIN_ID=0" | sudo tee -a /etc/environment
cd $DEST_DIR
chmod +x ./*.sh
sudo ln -s $DEST_DIR/STM32.sh /usr/bin/stm32_ros_agent 
sudo ln -s $DEST_DIR/SET_ROS_DOMAIN_ID.sh /usr/bin/set_ros_id

###################### STM32 ##################################################
sudo apt install wget -y
cd $DEST_DIR
wget https://www.enib.fr/~kerhoas/ROBOT_ROS_/WORKSPACE_F411_uROS6.zip
unzip WORKSPACE_F411_uROS6.zip

sudo apt install gtkterm -y

###################### INSTALL ROS #############################################
sudo apt install software-properties-common -y
sudo add-apt-repository universe -y

sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
sudo apt upgrade -y
sudo apt install ros-humble-desktop -y
sudo apt install ros-humble-ros-base -y
sudo apt install ros-dev-tools -y

# Replace ".bash" with your shell if you're not using bash
# Possible values are: setup.bash, setup.sh, setup.zsh
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

###################### INSTALL MICRO_ROS #######################################
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

######################## Install qtcreator + qtcreator-ros #######################
sudo apt install qtcreator -y
sudo snap install qtcreator-ros --classic

#lauch ihm -> build
#apres build allez dans les propriete du projet pour indiquer l'emplacement de l'executable créé

###################### STM32 CUBE IDE ###########################################
cd $BASE_DIR
echo 'Installation de STM32 cube IDE en cours, veuillez patientez ...'
sudo ./st-stm32cubeide_1.14.0_19471_20231121_1200_amd64.deb_bundle.sh --quiet && echo 'Installation de STM32 cube IDE finie'
