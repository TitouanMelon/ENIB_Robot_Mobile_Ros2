# ENIB_Robot_Mobile_Ros2

Project to make a moving robot with communication and sensors

## Project member
- Titouan Melon : https://www.linkedin.com/in/titouan-melon
- Louanne Floch : https://www.linkedin.com/in/louanne-floch
- Jérémy Plantec : https://www.linkedin.com/in/j%C3%A9r%C3%A9my-plantec-7a24a81bb
- Donald TOGNIA DJANKO DIALLO : https://www.linkedin.com/in/tognia-djanko-diallo-donald-b38281228

## Google drive's link
- https://drive.google.com/drive/folders/1ekVw0Pp5evgDc79cSzruJdU0V-NTOiN-?usp=sharing

## Schema
![robot sheme](./conception/img/robotScheme.png)

## Folder description

```
\---Camera : camera's program 
\---IHM : qt ihm
\---WORKSPACE_F411_uROS6 : stm32 workspace
\---conception
|   \---3D_models
|   \---img : img to readme
|   \---installation scripts
|   \---schematique : eagle
\---docs
|   \---externe : extern pdf 
|   \---html : documentation in web format
|   \---latex : documentation in pdf format
\---report : report for ERI
```

# Software installation procedure
## Install OS

### On a PC
Install <a href="https://ubuntu.com/download/desktop" >Ubuntu 22.04.3 LTS</a> on a pc
### On a RPI
Install <a href="https://www.raspberrypi.com/software/">RPI imager</a> and with this install Ubuntu like this

CHOOSE OS -> Other general-purpose OS -> Ubuntu -> Ubuntu Desktop 22.04.03 LTS (64bits)

## Install all in one script
>[!NOTE]
>this section is recommended for a pc but it is possible to apply it to the raspberry PI
>You need 8Go of free space

You can install all necessary tools with this following commands.

TODO - add the possibility to just install ros et µros by specified rpi in parameter of script to avoid download all things

```
sudo apt install wget -y
wget https://raw.githubusercontent.com/TitouanMelon/ENIB_Robot_Mobile_Ros2/main/conception/installation%20scripts/ROS.sh
wget STM32cubeIDE.sh
sed -i 's/\r$//' ./ROS.sh && chmod +x ./ROS.sh && ./ROS.sh
```

## For PC and RaspberryPi
### Install ROS2
ROS2 installation is the same on PC and on RPI

```
sudo apt install wget -y
wget https://raw.githubusercontent.com/TitouanMelon/ENIB_Robot_Mobile_Ros2/main/conception/installation%20scripts/ROS_Install.sh
sed -i 's/\r$//' ./ROS_Install.sh && chmod +x ./ROS_Install.sh && ./ROS_Install.sh
```

### Install MicroRos

>[!WARNING]
>Be carreful the installation of microRos require a minimum of 1GB of RAM to succes

>[!NOTE]
>In order to install on a RPI4 with 2GB of RAM you can remove gdm3 install the script with the console and reinstall gdm3

```
sudo apt install wget -y
wget https://raw.githubusercontent.com/TitouanMelon/ENIB_Robot_Mobile_Ros2/main/conception/installation%20scripts/MicroROS_Install.sh
sed -i 's/\r$//' ./MicroROS_Install.sh && chmod +x ./MicroROS_Install.sh && ./MicroROS_Install.sh
```

### Copy scripts on pc
```
cd ~
sudo apt install wget -y
wget https://raw.githubusercontent.com/TitouanMelon/ENIB_Robot_Mobile_Ros2/main/conception/installation%20scripts/STM32.sh
wget https://raw.githubusercontent.com/TitouanMelon/ENIB_Robot_Mobile_Ros2/main/conception/installation%20scripts/SET_ROS_DOMAIN_ID.sh
sed -i 's/\r$//' ./STM32.sh && chmod +x ./STM32.sh
sed -i 's/\r$//' ./SET_ROS_DOMAIN_ID.sh && chmod +x ./SET_ROS_DOMAIN_ID.sh
echo "ROS_DOMAIN_ID=0" | sudo tee -a /etc/environment
sudo ln -s ./STM32.sh /usr/bin/stm32_ros_agent 
sudo ln -s ./SET_ROS_DOMAIN_ID.sh /usr/bin/set_ros_id
```

## For Pc only
>[!NOTE]
>this section is recommended for a pc but it is possible to apply it to the raspberry PI

### STM32CubeIde

Get <a href="https://www.st.com/en/development-tools/stm32cubeide.html">STM32CubeIde</a> and install it 

#### Install

Go to the **Get Software** level and select the latest version of your OS

Next accept the EULA and fil the form with the name of your choice but a valid email address on the next prompt

![STM32 prompt](./conception/img/stm32.PNG)

You will receive an email with the download link

After download just launch the executable and follow the instructions

#### Install gtkterm
```
sudo apt install gtkterm -y
```

### QT creator and QT creator ros
you need to install the qtcreator and qtcreator-ros software

```
sudo apt install qtcreator -y
sudo snap install qtcreator-ros --classic
```

# Get startup workspace ready
## Get the STM32 workspace
```
sudo apt install wget unzip -y
wget https://github.com/TitouanMelon/ENIB_Robot_Mobile_Ros2/raw/main/startupCode/WORKSPACE_F411_uROS.zip
unzip WORKSPACE_F411_uROS.zip
```

## Get the qt workspace
```
sudo apt install wget unzip -y
wget https://github.com/TitouanMelon/ENIB_Robot_Mobile_Ros2/raw/main/startupCode/IHM.zip
unzip IHM.zip
```

## Get the camera workspace
>[!NOTE]
>You need to have ROS2 install to follow this tutorial (see [here](../README.md#install-ros2) to install)

Go to your project directory and run this command
```
rosdep install -i --from-path py_pubsub/py_pubsub --rosdistro humble -y
wget https://github.com/TitouanMelon/ENIB_Robot_Mobile_Ros2/raw/main/startupCode/Camera.zip
unzip Camera.zip
```

# Get final workspace
```
sudo apt install git -y
git clone https://github.com/TitouanMelon/ENIB_Robot_Mobile_Ros2.git
mkdir -p ~/robot
mv -r ./ENIB_Robot_Mobile_Ros2/finalCode/* ~/robot/
#uncomment to remove git directory
#rm -r ./ENIB_Robot_Mobile_Ros2

```

# Launch procedure
* Build and run IHM
Open a terminal in the IHM directory and run this command

```
colcon build
source install/setup.bash
./build/robot_mobile_pkg_cpp/my_node_ihm
# if you change the name of project change this line like this ./build/<node_name>/<executable_name>
```

* Compile, build and run the camera program
```
colcon build --packages-select py_pubsub
source install/setup.bash
ros2 run py_pubsub camera
```

* Start the STM32.sh or stm32_ros_agent
```
stm32_ros_agent
```
* Flash the stm32 and run the code
>[!NOTE]
>If multiple pattern error clean and rebuild until the error dissipear
- Flash the stm32 and run

# modify code
## Open stm32 project
>[!NOTE]
>you need STM32CubeIde to follow this tutorial (see [here](../README.md#STM32CubeIde) to install)

- Open stm32cubeIde and select the workspace and select ok

## Open IHM
>[!NOTE]
>you need qtcreator and qtcreator-ros to follow this tutorial (see [here](../README.md#qt-creator-and-qt-creator-ros) to install)

- Open qtcreator-ros
- go to File -> Open project or file

![openproject](../conception/img/openWorkspace.png)
- Open the file 'workspace.user'
- Then close and repeat to make qtcreator-ros load the project files

## Open camera code
- You can code directly on the RPI or use the visual studio code to connect in ssh to your raspberryPi and have access to his folder 

# Other information
## Config camera
```
sudo apt install v412-ctl
v4l2-ctl -d /dev/video0 --all
```

## Publish an array of 3 Int8 on the topic camera/hsv_low once
```
ros2 topic pub --once camera/hsv_low std_msgs/msg/Int8MultiArray "{data : [99,91,30]}"
```

## Pinout

| Pin STM32 | Function      | Use             |
|---------- | ------------- | --------------- |
|PA8        | PWM1/1        | Encodeur Voie A |
|PA9        | PWM1/2        | Encodeur Voie B |
|PB10       | EXTI1         | Index encodeur  |
|PA0        | PWM2/1        | Encodeur Voie A |
|PA1        | PWM2/2        | Encodeur Voie B |
|PC0        | EXTI          | Index Moteur    |
|PA6        | PWM3/1        | PWM motor 1     |
|PC7        | PWM3/2        | PWM motor 2     |
|PB3        | ENABLE MOTEUR | actif état Bas  |
|PA4        | ADC1_8        | Sensor IR 1     |
|PB0        | ADC1_4        | Sensor IR 2     |

## Robot cablage
![STM32 prompt](./conception/img/UART1_STM32.jpg)
![STM32 prompt](./conception/img/Shield_Ard.jpg)
