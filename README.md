# ENIB_Robot_Mobile_Ros2

Project to make a moving robot with communication and sensors

![robot sheme](./conception/img/robotScheme.png)

# Folder description

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

# Install OS

## On a PC
  Install <a href="https://ubuntu.com/download/desktop" >Ubuntu 22.04.3 LTS</a> on a pc
## On a RPI
Install <a href="https://www.raspberrypi.com/software/">RPI imager</a> and with this install Ubuntu like this

CHOOSE OS -> Other general-purpose OS -> Ubuntu -> Ubuntu Desktop 22.04.03 LTS (64bits)

# Install all in one script
>[!NOTE]
>this section is recommended for a pc but it is possible to apply it to the raspberry PI

You can install all necessary tools with this following commands.
```
wget ROS.sh
wget STM32cubeIDE.sh
sed blabla && ./ROS.sh
```

# For PC and RaspberryPi
## Install ROS2
ROS2 installation is the same on PC and on RPI

```
sudo apt install wget
wget https://github.com/TitouanMelon/ENIB_Robot_Mobile_Ros2/edit/main/script/rosInstall.sh
chmod +x ./rosInstall.sh
./rosInstall.sh
```

## Install MicroRos

>[!WARNING]
>Be carreful the installation of microRos require a minimum of 1GB of RAM to succes

>[!NOTE]
>In order to install on a RPI4 with 2GB of RAM you can remove gdm3 install the script with the console and reinstall gdm3

```
sudo apt install wget
wget https://github.com/TitouanMelon/ENIB_Robot_Mobile_Ros2/edit/main/script/microRosInstall.sh
chmod +x ./microRosInstall.sh
./microRosInstall.sh
```

## Copy scripts on pc
```
cd ~
wget https://github.com/TitouanMelon/ENIB_Robot_Mobile_Ros2/blob/main/conception/installation%20scripts/STM32.sh
wget https://github.com/TitouanMelon/ENIB_Robot_Mobile_Ros2/blob/main/conception/installation%20scripts/SET_ROS_DOMAIN_ID.sh
sudo ln -s ./STM32.sh /usr/bin/stm32_ros_agent 
sudo ln -s ./SET_ROS_DOMAIN_ID.sh /usr/bin/set_ros_id
```

# For Pc only
>[!NOTE]
>this section is recommended for a pc but it is possible to apply it to the raspberry PI

## STM32CubeIde

Get <a href="https://www.st.com/en/development-tools/stm32cubeide.html">STM32CubeIde</a> and install it 

### Install

Go to the **Get Software** level and select the latest version of your OS

Next accept the EULA and fil the form with the name of your choice but a valid email address on the next prompt

![STM32 prompt](./conception/img/stm32.PNG)

You will receive an email with the download link

After download just launch the executable and follow the instructions

### Get the workspace

```
sudo apt install git unzip -y
git clone https://github.com/TitouanMelon/ENIB_Robot_Mobile_Ros2.git
unzip ./ENIB_Robot_Mobile_Ros2/WORKSPACE_F411_uROS6/base_robot/micro_ros_stm32cubemx_utils.zip -o -d ./ENIB_Robot_Mobile_Ros2/WORKSPACE_F411_uROSS6/base_robot/
cp -r ./ENIB_Robot_Mobile_Ros2/WORKSPACE_F411_uROS6 /your/path/of/workspace/folder
# Uncomment the next line to remove the git folder after move workspace
#sudo rm -r ENIB_Robot_Mobile_Ros2
```

### Install gtkterm
```
sudo apt install gtkterm -y
```

## QT creator and QT creator ros
```
sudo apt install qtcreator -y
sudo snap install qtcreator-ros --classic
```

# Pinout

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


![STM32 prompt](./conception/img/UART1_STM32.jpg)
![STM32 prompt](./conception/img/Shield_Ard.jpg)
