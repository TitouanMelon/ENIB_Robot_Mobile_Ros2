# ENIB_Robot_Mobile_Ros2

# Install OS

## On a PC
  Install <a href="https://ubuntu.com/download/desktop" >Ubuntu 22.04.3 LTS</a> on a pc
## On a RPI
Install <a href="https://www.raspberrypi.com/software/">RPI imager</a> and with this install Ubuntu like this

CHOOSE OS -> Other general-purpose OS -> Ubuntu -> Ubuntu Desktop 22.04.03 LTS (64bits)

# Install ROS2

ROS2 installation is the same on PC and on RPI

```
sudo apt install wget
wget https://github.com/TitouanMelon/ENIB_Robot_Mobile_Ros2/edit/main/script/rosInstall.sh
chmod +x ./rosInstall.sh
./rosInstall.sh
```

# Install MicroRos

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
# Install STM32CubeIde

Get <a href="https://www.st.com/en/development-tools/stm32cubeide.html">STM32CubeIde</a> and install it 

## Install

Go to the **Get Software** level and select the latest version of your OS

Next accept the EULA and fil the form with the name of your choice but a valid email address on the next prompt

![STM32 prompt](./img/stm32.PNG)

You will receive an email with the download link

After download just launch the executable and follow the instructions

## Get the workspace

```
sudo apt install git
git clone https://github.com/TitouanMelon/ENIB_Robot_Mobile_Ros2.git
cp -r ./WORKSPACE_F411_uROSS6 /your/path/of/workspace/folder
# Uncomment the next line to remove the git folder after move workspace
#rm -r ENIB_Robot_Mobile_Ros2
```
