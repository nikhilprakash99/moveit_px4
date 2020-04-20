# ROS SITL Package for px4

This is a ros package implementation of the sitl to be used with the px4 firmware

## Setup

### PX4 Firmware

clone the px4 firmware to your desired location

```bash
 git clone --recurse-submodules -j8 git@github.com:PX4/Firmware.git
```
--recurse-submodules : clones all submodules 
-j8 : an optional argument that speeds up the fetching of multiple submodules

### Dependancies

install firmware dependancies using the provided installtion script that comes with it

note: make sure pip3 is installed before running the script . else esential python3 modules will be skipped

```bash
sudo apt install python3-pip
cd <Firmware_clone>
Tools/setup/ubuntu.sh -y
```

### Build SITL

build SITL using px4 once to generate essential libraries

```bash
cd <Firmware_clone>
DONT_RUN=1 make px4_sitl_default gazebo
```

### Set up Gazebo and ROS Paths

add the following lines to ur .rc file

```bash
FIRMWARE='Path_To_Your_Firmware_Clone'
source $FIRMWARE/Tools/setup_gazebo.bash $FIRMWARE $FIRMWARE/build/px4_sitl_default > /dev/null
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$FIRMWARE
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$FIRMWARE/Tools/sitl_gazebo
```

## Adding Custom Models

1. Create a folder under sitl_gazebo/models for your model, let’s call it my_vehicle
2. Create the following files under sitl_gazebo/models/my_vehicle: model.config and my_vehicle.sdf
3. (ptional) Create a world file in Tools/sitl_gazebo/worlds called my_vehicle.world
4. Create an airframe file under px4fmu_common/init.d-posix, give it a number (for example 4229) and name it 4229_my_vehicle. base the airframe name out of simmilar models

## TODO

* support for saving and loading cutom parameters while launching
* keeping models and worlds updated with upstream sitl_gazebo 