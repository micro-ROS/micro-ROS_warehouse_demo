# micro-ROS_warehouse_demo

## Setting up the sensors

### Enviroment set up using docker

Download the micro-ROS base Foxy image from the Docker Hub https://hub.docker.com/, then run a docker container.

```bash
sudo docker pull microros/base:foxy
sudo docker run -it --net=host --privileged -v /dev/bus/usb:/dev/bus/usb microros/base:foxy
```

### Create a ROS 2 workspace in the uros_ws folder of the docker container and build the package.

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
git clone -b $ROS_DISTRO https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup
apt update && rosdep update
rosdep install --from-path src --ignore-src -y
apt-get install python3-pip
apt-get -y install python3-pip
colcon build
source install/local_setup.bash
```

### Create the Nuttx firmware on Olimex-E407 with user case sensor applications.

```bash
ros2 run micro_ros_setup create_firmware_ws.sh nuttx olimex-stm32-e407
cd firmware/NuttX
git checkout -t origin/ucs_demo_f
cd ../apps
git checkout -t origin/ucs_demo_f
cd ..
```

### Configurate NUTTX tools to be able using the menuconfig application.

```bash
git clone https://bitbucket.org/nuttx/tools.git firmware/tools
cd firmware/tools/kconfig-frontends
./configure
autoreconf -f -i 
make
make install
ldconfig
cd -
```

### Install additional applications

```bash
apt update
apt install -y genromfs vim
```

### Complete configuration with micro-ros DEMO BOX messages and the convinient config file for flashing firmware,

```bash
git clone http://10.0.9.18/abratek/uros-ucs.git ./tmp
cp -r ./tmp/uros_demo/ ./firmware/mcu_ws/uros_demo
cp ./tmp/config_f/flash_dashing.sh ./install/micro_ros_setup/config/nuttx/generic/flash.sh 
rm -rf tmp/
```

### Building the laser distance application

Set the config profile variable to select the TFMini distance application.

```bash
CFG_PROFILE=ucs_distance_romfs
```

Follow up the building and flashing procedure

### Building the humidity sensor application

Set the config profile variable to select the Hih6130 humidity application.

```bash
CFG_PROFILE=ucs_hih6130_romfs
```

Follow up the building and flashing procedure

### Building the door opener application

Set the config profile variable to select the door opener application.

```bash
CFG_PROFILE=ucs_opener_romfs
```

Follow up the building and flashing procedure

### Building the Door Final Effector

Set the config profile variable to select the final effector application.

```bash
CFG_PROFILE=ucs_effector_romfs
```

Follow up the building and flashing procedure

### Building and flashing an application firmware

Run the following commands in the micro-ROS workspace folder to get the config profile, prepare a startup script, then build and flash an application to the Olimex-E407 board.

```bash
cd /uros_ws/
ros2 run micro_ros_setup configure_firmware.sh $CFG_PROFILE
cp firmware/NuttX/configs/olimex-stm32-e407/$CFG_PROFILE/rcS.template firmware/apps/nshlib/rcS.template
cd firmware/apps/nshlib/
../../NuttX/tools/mkromfsimg.sh -nofat ../../NuttX/
cd /uros_ws/
ros2 run micro_ros_setup build_firmware.sh
ros2 run micro_ros_setup flash_firmware.sh
```
