# lucia_controller

## 🚀 Overview
**lucia_controller** connects Lucia robot hardware to your ROS 2 ecosystem using YARP, featuring:

## 📦 Features
- Seamless integration between YARP and ROS 2.
- Launch files to bring up hardware interfaces.

## 🧩 Nodes & Topics
- `lucia_controller_node`: main hardware interface
- **Topics:**
  - `/cmd_vel` _(geometry_msgs/Twist)_
  - `/odom` _(nav_msgs/Odometry)_

![Node & Topics](/manual/node_topic_conection/node_motor.png)

## 📋 Requirements
- **OS:** Ubuntu 22.04
- **ROS version:** ROS 2 Humble
- **YARP:** Use a version older than `YARP-3.11`

## Setup
### YRAP

Install basic build tools
```bash
sudo apt install build-essential git cmake cmake-curses-gui
```
Install YCM (YARP CMake Modules) from source
```bash
git clone https://github.com/robotology/ycm/
cd ycm && mkdir build && cd build
cmake ..
make
sudo make install
```
Additional dependencies
```bash
sudo apt-get install -y build-essential git cmake cmake-curses-gui ycm-cmake-modules libeigen3-dev libace-dev libedit-dev libsqlite3-dev libtinyxml-dev qtbase5-dev qtdeclarative5-dev qtmultimedia5-dev qml-module-qtquick2 qml-module-qtquick-window2 qml-module-qtmultimedia qml-module-qtquick-dialogs qml-module-qtquick-controls qml-module-qt-labs-folderlistmodel qml-module-qt-labs-settings libqcustomplot-dev libgraphviz-dev libjpeg-dev libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev gstreamer1.0-plugins-base gstreamer1.0-plugins-good gstreamer1.0-plugins-bad gstreamer1.0-libav
```
Build and install YARP
```bash
git clone -b yarp-3.11 https://github.com/robotology/yarp.git
cd yarp && mkdir build && cd build
cmake ..
make -j8
sudo make install
sudo ldconfig
```
> [!IMPORTANT]
> Use a version older than `YARP-3.11`

Verify YARP server
```bash
yarpserver
# Press CTRL-C to stop
# You should see "Ok. Ready!" if it started successfully.
```

YARP network configuration example
```bash
cd ~
yarp conf
cd ~/.config/yarp/
cp yarp.conf _lucia_g.conf
nano _lucia_g.conf
```
YARP network configuration example  
`_lucia_g.conf`
```yaml
192.168.1.221 10000 yarp
```
Set YARP namespace
```bash
yarp namespace /lucia_g
```
## ROS2 Packages install & build
Install robot-localization
```bash
sudo apt update
sudo apt install ros-${ROS_DISTRO}-nav-msgs ros-${ROS_DISTRO}-nav2-bringup ros-${ROS_DISTRO}-tf2-ros ros-${ROS_DISTRO}-tf2-geometry-msgs ros-${ROS_DISTRO}-joy
```
Clone & Build
```bash
cd ~/ros2_ws/src  #Go to ros workspace
git clone https://github.com/iHaruruki/lucia_controller.git #clone this package
cd ~/ros2_ws
colcon build --symlink-install --packages-select lucia_controller
source install/setup.bash
```

## 🎮 Usage
1. Power on Lucia and NUC 21
2. Power on [Lucia-04-Green-01-Main]
3. (Wi-Fi settings) Connect to [lucia-g-router]
4. Release the emergency stop button
5. Switch Lucia's mode to [Remote] (`remote`モードに切り替える)
6. Launch ROS2 Node
```bash
ros2 launch lucia_controller bringup.launch.py 
```

## 🛠️ Debug
## Debug mode
bringup.launch.py
```bash
ros2 launch lucia_controller bringup.launch.py log_level:=debug
```
lucia_minimal_controller_node
```bash
ros2 run lucia_controller lucia_minimal_controller_node --ros-args --log-level debug
```
lucia_velocity_smoother
```bash
ros2 run lucia_controller lucia_velocity_smoother_node --ros-args --log-level debug
```

## 📜 License

## 👤 Authors

- **[iHaruruki](https://github.com/iHaruruki)** — Main author & maintainer

## 🔗 References
- [YARP](https://github.com/robotology/yarp)
- [YCM](https://github.com/robotology/ycm)
- [ROS 2 Humble](https://docs.ros.org/en/humble/)
- [ros2_control](https://control.ros.org/humble/index.html)
- [robot localization](https://docs.ros.org/en/melodic/api/robot_localization/html/index.html)
- [joy](https://docs.ros.org/en/humble/p/joy/index.html)
