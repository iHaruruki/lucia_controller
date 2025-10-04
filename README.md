# lucia_controller
> An elegant bridge between Lucia robots, YARP, and ROS 2 🚀


Table of Contents
- Features
- Nodes & Topics
- Requirements
- Quick Setup
  - System dependencies
  - YCM (YARP CMake Modules)
  - YARP build & configuration
  - ROS 2 package build
- Usage
- YARP network example
- Troubleshooting
- Contributing
- License
- Authors
- References

---

## 🚀 Overview
**lucia_controller** connects Lucia robot hardware to your ROS 2 ecosystem using YARP, featuring:
- Effortless launch for hardware interfaces & sensor fusion (EKF)
- Clean networking, reproducible builds
- Modern, readable code

## 📦 Features
- Seamless integration between YARP and ROS 2.
- Launch files to bring up hardware interfaces and optional EKF.
- Clear build and network configuration steps for reproducible setup.

## 🧩 Nodes & Topics
- `lucia_controller_node`: main hardware interface
- **Example Topics:**
  - `/cmd_vel` _(geometry_msgs/Twist)_
  - `/odom` _(nav_msgs/Odometry)_
  - `/joint_states` _(sensor_msgs/JointState)_
  - `/diagnostics`

## 📋 Requirements
- **OS:** Ubuntu 22.04
- **Middleware:** ROS 2 Humble
- **Build tools:** cmake, git, build-essential
- **YARP:** recommended build from source
- **Dev libraries:** see Quick Setup

## Setup
### YRAP

Install basic build tools
```shell
sudo apt install build-essential git cmake cmake-curses-gui
```
Install YCM (YARP CMake Modules) from source
```shell
git clone https://github.com/robotology/ycm/
cd ycm && mkdir build && cd build
cmake ..
make
sudo make install
```
Additional dependencies
```shell
sudo apt-get install -y build-essential git cmake cmake-curses-gui ycm-cmake-modules libeigen3-dev libace-dev libedit-dev libsqlite3-dev libtinyxml-dev qtbase5-dev qtdeclarative5-dev qtmultimedia5-dev qml-module-qtquick2 qml-module-qtquick-window2 qml-module-qtmultimedia qml-module-qtquick-dialogs qml-module-qtquick-controls qml-module-qt-labs-folderlistmodel qml-module-qt-labs-settings libqcustomplot-dev libgraphviz-dev libjpeg-dev libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev gstreamer1.0-plugins-base gstreamer1.0-plugins-good gstreamer1.0-plugins-bad gstreamer1.0-libav
```
Build and install YARP
```shell
git clone -b yarp-3.11 https://github.com/robotology/yarp.git
cd yarp && mkdir build && cd build
cmake ..
make -j8
sudo make install
sudo ldconfig
```
> [!NOTE]
> Use a version older than YARP-3.11

Verify YARP server
```shell
yarpserver
# Press CTRL-C to stop
# You should see "Ok. Ready!" if it started successfully.
```

YARP network configuration example
```shell
yarp conf
cd ~/.config/yarp/
cp yarp.conf _lucia_g.conf
nano _lucia_g.conf
```
YARP network configuration example
```fiff_plaintext
- 192.168.27.132 10000 yarp
+ 192.168.1.221 10000 yarp
```
Set YARP namespace
```shell
yarp namespace /lucia_g
```
## ROS2 Packages install & build
Install robot-localization
```bash
sudo apt update
sudo apt install ros-humble-robot-localization
```
Clone & Build
```shell
cd ~/ros2_ws/src  #Go to ros workspace
git clone https://github.com/iHaruruki/lucia_controller.git #clone this package
cd ~/ros2_ws
colcon build --symlink-install --packages-select lucia_controller
source install/setup.bash
```

## 🛠️ Usage
1. Power on Lucia and NUC 21
2. Power on [Lucia-04-Green-01-Main]
3. (Wi-Fi settings) Connect to [lucia-g-router]
4. Release the emergency stop button
5. Switch Lucia's mode to [Remote] (`remote`モードに切り替える)
6. Launch ROS2 Node
```shell
ros2 launch lucia_controller bringup.launch.py 
```
If you want to use EKF, pelase run:
```shell
ros2 launch lucia_controller bringup_ekf.launch.py
```
## 📜 License

## 👤 Authors

- **[iHaruruki](https://github.com/iHaruruki)** — Main author & maintainer

## 🔗 References
- [YARP](https://github.com/robotology/yarp)
- [YCM](https://github.com/robotology/ycm)
- [ROS 2 Humble](https://docs.ros.org/en/humble/)
- [robot localization](https://docs.ros.org/en/melodic/api/robot_localization/html/index.html)
