# lucia_controller
### Node and Topic
## Dependency
    $ sudo apt install ros-humble-hardware-interface
YRAP
```
$ sudo apt install build-essential git cmake cmake-curses-gui
# Install YCM from source codes
$ cd
$ git clone https://github.com/robotology/ycm/
$ cd ycm && mkdir build && cd build
$ cmake ..
$ make
$ sudo make install
# Install other dependences
$ sudo apt-get install -y build-essential git cmake cmake-curses-gui ycm-cmake-modules libeigen3-dev libace-dev libedit-dev libsqlite3-dev libtinyxml-dev qtbase5-dev qtdeclarative5-dev qtmultimedia5-dev qml-module-qtquick2 qml-module-qtquick-window2 qml-module-qtmultimedia qml-module-qtquick-dialogs qml-module-qtquick-controls qml-module-qt-labs-folderlistmodel qml-module-qt-labs-settings libqcustomplot-dev libgraphviz-dev libjpeg-dev libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev gstreamer1.0-plugins-base gstreamer1.0-plugins-good gstreamer1.0-plugins-bad gstreamer1.0-libav
# YARP Install
$ cd ~/opt
$ git clone https://github.com/robotology/yarp.git
$ cd yarp && mkdir build && cd build
$ cmake ..
$ make -j8
$ sudo make install
$ sudo ldconfig
# Check YARP functions correctly
$ yarpserver
$ CTRL-C
# If "Ok. Ready!" is printed, it functions correctly
$ yarp conf
$ cd /.config/yaro/
$ cp yarp.conf _lucia_g.conf
$ gedit _lucia_g.conf
# Change the IP address to match Lucia
$ yarp namespace /lucia_g
```
## Setup
```
$ sudo apt update
$ sudo apt install ros-humble-hardware-interface
$ cd ~/ros2_ws/src  #Go to ros workspace
$ git clone https://github.com/iHaruruki/lucia_controller.git #clone this package
$ cd ~/ros2_ws
$ colcon build --symlink-install
$ source install/setup.bash
```

## Usage
1. Power on Lucia
2. Power on [Lucia-04-Creen-01-Main]
3. (Wi-Fi settings) Connect to lucia-g-router
4. Release the emergency stop button
5. Switch Lucia's mode to [Remote Movement] (YRAPをリモート移動モードに切り替える)
6. Launch ROS2 Node
    $ ros2 run lucia_controller lucia_controller_node

## License
## Authors

## References
