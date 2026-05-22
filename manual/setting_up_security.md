## Create a folder for the security fiel
```bash
mkdir ~/sros2_demo
```
## Generate a keystore
```bash
cd ~/sros2_demo
ros2 security create_keystore demo_keystore
```
## Generate keys and certificates
```bash
ros2 security create_enclave demo_keystore /talker_listener/talker
ros2 security create_enclave demo_keystore /talker_listener/listener
```
## Configure environment variables
```bash
export ROS_SECURITY_KEYSTORE=~/sros2_demo/demo_keystore
export ROS_SECURITY_ENABLE=true
export ROS_SECURITY_STRATEGY=Enforce
```
Run the `talker/listerner` demo
Begin the demo by launching the talker node.
```bash
ros2 run demo_nodes_cpp talker --ros-args --enclave /talker_listener/talker
```
In another terminal, do the same to launch the listener node.
```bash
ros2 run demo_nodes_py listener --ros-args --enclave /talker_listener/listener
```
## Use `ros2cli` with security
To use ros2cli to iterate with ROS 2 secured network, you need to provide it with override enclave by ROS_SECURITY_ENCLAVE_OVERRIDE environmental variable. Open an another terminal and set up the following environmental variables.
```bash
export ROS_SECURITY_KEYSTORE=~/sros2_demo/demo_keystore
export ROS_SECURITY_ENABLE=true
export ROS_SECURITY_STRATEGY=Enforce
export ROS_SECURITY_ENCLAVE_OVERRIDE=/talker_listener/listener
```
Now you can use ros2cli to communicate with ROS 2 secured network.
```bash
ros2 node list --no-daemon --spin-time 3
```
```bash
ros2 topic list --no-daemon --spin-time 3
```