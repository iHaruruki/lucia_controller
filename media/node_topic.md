---
config:
  layout: elk
---
flowchart TB
    cmdvel["/cmd_vel<br>geometry_msgs/Twist"] -- subscribe --> controller(["lucia_controller_node"]) & controller_ekf(["lucia_controller_ekf_node"])
    controller -- publish --> odom["/odom<br>nav_msgs/Odometry"]
    controller_ekf -- publish --> wheelodom["/wheel_odom<br>nav_msgs/Odometry"]
    wheelodom -- subscribe --> ekf_node(["ekf_odom"])
    ekf_node -- publish --> odomfilt["/odometry/filtered<br>nav_msgs/Odometry"]
    controller -. write .-> yarp_cmd["YARP:  /robot_driver/command:o"]
    controller_ekf -. write .-> yarp_cmd
    yarp_cmd -.-> hardware(["Lucia Robot Hardware"])
    hardware -.-> yarp_enc["YARP: /robot_driver/encoder:i"]
    yarp_enc -. read .-> controller & controller_ekf
