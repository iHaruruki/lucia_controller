---
config:
  layout: elk
---
flowchart TB
    cmdvel["/cmd_vel<br>(geometry_msgs/Twist)"] -- subscribe --> controller(["lucia_controller_node"])
    controller -- publish --> odom["/odom<br>(nav_msgs/Odometry)"]
    controller -. write .-> yarp_cmd["YARP:  /robot_driver/command:o"]
    yarp_cmd -.-> hardware[["Hardware (Motor & Encoder)"]]
    hardware -.-> yarp_enc["YARP: /robot_driver/encoder:i"]
    yarp_enc -. read .-> controller
