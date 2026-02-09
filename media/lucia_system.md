---
config:
  layout: elk
---
flowchart TB
 subgraph Lucia["PC installed in Lucia"]
        controller(["lucia_controller_node"])
        cmdvel["/cmd_vel<br>(geometry_msgs/Twist)"]
        odom["/odom<br>(nav_msgs/Odometry)"]
        yarp_cmd["YARP:  /robot_driver/command:o"]
        hardware[["Hardware (Motor & Encoder)"]]
        yarp_enc["YARP: /robot_driver/encoder:i"]
        SCAN["/scan<br>(sensor_msgs/LaserScan)"]
        LIDAR(["lidar_node"])
  end
 subgraph NUC["External PC"]
        SLAM(["salm_node"])
        NAV(["navigation_node"])
        AUDIO(["audio_guidance_node"])
        SPINA(["arm_controll_node"])
        VITAL(["vital_sensor_node"])
        MAP["map<br>(nav_msgs/OccupancyGrid)"]
  end
    cmdvel -- subscribe --> controller
    controller -- publish --> odom
    controller -. write .-> yarp_cmd
    yarp_cmd -.-> hardware
    hardware -.-> yarp_enc
    yarp_enc -. read .-> controller
    LIDAR --> SCAN
    SCAN --> SLAM & NAV
    odom --> SLAM & NAV
    NAV --> AUDIO
    AUDIO --> SPINA
    SPINA --> AUDIO
    VITAL --> AUDIO
    SLAM --> MAP