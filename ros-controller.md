# Velocity Pipeline Overview

Flow:
  /cmd_vel_raw  --> nav2_velocity_smoother --> /cmd_vel --> robot_driver --> hardware

robot_driver:
  - No acceleration / jerk shaping
  - Only safety clamp (+8%) and timeout
  - Encoder filtering & odometry integration

Tuning Steps:
1. Disable jerk (comment out max_jerk).
2. Log step response (ros2 bag record /cmd_vel_raw /cmd_vel /odom).
3. Measure rise time, overshoot, steady-state error.
4. Adjust max_accel / max_decel until desired compromise.
5. Enable jerk if velocity transitions feel too sharp.
6. Move feedback from OPEN_LOOP to ODOMETRY once odom drift & delay are acceptable.
7. Set driver safety_scale_* ~1.05–1.10 relative to smoother limits.

Bypass:
  ros2 launch ... use_raw_cmd_vel:=true
  -> robot_driver subscribes /cmd_vel_raw directly (for debugging).