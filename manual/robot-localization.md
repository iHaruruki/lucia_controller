![](https://docs.ros.org/en/melodic/api/robot_localization/html/index.html)

# 手順
ビルド: colcon build --packages-select lucia_controller
実行: ros2 launch lucia_controller ekf_odom.launch.py
確認:
ros2 topic echo /wheel_odom （raw データ）
ros2 topic echo /odometry/filtered （EKF 出力）
ros2 run tf2_tools view_frames.py → odom→base_link のみ
rviz2 で /odometry/filtered の Pose 表示
速度コマンド: ros2 topic pub /cmd_vel geometry_msgs/Twist '{linear: {x:0.1,y:0.0}, angular:{z:0.2}}'
不具合時:
sensor_timeout 超過なら timestamps を確認
位置飛び: ramp/smoothing パラメータか covariance の見直し

# 調整ポイント
オドメトリが遅延する: publish 周期 (LOOP_PERIOD) を小さく、または EKF frequency を合わせる
採用する成分をもっと速度寄りにしたい: pose の covariance をさらに大きくして yaw のみやや小さく保つ
IMU 追加時: ekf_odom.yaml に imu0, imu0_config を追記（two_d_mode=true なら roll/pitch 固定）
