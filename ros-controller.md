運用手順

ビルド colcon build --packages-select lucia_controller
生コマンド発行 (例: teleop) ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/cmd_vel_raw
平滑化起動 ros2 launch lucia_controller velocity_smoothing.launch.py
ドライバ起動（別ターミナル） ros2 run lucia_controller lucia_controller_node
動作確認
ros2 topic echo /cmd_vel_raw
ros2 topic echo /cmd_vel
/cmd_vel が段差 → 滑らかな波形に
パラメータ調整
初回は max_jerk をコメントアウトして応答を比較
加速が遅い → max_accel を少し増やす (0.55→0.65 など)
減速が鋭すぎる → max_decel の絶対値を減らす (-0.70→-0.55)
よくある調整ポイント

オーバーシュート: jerk 値大きすぎ → 下げる または削除
反応遅延: smoothing_frequency を 40Hz, jerk 制限緩和, accel 増
ガタ小さいが微振動: smoothing_frequency を 25–30Hz に下げる
追加で必要であれば

diagnostics 出力
velocity_smoother パラメータ動的変更 (dyn params)
/cmd_vel_raw をログ解析して加速度自動推定する Python スクリプト
なども提供可能です。必要があれば指示ください。