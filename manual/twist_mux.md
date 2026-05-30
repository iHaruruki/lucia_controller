

## Rejection of control command
- トピックから `true` が送られてくるとロックが有効になり，速度入力をブロック
- 有効になったロックは，自身と同じ，または自身より低い優先度（Priority）を持つトピックのみをブロック

### **Tatto**と**Nav2**からの制御コマンドを拒否（自律移動も停止する）
```bash
ros2 topic pub /reject_tatto_vel std_msgs/msg/Bool "{data: true}" --once 
```
速度入力のブロックを解除
```bash
ros2 topic pub /reject_tatto_vel std_msgs/msg/Bool "{data: false}" --once 
```
### 幕張メッセから受け取る全制御コマンドを拒否（自律移動も停止する）
```bash
ros2 topic pub /reject_makuhari_vel std_msgs/msg/Bool "{data: true}" --once 
```
速度入力のブロックを解除
```bash
ros2 topic pub /reject_makuhari_vel std_msgs/msg/Bool "{data: false}" --once 
```

### Dualshock4 で`nav_vel`を制御する
自律移動解除： PS ボタン
自律移動に復帰：四角形ボタン