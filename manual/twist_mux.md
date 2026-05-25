- トピックから `true` が送られてくるとロックが有効になり，速度入力をブロック
- 有効になったロックは，自身と同じ，または自身より低い優先度（Priority）を持つトピックのみをブロック
```bash
ros2 topic pub /key_priority std_msgs/msg/Bool "{data: true}" --once 
```
速度入力のブロックを解除
```bash
ros2 topic pub /key_priority std_msgs/msg/Bool "{data: true}" --once 
```