# Joy to Cmd_vel Converter

A ROS 2 Humble node that converts PlayStation DualShock 4 controller input to cmd_vel velocity commands.

## Controller Mapping

### Speed Control
- **Triangle**: 1.0x speed
- **Circle**: 2.0x speed
- **Cross**: 3.0x speed
- **PlayStation Button**: Emergency Stop

### Movement
- **D-Pad Up**: Go forward (0.4 m/s × multiplier)
- **D-Pad Down**: Go backward (-0.4 m/s × multiplier)
- **D-Pad Left**: Turn left (0.8 rad/s × multiplier)
- **D-Pad Right**: Turn right (-0.8 rad/s × multiplier)
- **L1**: Turn left (0.8 rad/s × multiplier)
- **R1**: Turn right (-0.8 rad/s × multiplier)
- **Left Analog Stick Up**: Mirror rotate minus (negative angular)
- **Left Analog Stick Down**: Mirror rotate plus (positive angular)