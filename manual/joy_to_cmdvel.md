# Joy to Cmd_vel Converter

A ROS 2 Humble node that converts PlayStation DualShock 4 controller input to cmd_vel velocity commands with full 6-DOF (Degrees of Freedom) support.

## Controller Mapping

### Speed Control
- **Triangle Button**: 1.0x speed multiplier
- **Circle Button**: 2.0x speed multiplier
- **Cross Button**: 3.0x speed multiplier
- **PlayStation Button**: Emergency Stop (all velocities set to 0)

### Movement Control

#### D-Pad (Cross-Key) - Linear Motion
- **D-Pad Up**: Go forward (linear_x positive)
- **D-Pad Down**: Go backward (linear_x negative)
- **D-Pad Left**: Go left (linear_y negative)
- **D-Pad Right**: Go right (linear_y positive)

#### Shoulder Buttons - Angular Motion
- **L1 Button**: Turn left (angular_z positive, counter-clockwise)
- **R1 Button**: Turn right (angular_z negative, clockwise)

#### Analog Sticks
- **Left Analog Stick**: Currently reserved for mirror rotation control (commented out)
- **Right Analog Stick**: Available for future mapping

## Velocity Parameters

All parameters can be configured via ROS parameters:

| Parameter | Default | Unit | Description |
|-----------|---------|------|-------------|
| `linear_x_base` | 0.1 | m/s | Base linear velocity in X direction (forward/backward) |
| `linear_y_base` | 0.1 | m/s | Base linear velocity in Y direction (left/right) |
| `linear_z_base` | 0.1 | m/s | Base linear velocity in Z direction (up/down) |
| `angular_x_base` | 0.3 | rad/s | Base angular velocity around X axis (roll) |
| `angular_y_base` | 0.3 | rad/s | Base angular velocity around Y axis (pitch) |
| `angular_z_base` | 0.3 | rad/s | Base angular velocity around Z axis (yaw) |

**Speed Multiplier Effect**: All velocities are multiplied by the current speed multiplier (1.0x, 2.0x, or 3.0x).