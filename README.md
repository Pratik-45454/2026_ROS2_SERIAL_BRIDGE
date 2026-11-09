# 2026_ROS2_SERIAL_BRIDGE

Serial bridge between the **Jetson Orin Nano** and the **STM32** (Bluepill / Nucleo) boards, plus the odometry and velocity plumbing around it.

## Data flow

```
 servers ──► cmd_multiplex ──► /cmd_vel ──► serial_bridge_node ══UART══► STM32
                                                   │
        /odom_data_received, /imu_data_received, /sick_data,
        /prox_front_received, /prox_rear_received
                                                   ▼
                              odometry · process_odom · servers / BT
```

Frames are `0xA5`-prefixed with a CRC8 checksum (`dev_pi_communicate/crc8.py`) at 115200 baud over a CP2102 USB-UART adapter.

## Nodes (`scripts/`)

| Script | Role | Key topics |
|--------|------|------------|
| `serial_bridge_node.py` | UART link to the STM32. Sends velocity + yaw setpoint, receives IMU, deadwheel counts, SICK, proximity. Logs odom to `~/tmp1/*.csv` | sub `/cmd_vel`, `/setpoint_yaw` · pub `/imu_data_received`, `/odom_data_received`, `/sick_data`, `/prox_front_received`, `/prox_rear_received` |
| `cmd_multiplex.py` | Picks the freshest `Twist` (1 s staleness) from the active controller | sub `/move_server/cmd_vel`, `/rotate_server/cmd_vel`, `/vel_pub/cmd_vel`, `/brake_server/cmd_vel` · pub `/cmd_vel` |
| `odometry.py` | Deadwheel + IMU dead reckoning | sub `/odom_data_received`, `/imu_data_received` · pub `/odom` |
| `process_odom.py` | Raw encoder counts → distance in metres | pub `/odom_dist_m` |
| `vel_pid.py` | Velocity-tracking PID on top of `/cmd_vel` | pub `/adjusted_vel` |
| `motor_launcher.py` | Byte command → motor init service calls (climb, roll, elevator, gripper) | sub `/activate_motors_data` |
| `gpio_button.py` | Jetson GPIO start/reset button | pub `/set_reset` |
| `serial_bridge_display.py`, `grippersim.py` | Debug display and gripper simulation | |

`dev_pi_communicate/` holds the shared serial, CRC8, joystick and camera helpers.

## Setup

```bash
# Serial port is hardcoded in serial_bridge_node.py (by-id path of the CP2102 adapter)
ls /dev/serial/by-id/
sudo usermod -aG dialout $USER     # re-login after
```

```bash
cd ~/arrobo_ws
colcon build --symlink-install --packages-select dev_pi_communicate
source install/setup.bash
```

## Run

```bash
python3 src/2026_ROS2_SERIAL_BRIDGE/scripts/serial_bridge_node.py
python3 src/2026_ROS2_SERIAL_BRIDGE/scripts/cmd_multiplex.py
python3 src/2026_ROS2_SERIAL_BRIDGE/scripts/odometry.py
```

## Tuning

- SICK linearisation constants (`SICK_FR_M/B`, `SICK_FL_M/B`) and the EMA factor `_sick_alpha` are at the top of `serial_bridge_node.py`.
- `HARDCODED_MODE` sends a fixed velocity for bench testing without the BT.