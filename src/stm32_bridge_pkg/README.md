# stm32_bridge_pkg

ROS 2 bridge package for the STM32 unified device controller.

This package replaces the old direct Linux control path. ROS now sends one text protocol over a single STM32 serial port:

```text
#<id> <device> <action> [args...]
```

The node parses STM32 responses:

```text
@ack id=<id> dev=<device> cmd=<action> result=<result>
@done id=<id> dev=<device> cmd=<action> result=<result>
@state id=<id> dev=<device> result=<result> ...
@err id=<id> dev=<device> cmd=<action> code=<code> ...
```

## Build

```bash
cd Auto_control_ws
colcon build --packages-select stm32_bridge_pkg
source install/setup.bash
```

## Run

```bash
ros2 run stm32_bridge_pkg stm32_bridge_node --ros-args \
  -p stm32.port:=/dev/ttyUSB0 \
  -p stm32.baudrate:=115200
```

## Debug Topics

| Topic | Type | Purpose |
|---|---|---|
| `/stm32/raw_tx` | `std_msgs/msg/String` | Actual command lines sent to STM32 |
| `/stm32/raw_rx` | `std_msgs/msg/String` | Parsed STM32 protocol lines from STM32 |
| `/stm32/error` | `std_msgs/msg/String` | Timeout, protocol, and STM32 `@err` messages |
| `/stm32/command` | `std_msgs/msg/String` | Direct command body, for example `clamp status` |
| `/emergency_stop` | `std_msgs/msg/Empty` | Sends `mtor1 stop`, `mtor2 stop`, `clamp release`, `vacum stop` |

## Control Topics

```bash
ros2 topic pub --once /stm32/command std_msgs/msg/String "{data: 'mtor1 status'}"
ros2 topic pub --once /mtor1/move geometry_msgs/msg/Vector3 "{x: 5.0, y: 100.0, z: 100.0}"
ros2 topic pub --once /mtor2/move geometry_msgs/msg/Vector3 "{x: -2.0, y: 80.0, z: 100.0}"
ros2 topic pub --once /clamp/move_percent geometry_msgs/msg/Vector3 "{x: 100.0, y: 30.0, z: 1.0}"
ros2 topic pub --once /vacuum/set_params geometry_msgs/msg/Vector3 "{x: 30.0, y: 70.0, z: 10.0}"
ros2 topic pub --once /vacuum/grip std_msgs/msg/Empty "{}"
ros2 topic pub --once /emergency_stop std_msgs/msg/Empty "{}"
```

## State Topics

| Topic | Type | Notes |
|---|---|---|
| `/mtor1/state` | `sensor_msgs/msg/JointState` | Position is output shaft revolutions |
| `/mtor2/state` | `sensor_msgs/msg/JointState` | Position is output shaft revolutions |
| `/clamp/state` | `sensor_msgs/msg/JointState` | Position is gap percent, `100=open`, `0=closed` |
| `/clamp/status_text` | `std_msgs/msg/String` | Extra clamp fields as text |
| `/vacuum/status_text` | `std_msgs/msg/String` | Vacuum state fields as compact JSON |

Status is queried manually:

```bash
ros2 topic pub --once /mtor1/status_query std_msgs/msg/Empty "{}"
ros2 topic pub --once /mtor2/status_query std_msgs/msg/Empty "{}"
ros2 topic pub --once /clamp/status_query std_msgs/msg/Empty "{}"
ros2 topic pub --once /vacuum/status_query std_msgs/msg/Empty "{}"
```
