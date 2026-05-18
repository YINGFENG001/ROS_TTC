# ROS_TTC

ROS 2 workspace for TTC hardware control.

## Current Control Path

The active control package is `stm32_bridge_pkg`.

ROS communicates with STM32 through one serial connection. STM32 is responsible for the lower-level device control of:

- `mtor1`
- `mtor2`
- `clamp`
- `vacum`

The old `hardware_pkg` is kept only for reference and rollback. New integration should use `stm32_bridge_pkg`.

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

## Main Topics

Debug:

- `/stm32/command`
- `/stm32/raw_tx`
- `/stm32/raw_rx`
- `/stm32/error`

Motor:

- `/mtor1/move`
- `/mtor2/move`
- `/mtor1/stop`
- `/mtor2/stop`
- `/mtor1/status_query`
- `/mtor2/status_query`

Clamp:

- `/clamp/move_percent`
- `/clamp/status_query`
- `/clamp/state`
- `/clamp/status_text`

Vacuum:

- `/vacuum/set_params`
- `/vacuum/grip`
- `/vacuum/release`
- `/vacuum/stop`
- `/vacuum/status_query`
- `/vacuum/status_text`

Emergency stop:

- `/emergency_stop`

## Notes

- STM32 serial protocol details are documented in `ROS接入STM32控制方案.md`.
- Code change notes are documented in `修改代码说明.md`.
- This package has passed Python syntax checking in the current environment.
- ROS2 `colcon` build was not verified on this Windows machine because `colcon` is not installed here.
