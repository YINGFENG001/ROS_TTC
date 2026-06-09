# ROS_TTC

这是 TTC 末端设备控制 ROS 2 工作区。当前主控链路已经切换为 `stm32_bridge_pkg`：ROS 只通过一根串口和 STM32 通信，由 STM32 统一控制底层末端设备。

## 当前控制链路

```text
ROS 2 / joy_pkg / 上层任务节点
  -> stm32_bridge_pkg
  -> STM32 USART1 115200 8N1
  -> mtor1 / mtor2 / clamp / vacum
```

STM32 负责控制：

- `mtor1`：1:1 直驱步进电机。
- `mtor2`：1:20 减速步进电机。
- `clamp`：串口舵机夹爪。
- `vacum`：EVS08 真空吸盘。

旧 `hardware_pkg` 仅保留作参考和回退，不再作为当前推荐控制入口。

## 构建

```bash
cd Auto_control_ws
colcon build --packages-select stm32_bridge_pkg
source install/setup.bash
```

当前 Windows 环境已做 Python 语法检查；ROS 2 `colcon build` 需要后续在 Linux/ROS 环境中验证。

## 运行

```bash
ros2 run stm32_bridge_pkg stm32_bridge_node --ros-args \
  -p stm32.port:=/dev/ttyUSB0 \
  -p stm32.baudrate:=115200
```

常用参数：

```text
stm32.port                 /dev/ttyUSB0
stm32.baudrate             115200
stm32.timeout              1.0
command.default_timeout    2.0
motion.default_timeout     10.0
legacy.enable              true
legacy.motor_device        mtor1
legacy.motor_max_rpm       1500
legacy.clamp_wait_done     false
```

`legacy.motor_device` 默认是 `mtor1`，因为旧遥控器电机逻辑适合 1:1 直驱轴；`mtor2` 带 1:20 减速，不建议直接复用旧 `/motor/jog_cmd` 调速逻辑。

## STM32 协议

ROS 发送给 STM32 的命令格式：

```text
#<id> <device> <cmd> [args...]
```

STM32 返回的机器可解析行：

```text
@ack id=<id> dev=<dev> cmd=<cmd> result=ok ...
@done id=<id> dev=<dev> cmd=<cmd> result=ok ...
@state id=<id> dev=<dev> result=ok ...
@err id=<id> dev=<dev> cmd=<cmd> code=<code> [detail=...]
@event level=info|warn|fault dev=<dev> event=... ...
```

`@event` 是异步事件，通常不带 `id=`。ROS 侧会发布到 `/stm32/event`；`level=warn/fault` 也会发布到 `/stm32/error`。

## 调试 Topic

| Topic | 类型 | 说明 |
|---|---|---|
| `/stm32/command` | `std_msgs/msg/String` | 直接发送不带 `#id` 的 STM32 命令体，例如 `mtor1 status` |
| `/stm32/raw_tx` | `std_msgs/msg/String` | 实际发给 STM32 的完整串口行 |
| `/stm32/raw_rx` | `std_msgs/msg/String` | STM32 返回的协议行 |
| `/stm32/error` | `std_msgs/msg/String` | 超时、`@err`、`@event level=warn/fault` 等错误/告警 |
| `/stm32/event` | `std_msgs/msg/String` | STM32 后台异步事件 |

示例：

```bash
ros2 topic pub --once /stm32/command std_msgs/msg/String "{data: 'mtor1 status'}"
ros2 topic echo /stm32/raw_rx
ros2 topic echo /stm32/event
```

## 电机 Topic

### 定长移动

```text
/mtor1/move    geometry_msgs/msg/Vector3
/mtor2/move    geometry_msgs/msg/Vector3
```

字段：

```text
x = 输出轴圈数，ROS 内部换算为 0.1 圈单位
y = 当前忽略
z = 当前忽略
```

映射：

```text
/mtor1/move x=5.0 -> mtor1 move 50
/mtor2/move x=-2.0 -> mtor2 move -20
```

示例：

```bash
ros2 topic pub --once /mtor1/move geometry_msgs/msg/Vector3 "{x: 5.0, y: 0.0, z: 0.0}"
```

### 连续运行调速

```text
/mtor1/rpm    std_msgs/msg/Int32
/mtor2/rpm    std_msgs/msg/Int32
```

映射：

```text
/mtor1/rpm data=100  -> mtor1 rpm 100
/mtor1/rpm data=-50  -> mtor1 rpm -50
/mtor1/rpm data=0    -> mtor1 rpm 0
```

示例：

```bash
ros2 topic pub --once /mtor1/rpm std_msgs/msg/Int32 "{data: 100}"
```

### 参数设置

```text
/mtor1/set_params    std_msgs/msg/String
/mtor2/set_params    std_msgs/msg/String
```

字符串格式：

```text
"<accel> <decel> <gear> <micro>"
```

映射：

```text
/mtor1/set_params "400 400 1:1 4" -> mtor1 set 400 400 1:1 4
```

示例：

```bash
ros2 topic pub --once /mtor1/set_params std_msgs/msg/String "{data: '400 400 1:1 4'}"
```

### 停止和状态

```text
/mtor1/stop            std_msgs/msg/Empty
/mtor2/stop            std_msgs/msg/Empty
/mtor1/status_query    std_msgs/msg/Empty
/mtor2/status_query    std_msgs/msg/Empty
```

状态发布：

```text
/mtor1/state    sensor_msgs/msg/JointState
/mtor2/state    sensor_msgs/msg/JointState
```

## 夹爪 Topic

### 百分比移动

```text
/clamp/move_percent    geometry_msgs/msg/Vector3
```

字段：

```text
x = 夹爪开度百分比，0~100，100 表示最大张开
y = 当前忽略
z = 1 等待 @done，0 只等待 @ack
```

映射：

```text
/clamp/move_percent x=75.0 -> clamp move 75.0%
```

示例：

```bash
ros2 topic pub --once /clamp/move_percent geometry_msgs/msg/Vector3 "{x: 75.0, y: 0.0, z: 1.0}"
```

状态查询和发布：

```text
/clamp/status_query    std_msgs/msg/Empty
/clamp/state           sensor_msgs/msg/JointState
/clamp/status_text     std_msgs/msg/String
```

`/clamp/state.position[0]` 发布开度百分比。当前代码兼容 STM32 返回中的 `openPos/Pct = 700 (100.0%)` 文本格式。

## 真空吸盘 Topic

```text
/vacuum/set_params      geometry_msgs/msg/Vector3
/vacuum/grip            std_msgs/msg/Empty
/vacuum/release         std_msgs/msg/Empty
/vacuum/stop            std_msgs/msg/Empty
/vacuum/status_query    std_msgs/msg/Empty
/vacuum/status_text     std_msgs/msg/String
```

`/vacuum/set_params` 字段：

```text
x = min_vac，0~100
y = max_vac，0~100
z = timeout，单位 100ms，范围 1~255
```

内部仍发送 STM32 当前设备名 `vacum`。

示例：

```bash
ros2 topic pub --once /vacuum/set_params geometry_msgs/msg/Vector3 "{x: 30.0, y: 70.0, z: 10.0}"
ros2 topic pub --once /vacuum/grip std_msgs/msg/Empty "{}"
```

## 旧 joy_pkg 兼容 Topic

为兼容当前 `joy_pkg/joy2robot.py`，`stm32_bridge_pkg` 在 `legacy.enable=true` 时订阅旧 `hardware_pkg` 风格 topic：

| Topic | 类型 | 映射 |
|---|---|---|
| `/motor/jog_cmd` | `geometry_msgs/msg/Vector3` | 未连续运行时先发 `mtor1 move +0/-0`，再发 `mtor1 rpm <signed_rpm>` |
| `/motor/stop` | `std_msgs/msg/Empty` | `mtor1 stop` |
| `/gripper/cmd_percent` | `geometry_msgs/msg/Vector3` | `clamp move <x>%` |

`/motor/enable` 已删除，不再使用。

当前 `joy2robot.py` 的电机控制：

- X 键：启动/停止 toggle。
- TL：速度增加 `20rpm`。
- TR：速度减少 `20rpm`。
- 速度范围限制为 `-1500~1500rpm`，允许跨过 0 后反向。
- 再次按 X 启动时会恢复上次停止时的速度和方向。

当前 `joy2robot.py` 的夹爪控制：

- 十字键左/右：调整开度百分比，发布 `/gripper/cmd_percent`。
- 十字键上：发布 `/stm32/command`，命令为 `clamp release`。
- 十字键下：发布 `/stm32/command`，命令为 `clamp grip 300`。

## 急停

```text
/emergency_stop    std_msgs/msg/Empty
```

当前实现会依次发送：

```text
mtor1 stop
mtor2 stop
clamp release
vacum stop
```

注意：当前尚未实现高优先级急停发送路径，仍使用普通串口命令等待流程。

## 说明

- `stm32_bridge_pkg/README.md` 是该包的简版使用说明。
- STM32 最新串口协议以 `D:/XGKJproject/stm32f4/DEVELOPING/Doc/串口命令说明.md` 为准。
- `ROS接入STM32控制方案.md` 是早期方案文档，部分命令格式已经过时，仅作历史参考。
- 本工作区 launch 仍需后续在 Linux/ROS 环境中按实际启动方式调整。
