# ROS接入STM32控制方案

## 背景

新的控制链路中，Linux / ROS 不再直接连接夹爪、电机驱动器或真空吸盘，而是只通过一根串口连接 STM32。STM32 作为所有末端设备的统一控制器，负责执行底层设备协议和实时控制。

目标链路：

```text
ROS 2 node
  -> STM32 serial protocol
  -> Linux serial /dev/ttyUSBx or /dev/ttyACMx
  -> STM32 USART1, 115200 8N1
  -> mtor1 / mtor2 / clamp / vacum
```

本方案建议新建 ROS package，不复用旧 `hardware_pkg` 的电机脉冲语义。新 package 直接面向 STM32 当前命令体系设计。

## 已确认约束

1. 所有末端设备都通过 STM32 控制，包括 `mtor1`、`mtor2`、`clamp`、`vacum`。
2. 电机 ROS 接口可以改成“圈”语义，不需要兼容旧 ROS 文件里的脉冲绝对位置。
3. 夹爪百分比定义为夹爪中间空隙大小比例：
   - `100%`：完全张开
   - `0%`：完全夹紧
4. ROS 到 STM32 串口参数固定为 `115200 8N1`。
5. 当前阶段所有设备状态先手动读取，不做周期自动轮询。
6. 电机运动使用相对运动，不要求 STM32 提供绝对位置模式。
7. 电机圈数统一为设备输出轴圈数。
8. 两个电机不要求同步启动或同步完成，分别下发控制命令。
9. 夹爪 ROS 侧主要提供百分比移动接口 `/clamp/move_percent`。
10. 真空吸盘 `grip` / `release` / `stop` 只等待 STM32 下发确认，不等待吸附成功。
11. ROS 对外使用标准英文 `/vacuum/...`，内部仍发送 STM32 当前设备名 `vacum`。
12. ROS 侧提供紧急停机命令，顺序停止 `mtor1`、`mtor2`、`clamp`、`vacum`。

## 推荐新package

建议新建 package：

```text
stm32_bridge_pkg
```

建议文件结构：

```text
stm32_bridge_pkg/
  package.xml
  setup.py
  setup.cfg
  resource/stm32_bridge_pkg
  stm32_bridge_pkg/
    __init__.py
    stm32_serial_client.py
    stm32_bridge_node.py
    protocol.py
```

职责划分：

| 文件 | 职责 |
|---|---|
| `protocol.py` | 解析 STM32 `@ack/@done/@state/@err` 行，格式化 `#<id>` 命令 |
| `stm32_serial_client.py` | 串口打开、读线程、写锁、命令 id、等待响应、超时处理 |
| `stm32_bridge_node.py` | ROS topic/service 与 STM32 命令之间的映射 |

不建议继续让 ROS 侧保留旧的 `motor_ctl.py` 或 `gripper_ctl.py`。这些旧文件直接操作硬件底层协议，会和 STM32 的统一控制权冲突。

## STM32串口协议假设

ROS 侧按以下机器协议设计：

输入：

```text
#<id> <device> <action> [args...]
```

输出：

```text
@ack id=<id> dev=<device> cmd=<action> result=<result> [fields...]
@done id=<id> dev=<device> cmd=<action> result=<result> [fields...]
@state id=<id> dev=<device> result=<result> [fields...]
@err id=<id> dev=<device> cmd=<action> code=<code> [detail=<detail>]
```

ROS 侧只解析 `@` 开头的行，忽略 STM32 启动提示、人工调试日志和帮助文本。

## 串口客户端设计

`Stm32SerialClient` 建议提供以下能力：

```python
send_command("mtor1 move 10 100 100 100", wait_for="ack", timeout=1.0)
send_command("mtor1 move 10 100 100 100", wait_for="done", timeout=10.0)
send_command("clamp status", wait_for="state", timeout=2.0)
send_command("vacum grip", wait_for="ack", timeout=2.0)
```

内部行为：

1. 使用递增命令 `id`。
2. 发送实际串口行：

```text
#12 clamp move 1200 500
```

3. 后台读线程持续读取串口行。
4. 将 `@` 协议行解析为字典：

```python
{
  "type": "state",
  "id": "12",
  "dev": "clamp",
  "result": "ok",
  "pos": "1200"
}
```

5. 按 `id` 分发给对应 pending command。
6. 如果收到 `@err`，立即返回失败。
7. 如果超时未收到期望类型，返回超时错误。

注意：

- 同一根串口建议同一时刻只允许一个命令等待结果，先用串行化队列实现，降低联调复杂度。
- 后续如果需要并发命令，再基于 `id` 放开。
- 所有写串口动作必须加锁。

## ROS接口建议

当前阶段推荐先使用 topic 做命令触发，状态使用手动查询 topic。后续如果机械臂任务流程需要严格结果反馈，再升级为 service/action。

### 公共调试接口

建议提供：

```text
/stm32/raw_tx       std_msgs/msg/String
/stm32/raw_rx       std_msgs/msg/String
/stm32/command      std_msgs/msg/String
/emergency_stop     std_msgs/msg/Empty
```

说明：

| Topic | 说明 |
|---|---|
| `/stm32/raw_tx` | 发布实际发给 STM32 的命令行，便于调试 |
| `/stm32/raw_rx` | 发布 STM32 返回的原始 `@` 协议行 |
| `/stm32/command` | 允许直接发送不带 `#id` 的设备命令，例如 `vacum status`，节点自动补 id |
| `/emergency_stop` | 紧急停机入口，顺序停止所有末端设备 |

`/stm32/command` 适合现场联调：

```bash
ros2 topic pub --once /stm32/command std_msgs/msg/String "{data: 'clamp status'}"
```

紧急停机收到 `/emergency_stop` 后，ROS 节点按固定顺序发送：

```text
mtor1 stop
mtor2 stop
clamp release
vacum stop
```

说明：

- `mtor1 stop` / `mtor2 stop`：停止两路步进。
- `clamp release`：夹爪按 STM32 当前释放流程卸力并退开。
- `vacum stop`：停止 EVS08 当前动作。

### 步进电机接口

因为新功能不需要兼容旧脉冲语义，ROS 对外使用设备输出轴“圈”，内部换算为 STM32 当前单位 `0.1圈`。所有电机运动均为相对运动。

#### 电机运动

```text
/mtor1/move    geometry_msgs/msg/Vector3
/mtor2/move    geometry_msgs/msg/Vector3
```

字段：

| 字段 | 含义 |
|---|---|
| `x` | 目标位移，单位：圈 |
| `y` | 速度，单位：rpm |
| `z` | 加减速，单位：rpm/s |

ROS 内部换算：

```text
rev_0p1 = round(x * 10)
accel = round(z)
decel = round(z)
rpm = round(y)
```

发送给 STM32：

```text
mtor1 move <rev_0p1> <accel> <decel> <rpm>
```

示例：

```text
ROS:  x=5.0, y=100, z=100
STM32: mtor1 move 50 100 100 100
```

#### 电机停止

```text
/mtor1/stop    std_msgs/msg/Empty
/mtor2/stop    std_msgs/msg/Empty
```

发送：

```text
mtor1 stop
mtor2 stop
```

#### 电机状态手动查询

```text
/mtor1/status_query    std_msgs/msg/Empty
/mtor2/status_query    std_msgs/msg/Empty
```

发送：

```text
mtor1 status
mtor2 status
```

状态发布：

```text
/mtor1/state    sensor_msgs/msg/JointState
/mtor2/state    sensor_msgs/msg/JointState
```

建议字段：

| JointState字段 | 含义 |
|---|---|
| `name[0]` | `mtor1` 或 `mtor2` |
| `position[0]` | 当前圈数，单位：圈 |
| `velocity[0]` | 当前速度，单位：rpm |
| `effort[0]` | 错误码或故障标志，`0` 表示正常 |

如果 STM32 返回 `rev=12`，ROS 发布 `position=1.2` 圈。

### 夹爪接口

夹爪百分比按“空隙大小”定义：

```text
100% = 完全张开
0%   = 完全夹紧
```

STM32 当前标定：

```text
open_position  = 800
close_position = 2048
```

注意：位置原始值越小越张开，越大越夹紧。因此 ROS 百分比到原始位置的推荐换算为：

```text
position = close_position - percent / 100.0 * (close_position - open_position)
```

示例：

```text
percent=100 -> position=800
percent=0   -> position=2048
percent=50  -> position=1424
```

建议参数化：

```text
gripper.open_position       默认 800
gripper.close_position      默认 2048
gripper.min_speed           默认 1
gripper.max_speed           默认 3000
gripper.default_speed       默认 800
```

#### 夹爪按百分比移动

```text
/clamp/move_percent    geometry_msgs/msg/Vector3
```

字段：

| 字段 | 含义 |
|---|---|
| `x` | 空隙百分比，`0~100` |
| `y` | 速度百分比，`0~100` |
| `z` | 是否等待完成，`1` 等待 `@done`，`0` 只等待 `@ack` |

速度换算：

```text
speed = min_speed + speed_percent / 100.0 * (max_speed - min_speed)
```

发送：

```text
clamp move <position> <speed>
```

当前 ROS 侧主控制接口只暴露 `/clamp/move_percent`。STM32 已支持的 `clamp open/close/grip/release` 暂不单独设计 ROS topic；如现场调试需要，可通过 `/stm32/command` 直接发送原始命令。

#### 夹爪状态手动查询

```text
/clamp/status_query    std_msgs/msg/Empty
```

发送：

```text
clamp status
```

状态发布：

```text
/clamp/state    sensor_msgs/msg/JointState
```

建议字段：

| JointState字段 | 含义 |
|---|---|
| `name[0]` | `clamp` |
| `position[0]` | 空隙百分比，`0~100` |
| `velocity[0]` | 速度原始值或 `0` |
| `effort[0]` | 电流或负载，可先用 STM32 返回的 `current` |

同时建议发布文本状态：

```text
/clamp/status_text    std_msgs/msg/String
```

用于保留 `state`、`voltage`、`temp`、`load` 等非 JointState 字段。

### 真空吸盘接口

STM32 侧设备名当前为 `vacum`，ROS 对外使用标准英文 `/vacuum/...`。节点内部将 `/vacuum` 命令映射为 STM32 串口命令 `vacum`。

#### 参数设置

```text
/vacuum/set_params    geometry_msgs/msg/Vector3
```

字段：

| 字段 | 含义 |
|---|---|
| `x` | `min_vac`，最小保持真空度，`0~100%` |
| `y` | `max_vac`，最大真空度，`0~100%` |
| `z` | `timeout`，单位 `100ms`，范围 `1~255` |

发送：

```text
vacum set <min_vac> <max_vac> <timeout>
```

#### 动作命令

```text
/vacuum/grip       std_msgs/msg/Empty
/vacuum/release    std_msgs/msg/Empty
/vacuum/stop       std_msgs/msg/Empty
```

发送：

```text
vacum grip
vacum release
vacum stop
```

#### 状态手动查询

```text
/vacuum/status_query    std_msgs/msg/Empty
```

发送：

```text
vacum status
```

状态发布：

```text
/vacuum/status_text    std_msgs/msg/String
```

建议初期直接发布解析后的 JSON 字符串，例如：

```json
{"busy1":0,"busy2":0,"obj1":1,"obj2":1,"vac1":68,"vac2":70,"fault":"0x0000","temp":31,"bus":242}
```

如果后续需要更规范的数据接口，可再增加自定义 message。

## 节点参数建议

```text
stm32.port                 /dev/ttyUSB0
stm32.baudrate             115200
stm32.timeout              1.0
command.default_timeout    2.0
motion.default_timeout     10.0

gripper.open_position      800
gripper.close_position     2048
gripper.min_speed          1
gripper.max_speed          3000
gripper.default_speed      800

motor.default_accel        100
motor.default_decel        100
motor.default_rpm          100
```

状态手动读取阶段，不需要 `state_pub_period`。如果保留，也只用于内部心跳或调试，不自动发送 `status`。

## 命令等待策略

不同命令建议等待不同响应：

| 命令 | 等待类型 | 说明 |
|---|---|---|
| `mtor move` | `ack` | 电机为相对运动，topic 模式只确认命令下发成功 |
| `mtor stop` | `ack` | 停止命令应快速返回 |
| `mtor status` | `state` | 手动查询 |
| `clamp move` | 根据消息字段等待 `ack` 或 `done` | `/clamp/move_percent` 的 `z=1` 等待完成，`z=0` 只等下发 |
| `clamp status` | `state` | 手动查询 |
| `vacum set/grip/release/stop` | `ack` | 当前阶段动作命令只确认下发成功 |
| `vacum status` | `state` | 手动查询 |
| `emergency_stop` | `ack` | 顺序发送多个停止/释放命令，每条只等待下发确认 |

如果 STM32 当前夹爪动作是阻塞执行，那么 `clamp move` 可能直接返回 `@done` 而不是先 `@ack`。ROS 串口客户端需要兼容：等待 `ack` 时收到同 id 的 `done` 也视为成功。

## 错误处理建议

ROS 收到：

```text
@err id=12 dev=clamp cmd=move code=range_error detail=position
```

节点行为：

1. 在日志中打印 warning/error。
2. 发布到：

```text
/stm32/error    std_msgs/msg/String
```

3. 如果该命令来自 topic，不再重试。
4. 如果该命令来自 service/action，返回失败结果。

常见错误：

| code | ROS侧处理 |
|---|---|
| `param_error` | 检查 ROS 映射或调用方参数 |
| `range_error` | 检查参数范围、标定值 |
| `busy` | 上层可稍后重试 |
| `uart_error` | 检查 STM32 到底层设备通信 |
| `timeout` | 检查设备接线或响应 |
| `crc_error` | 检查 RS485 通信质量 |
| `unknown_device` / `unknown_action` | ROS 与 STM32 协议版本不匹配 |

## 推荐实施顺序

1. 新建 ROS package 和 `stm32_serial_client.py`。
2. 实现 `@` 协议解析、命令 id、串口读线程、raw tx/rx topic。
3. 实现 `/stm32/command`，先能从 ROS 直接发送 `clamp status`、`vacum status`、`mtor1 status`。
4. 实现 `status_query` 类接口：`/mtor1/status_query`、`/mtor2/status_query`、`/clamp/status_query`、`/vacuum/status_query`。
5. 实现夹爪 `/clamp/move_percent`，验证百分比到原始位置映射。
6. 实现吸盘 `/vacuum/set_params`、`/vacuum/grip`、`/vacuum/release`、`/vacuum/stop`。
7. 实现电机 `/mtor1/move`、`/mtor2/move`、`/mtor1/stop`、`/mtor2/stop`。
8. 实现 `/emergency_stop`，顺序发送 `mtor1 stop`、`mtor2 stop`、`clamp release`、`vacum stop`。
9. 现场联调稳定后，再根据上层任务需要决定是否把动作接口升级为 service/action。

## 最小联调流程

### 1. 串口连通

```bash
ros2 topic pub --once /stm32/command std_msgs/msg/String "{data: 'mtor1 status'}"
ros2 topic echo /stm32/raw_rx
```

期望看到：

```text
@state id=... dev=mtor1 result=ok ...
```

### 2. 夹爪状态

```bash
ros2 topic pub --once /clamp/status_query std_msgs/msg/Empty "{}"
ros2 topic echo /clamp/state
```

### 3. 夹爪百分比移动

```bash
ros2 topic pub --once /clamp/move_percent geometry_msgs/msg/Vector3 "{x: 100.0, y: 30.0, z: 1.0}"
```

预期发送到 STM32：

```text
clamp move 800 <speed>
```

### 4. 真空吸盘

```bash
ros2 topic pub --once /vacuum/set_params geometry_msgs/msg/Vector3 "{x: 30.0, y: 70.0, z: 10.0}"
ros2 topic pub --once /vacuum/grip std_msgs/msg/Empty "{}"
ros2 topic pub --once /vacuum/status_query std_msgs/msg/Empty "{}"
```

### 5. 电机运动

```bash
ros2 topic pub --once /mtor1/move geometry_msgs/msg/Vector3 "{x: 5.0, y: 100.0, z: 100.0}"
```

预期发送到 STM32：

```text
mtor1 move 50 100 100 100
```

### 6. 紧急停机

```bash
ros2 topic pub --once /emergency_stop std_msgs/msg/Empty "{}"
```

预期依次发送到 STM32：

```text
mtor1 stop
mtor2 stop
clamp release
vacum stop
```
