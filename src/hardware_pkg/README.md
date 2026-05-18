# hardware_pkg

> Deprecated: STM32 unified control now uses `stm32_bridge_pkg`.
> Do not use `hardware_node` for the new control chain, because it talks directly to the old motor and gripper hardware interfaces.

ROS 2 (rclpy) 硬件控制包：

- `motor_ctl.py`：CL57R 步进驱动器（Modbus RTU）控制
- `gripper_ctl.py`：夹爪串口控制（自定义帧 + 校验）
- `hardware_node.py`：ROS2 节点封装（订阅控制话题 + 发布状态话题）

> 本 README 说明如何通过命令行发布话题控制 motor / gripper，以及各话题字段含义。

---

## 1. 节点启动

可执行入口：`hardware_node = hardware_pkg.hardware_node:main`

### 1.1 参数

| 参数名 | 类型 | 默认值 | 说明 |
|---|---:|---:|---|
| `motor.port` | string | `/dev/ttyUSB0` | 电机驱动串口端口 |
| `motor.baudrate` | int | `38400` | 电机驱动波特率 |
| `motor.slave_id` | int | `1` | Modbus 从站 ID |
| `motor.timeout` | double | `0.1` | 串口读超时 (s) |
| `gripper.port` | string | `/dev/ttyUSB1` | 夹爪串口端口 |
| `gripper.baudrate` | int | `115200` | 夹爪波特率（请按实际硬件修改） |
| `gripper.timeout` | double | `1.0` | 串口读超时 (s) |
| `state_pub_period` | double | `0.1` | 状态发布周期 (s) |

### 1.2 启动示例

```bash
ros2 run hardware_pkg hardware_node \
  --ros-args \
  -p motor.port:=/dev/ttyUSB0 -p motor.baudrate:=38400 -p motor.slave_id:=1 \
  -p gripper.port:=/dev/ttyUSB1 -p gripper.baudrate:=1000000 \
  -p state_pub_period:=0.1
```

---

## 2. 话题接口总览

> `hardware_node.py` 内使用相对话题名（如 `motor/enable`）。在命令行中通常以 `/motor/enable` 访问。

### 2.1 Motor 订阅话题（控制）

| 话题 | 类型 | 说明 |
|---|---|---|
| `/motor/enable` | `std_msgs/msg/Bool` | 电机使能/释放 |
| `/motor/position_cmd` | `geometry_msgs/msg/Vector3` | 位置模式（定位运动） |
| `/motor/jog_cmd` | `geometry_msgs/msg/Vector3` | 点动 JOG |
| `/motor/stop` | `std_msgs/msg/Empty` | 停止 |
| `/motor/home` | `std_msgs/msg/Int32` | 回零 |

### 2.2 Gripper 订阅话题（控制）

| 话题 | 类型 | 说明 |
|---|---|---|
| `/gripper/cmd_percent` | `geometry_msgs/msg/Vector3` | 按百分比控制夹爪（并可选等待） |
| `/gripper/stop_monitoring` | `std_msgs/msg/Empty` | 停止夹爪监视线程 |

### 2.3 状态发布话题

| 话题 | 类型 | 说明 |
|---|---|---|
| `/motor/state` | `sensor_msgs/msg/JointState` | motor 的位置/速度/故障位 |
| `/motor/status_text` | `std_msgs/msg/String` | motor 状态字典（可读文本） |
| `/gripper/state` | `sensor_msgs/msg/JointState` | gripper 的位置/电流 |

---

## 3. Motor 控制指令详解（topic pub）

### 3.1 使能/释放：`/motor/enable`

消息类型：`std_msgs/msg/Bool`

| 字段 | 类型 | 含义 |
|---|---|---|
| `data` | bool | `true`=使能，`false`=释放 |

示例：

```bash
# 使能
ros2 topic pub --once /motor/enable std_msgs/msg/Bool "{data: true}"

# 释放
ros2 topic pub --once /motor/enable std_msgs/msg/Bool "{data: false}"
```

### 3.2 定位运动：`/motor/position_cmd`

消息类型：`geometry_msgs/msg/Vector3`

| 字段 | 类型 | 含义 |
|---|---|---|
| `x` | float | `target_position`：目标位置（单位：脉冲） |
| `y` | float | `speed`：运行速度（单位：r/min） |
| `z` | float | `is_absolute`：`1`绝对位置，`0`相对位置 |

示例：

```bash
# 绝对到 0 脉冲，速度 1000 rpm
ros2 topic pub --once /motor/position_cmd geometry_msgs/msg/Vector3 "{x: 0.0, y: 1000.0, z: 1.0}"

# 相对 +5000 脉冲，速度 300 rpm
ros2 topic pub --once /motor/position_cmd geometry_msgs/msg/Vector3 "{x: 5000.0, y: 300.0, z: 0.0}"
```

### 3.3 点动：`/motor/jog_cmd`

消息类型：`geometry_msgs/msg/Vector3`

| 字段 | 类型 | 含义 |
|---|---|---|
| `x` | float | `direction`：方向（`>=0` 正向，`<0` 反向） |
| `y` | float | `speed`：速度（单位：r/min，建议正数；代码会取绝对值） |
| `z` | float | `duration`：持续时间（单位：s；`<=0` 表示持续运行，需要手动 stop） |

示例：

```bash
# 正向 200 rpm，持续 2 秒，之后自动 stop
ros2 topic pub --once /motor/jog_cmd geometry_msgs/msg/Vector3 "{x: 1.0, y: 200.0, z: 2.0}"

# 反向 500 rpm，持续运行（需手动 stop）
ros2 topic pub --once /motor/jog_cmd geometry_msgs/msg/Vector3 "{x: -1.0, y: 500.0, z: 0.0}"
```

### 3.4 停止：`/motor/stop`

消息类型：`std_msgs/msg/Empty`

```bash
ros2 topic pub --once /motor/stop std_msgs/msg/Empty "{}"
```

### 3.5 回零：`/motor/home`

消息类型：`std_msgs/msg/Int32`

| 字段 | 类型 | 含义 |
|---|---|---|
| `data` | int32 | `home_mode`：回零方式（详见 `motor_ctl.py::home()` 注释，默认常用为 `24`） |

```bash
ros2 topic pub --once /motor/home std_msgs/msg/Int32 "{data: 24}"
```

---

## 4. Gripper 控制指令详解（topic pub）

### 4.1 百分比控制：`/gripper/cmd_percent`

消息类型：`geometry_msgs/msg/Vector3`

| 字段 | 类型 | 含义 |
|---|---|---|
| `x` | float | `position_percent`：目标开合百分比（0~100） |
| `y` | float | `speed_percent`：速度百分比（0~100） |
| `z` | float | `wait_for_completion`：是否等待动作完成（`1`等待，`0`不等待） |

> 说明：节点内部会调用 `GripperController.send_command_with_monitoring_percent()`。
> 该函数会启动监视线程，基于电流/位置稳定自动停止监视。

示例：

```bash
# 夹到 100%，速度 50%，等待完成
ros2 topic pub --once /gripper/cmd_percent geometry_msgs/msg/Vector3 "{x: 100.0, y: 50.0, z: 1.0}"

# 张开到 0%，速度 30%，不等待（后台线程执行）
ros2 topic pub --once /gripper/cmd_percent geometry_msgs/msg/Vector3 "{x: 0.0, y: 30.0, z: 0.0}"
```

### 4.2 停止夹爪监视：`/gripper/stop_monitoring`

消息类型：`std_msgs/msg/Empty`

```bash
ros2 topic pub --once /gripper/stop_monitoring std_msgs/msg/Empty "{}"
```

---

## 5. 状态查看（topic echo）

### 5.1 Motor 状态：`/motor/state`

类型：`sensor_msgs/msg/JointState`

| 字段 | 含义 |
|---|---|
| `name[0]` | 固定为 `motor` |
| `position[0]` | 当前位置（脉冲） |
| `velocity[0]` | 当前速度（r/min） |
| `effort[0]` | 故障标志（`1.0`=故障，`0.0`=无故障） |

```bash
ros2 topic echo /motor/state
```

### 5.2 Motor 可读状态：`/motor/status_text`

类型：`std_msgs/msg/String`，内容是 `motor_ctl.py::get_status()` 返回字典的字符串形式。

```bash
ros2 topic echo /motor/status_text
```

### 5.3 Gripper 状态：`/gripper/state`

类型：`sensor_msgs/msg/JointState`

| 字段 | 含义 |
|---|---|
| `name[0]` | 固定为 `gripper` |
| `position[0]` | 当前夹爪位置（原始数值，通常 1600~3200） |
| `effort[0]` | 当前电流（A） |

> 若读取失败，会发布 `NaN`。

```bash
ros2 topic echo /gripper/state
```

---

## 6. 常见问题

1) **串口端口不存在/权限不足**：
- 检查 `motor.port`/`gripper.port` 是否正确
- Linux 下确认用户是否有串口权限（如 dialout 组）

2) **夹爪波特率默认值**：
- `hardware_node.py` 默认 `gripper.baudrate=115200`，但 `gripper_ctl.py` 示例里常见 `1000000`。
- 请按实际硬件设置启动参数：`-p gripper.baudrate:=1000000`

3) **JOG 持续运行**：
- `/motor/jog_cmd` 的 `z<=0` 表示持续运行，需要手动发 `/motor/stop`。
