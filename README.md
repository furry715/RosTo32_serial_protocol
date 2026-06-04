# 🚁 RosTo32_serial_protocol

> ROS 2 底层串口协议转换节点 —— 专为 **凌霄四轴飞行器（STM32 主控）** 设计

[ROS2](https://docs.ros.org/en/humble/)
[License: MIT](https://opensource.org/licenses/MIT)
[Platform]()

---

## 📖 目录

- [概述](#概述)
- [通信接口](#通信接口)
  - [🔹 速度控制话题（优先级 1）](#-速度控制话题优先级-1)
  - [🔸 降落服务（优先级 2）](#-降落服务优先级-2)
  - [🔸 解锁服务（优先级 3）](#-解锁服务优先级-3)
  - [🔸 锁定服务（优先级 4）](#-锁定服务优先级-4)
  - [🔴 紧急停机服务（优先级 5 - 最高）](#-紧急停机服务优先级-5---最高)

---

## 概述

`RosTo32_serial_protocol` 是 ROS 2 节点，负责将上层控制指令（如速度指令、飞行模式切换、安全指令）转换成飞行控制板（STM32）可识别的串口数据帧，并通过 UART 下发。  
节点内部实现优先级仲裁，确保**紧急停机**等安全指令能够优先发送。

---

## 通信接口

> **优先级策略**：数字越高越优先抢占串口发送权


| 优先级 | 接口类型 | 名称                                | 用途   |
| --- | ---- | --------------------------------- | ---- |
| 1   | 话题   | `/cmd_vel`                        | 速度控制 |
| 2   | 服务   | `/serial_protocol_node/land`      | 降落   |
| 3   | 服务   | `/serial_protocol_node/arm`       | 解锁电机 |
| 4   | 服务   | `/serial_protocol_node/disarm`    | 锁定电机 |
| 5   | 服务   | `/serial_protocol_node/emergency` | 紧急停机 |


---

### 🔹 速度控制话题（优先级 1）

**话题**：`/cmd_vel`  
**消息类型**：`geometry_msgs/msg/Twist`  

```bash
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.01}, angular: {z: 0.0}}"
```

linear.x – 前后方向速度 (m/s)

linear.y – 左右方向速度 (m/s)

linear.z – 升降速度 (m/s)

angular.z – 自旋角速度 (rad/s)  


### 🔸 降落服务（优先级 2）

**服务**：`/serial_protocol_node/land`  
**类型**：`std_srvs/srv/Trigger`
调用 /serial_protocol_node/land 服务（std_srvs/srv/Trigger）以触发降落指令。

```bash
ros2 service call /serial_protocol_node/land std_srvs/srv/Trigger
```

### 🔸 解锁服务（优先级 3）

**服务**：`/serial_protocol_node/arm`  
**类型**：`std_srvs/srv/Trigger`
调用 /serial_protocol_node/arm 服务（std_srvs/srv/Trigger）以解锁电机。

```bash
ros2 service call /serial_protocol_node/arm std_srvs/srv/Trigger
```

### 🔸 锁定服务（优先级 4）

**服务**：`/serial_protocol_node/disarm`  
**类型**：`std_srvs/srv/Trigger`
调用 /serial_protocol_node/disarm 服务（std_srvs/srv/Trigger）以锁定电机。

```bash
ros2 service call /serial_protocol_node/disarm std_srvs/srv/Trigger
```

### 🔴 紧急停机服务（优先级 5 - 最高）

**服务**：`/serial_protocol_node/emergency`  
**类型**：`std_srvs/srv/Trigger`
调用 /serial_protocol_node/emergency 服务（std_srvs/srv/Trigger）将立刻切断电机输出，不可恢复（需重新上锁/解锁）。

```bash
ros2 service call /serial_protocol_node/emergency std_srvs/srv/Trigger
```

⚠️ 仅限紧急情况使用，调用后飞行器会立即停止所有电机。
(新版本添加关于360保护代码，但暂不启用)