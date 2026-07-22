# 自动任务 YAML 使用说明

本文说明如何通过 `mission_example.yaml` 编写自动任务，以及手动发目标点和自动任务之间的关系。

默认任务文件：

```bash
/home/f/linxiao_ws/src/RosTo32_serial_protocol/config/mission_example.yaml
```

启动自动任务：

```bash
source /home/f/linxiao_ws/install/setup.sh
ros2 launch serial_protocol linxiao_real_system.launch.py use_mission:=true
```

指定自己的任务文件：

```bash
source /home/f/linxiao_ws/install/setup.sh
ros2 launch serial_protocol linxiao_real_system.launch.py \
  use_mission:=true \
  mission_file:=/home/f/linxiao_ws/src/RosTo32_serial_protocol/config/mission_example.yaml
```

## 1. YAML 整体结构

任务文件分成两部分：

```yaml
mission:
  frame_id: camera_init
  auto_arm: true
  arm_idle_duration: 2.0
  auto_land: true
  auto_disarm: false
  default_reach_timeout: 30.0

waypoints:
  - name: takeoff
    pose: {x: 0.0, y: 0.0, z: 1.0, yaw_deg: 0.0}
    gates:
      - type: hold
        duration: 2.0
```

`mission` 是全局任务设置。

`waypoints` 是航点列表，无人机会按顺序执行。

## 2. mission 参数

`frame_id`

所有航点使用哪个坐标系。当前推荐：

```yaml
frame_id: camera_init
```

`auto_arm`

任务开始时是否自动调用解锁服务：

```bash
/serial_protocol_node/arm
```

如果是真机自动任务，通常设为：

```yaml
auto_arm: true
```

如果只是测试规划，不接串口，设为：

```yaml
auto_arm: false
```

`arm_idle_duration`

解锁成功后原地等待多久，再发送第一个航点。

```yaml
arm_idle_duration: 2.0
```

意思是 arm 后怠速等待 2 秒。

`auto_land`

所有航点完成后是否自动调用降落服务：

```bash
/serial_protocol_node/land
```

真机自动任务通常设为：

```yaml
auto_land: true
```

`auto_disarm`

降落后是否再调用上锁服务：

```bash
/serial_protocol_node/disarm
```

目前飞控 land 后会自动上锁，所以推荐：

```yaml
auto_disarm: false
```

`default_reach_timeout`

每个航点最多允许多久没到达。超过这个时间，任务进入 `ERROR`。

```yaml
default_reach_timeout: 30.0
```

## 3. 航点写法

一个航点长这样：

```yaml
- name: point_1
  pose: {x: 0.5, y: 0.0, z: 1.0, yaw_deg: 0.0}
  gates: []
```

`name`

航点名字，只用于日志和状态显示。

`pose`

目标位置和目标机头方向。

```yaml
pose: {x: 0.5, y: 0.0, z: 1.0, yaw_deg: 0.0}
```

含义：

- `x`: 前后方向，正值向前
- `y`: 左右方向，正值向左
- `z`: 高度
- `yaw_deg`: 目标机头角度，单位是度

`gates`

到达该航点后还要等待什么条件。

如果写：

```yaml
gates: []
```

表示到达后立刻执行下一个航点。

## 4. 起飞和降落流程

推荐写法：

```yaml
mission:
  auto_arm: true
  arm_idle_duration: 2.0
  auto_land: true
  auto_disarm: false

waypoints:
  - name: takeoff
    pose: {x: 0.0, y: 0.0, z: 1.0, yaw_deg: 0.0}
    gates:
      - type: hold
        duration: 2.0

  - name: landing_approach
    pose: {x: 0.0, y: 0.0, z: 0.5, yaw_deg: 0.0}
    gates:
      - type: hold
        duration: 2.0
```

实际执行顺序：

```text
1. 自动调用 arm
2. 怠速等待 arm_idle_duration 秒
3. 发布 takeoff 航点，让无人机飞到 z=1.0
4. 执行中间任务航点
5. 发布 landing_approach 航点，让无人机先下降到 z=0.5
6. 自动调用 land
7. 飞控 land 后自动上锁
```

注意：

`takeoff` 和 `landing_approach` 不是飞控命令，它们是位置目标点。

真正的命令来自：

- `auto_arm: true` 触发 `arm`
- `auto_land: true` 触发 `land`

## 5. hold gate

到达航点后悬停等待一段时间。

```yaml
gates:
  - type: hold
    duration: 3.0
```

表示到达后悬停 3 秒。

## 6. topic_bool gate

等待某个 Bool 话题达到指定值。

```yaml
gates:
  - type: topic_bool
    topic: /vision/task_done
    value: true
    timeout: 10.0
    on_timeout: error
```

测试时可以手动发布：

```bash
source /home/f/linxiao_ws/install/setup.sh
ros2 topic pub --once /vision/task_done std_msgs/msg/Bool "{data: true}"
```

## 7. topic_string gate

等待某个 String 话题等于指定字符串。

```yaml
gates:
  - type: topic_string
    topic: /vision/task_done
    value: inspect_front
    timeout: 10.0
    on_timeout: error
```

测试时可以手动发布：

```bash
source /home/f/linxiao_ws/install/setup.sh
ros2 topic pub --once /vision/task_done std_msgs/msg/String "{data: 'inspect_front'}"
```

## 8. timeout 和 on_timeout

`timeout` 是等待 gate 的最长时间，单位秒。

`on_timeout` 支持：

- `error`: 进入 ERROR，停止任务
- `skip`: 跳过当前 gate
- `next`: 直接去下一个航点
- `land`: 触发降落流程

推荐实机初期使用：

```yaml
on_timeout: error
```

## 9. 加一个新任务点

比如你想飞到前方 1 米，然后等待视觉识别完成：

```yaml
- name: inspect_front
  pose: {x: 1.0, y: 0.0, z: 1.0, yaw_deg: 0.0}
  gates:
    - type: hold
      duration: 1.0
    - type: topic_string
      topic: /vision/task_done
      value: inspect_front
      timeout: 15.0
      on_timeout: error
```

视觉节点完成后发布：

```bash
ros2 topic pub --once /vision/task_done std_msgs/msg/String "{data: 'inspect_front'}"
```

mission 节点收到后才会继续执行下一个航点。

## 10. 只到点就继续

如果某个航点不需要执行额外任务：

```yaml
- name: point_1
  pose: {x: 0.5, y: 0.0, z: 1.0, yaw_deg: 0.0}
  gates: []
```

无人机到达后直接去下一个航点。

## 11. 手动发目标点还能不能用

可以继续用。

手动发目标点：

```bash
source /home/f/linxiao_ws/install/setup.sh
ros2 topic pub --once /move_base_simple/goal geometry_msgs/msg/PoseStamped \
"{header: {frame_id: 'camera_init'}, pose: {position: {x: 1.0, y: 0.0, z: 1.0}, orientation: {w: 1.0}}}"
```

但是注意：

如果 mission 节点正在运行，它也会向 `/move_base_simple/goal` 发布目标点。

所以不要同时让两边抢目标。

推荐：

手动测试时：

```bash
ros2 launch serial_protocol linxiao_real_system.launch.py use_mission:=false
```

自动任务时：

```bash
ros2 launch serial_protocol linxiao_real_system.launch.py use_mission:=true
```

## 12. 常用查看命令

```bash
ros2 topic echo /mission/state
ros2 topic echo /mission/current_waypoint
ros2 topic echo /mission/gate_status
ros2 topic echo /mission/goal_reached
ros2 topic echo /cmd_vel
ros2 topic echo /move_base_simple/goal
```

