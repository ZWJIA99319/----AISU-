# ---全国大学生智能汽车竞赛-国赛-AISU队-
# 实现 B：函数式转换方案

这是功能等价的另一套 ROS 2 工作区源码。话题、路线点、控制周期、到点阈值、角度死区、误差滤波、角速度限幅和倒车动作均与输入工程保持一致。

## 结构

- `route_executor`：以不可变 `Snapshot` 表示任务状态；`transition()` 是无 ROS 副作用的纯函数。
- `vehicle_bridge`：帧编解码使用独立 `.hpp/.cpp`，ROS 节点只负责设备 I/O 与消息映射。
- `serial_ros2` 与 `底盘烧录`：硬件依赖和配套文件原样保留。

## 构建与运行

```bash
cd <本目录>
colcon build --symlink-install
source install/setup.bash
ros2 launch vehicle_bridge system.launch.py
ros2 launch route_executor execute_route.launch.py line_v:=1.0 line_kp:=-0.030 route_mode:=1
```

串口参数名称在本实现中为 `serial_device` 和 `serial_baud`：

```bash
ros2 run vehicle_bridge vehicle_bridge_node --ros-args -p serial_device:=/dev/ttyUSB0
```

首次实车运行请降低 `line_v`，确认 `/odom_combined`、IMU 方向和急停手段均正常。
