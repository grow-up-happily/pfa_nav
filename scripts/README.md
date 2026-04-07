# Scripts README

这个目录里的脚本主要是给 ROS 2 / Gazebo / RViz 排障用的。
可以把它们理解成 4 个小工具：

- 查是不是可视化把机器拖慢了
- 清理没退干净的仿真环境
- 查激光为什么没变成地图更新
- 查哪个图形窗口最吃资源

## 使用前

当前目录：

```bash
cd /home/lcy/sight_test/pfa-nav/scripts
```

如果要运行两个 ROS 2 相关的 Python 脚本，先把 ROS 环境 source 好：

```bash
source /opt/ros/<ros_distro>/setup.bash
source <your_ws>/install/setup.bash
```

想先看所有参数，可以直接：

```bash
./diagnose_viz_cpu.py --help
./diagnose_slam_scan_gates.py --help
./diagnose_x11_clients.py --help
./cleanup_ros2_env.sh --help
```

## 1. diagnose_viz_cpu.py

用途：

查“是不是 RViz / 点云 / 激光 / 地图显示把 CPU 吃高了”。

它会同时看两件事：

- 哪些 ROS topic 很忙，频率和数据量有多大
- 哪些进程和线程最吃 CPU，比如 `rviz2`、Gazebo、Nav2、SLAM

最后会给一个总结，帮助判断卡顿更像是谁造成的。

常用启动命令：

```bash
./diagnose_viz_cpu.py
```

指定采样时间和间隔：

```bash
./diagnose_viz_cpu.py --duration 30 --interval 1
```

如果你的机器人有命名空间：

```bash
./diagnose_viz_cpu.py --namespace red_standard_robot1
```

想额外盯某些 topic：

```bash
./diagnose_viz_cpu.py --extra-topic /your/topic --extra-topic /another/topic
```

适合什么时候用：

- 打开 RViz 后机器明显变卡
- 不确定是 RViz、Gazebo 还是导航节点在吃 CPU
- 想确认是不是某个点云/激光显示项太重

## 2. cleanup_ros2_env.sh

用途：

清理 ROS 2 / Gazebo / FastDDS 的“残局”。

它会做几件事：

- 杀掉常见的 ROS / Gazebo 相关残留进程
- 停掉 `ros2 daemon`
- 清掉 `/dev/shm` 里 FastDDS 共享内存残留

常用启动命令：

先看它准备清什么，不真的动手：

```bash
./cleanup_ros2_env.sh --dry-run
```

直接清理：

```bash
./cleanup_ros2_env.sh
```

只杀进程，不动其他东西：

```bash
./cleanup_ros2_env.sh --no-daemon --no-shm
```

适合什么时候用：

- 上一次仿真没退干净
- 重新启动后发现节点、DDS、Gazebo 状态不对
- 怀疑有后台残留进程在占资源或占端口

## 3. diagnose_slam_scan_gates.py

用途：

查“为什么 `obstacle_scan` 收到了，但地图就是不更新”。

这个脚本会顺着链路一层层排查：

- TF 对不对，时间戳能不能对上
- scan 有没有被节流、最小时间间隔、最小位移这些门槛挡掉
- scan 本身是不是不合法
- 就算 scan 进去了，里面的 beam 有没有真的打到有效障碍

最后它会告诉你：最像是卡在 TF、外层过滤、SLAM 内部过滤，还是数据本身没效果。

常用启动命令：

按默认话题直接查：

```bash
./diagnose_slam_scan_gates.py
```

手动指定 scan、map 和 slam 节点：

```bash
./diagnose_slam_scan_gates.py \
  --scan-topic obstacle_scan \
  --map-topic map \
  --slam-node /slam_toolbox
```

每 5 秒打印一次报告，持续 30 秒：

```bash
./diagnose_slam_scan_gates.py --report-period 5 --duration 30
```

如果 TF 话题不是默认推导出来的，也可以手动指定：

```bash
./diagnose_slam_scan_gates.py \
  --tf-topic /tf \
  --tf-static-topic /tf_static
```

适合什么时候用：

- `obstacle_scan` 有数据，但 `map` 几乎不变
- 怀疑 TF、时间戳、节流参数有问题
- 想知道 scan 是在哪一层被过滤掉的

## 4. diagnose_x11_clients.py

用途：

查“到底是哪个图形窗口最吃资源”。

它会把这些信息串起来看：

- `xrestop` 看到的 X11 client 内存 / pixmap 占用
- 窗口标题、窗口类名、PID
- CPU 使用率
- NVIDIA GPU 使用率

最后会列出峰值最高的几个图形客户端。

常用启动命令：

```bash
./diagnose_x11_clients.py
```

采样 20 次，每次间隔 1 秒：

```bash
./diagnose_x11_clients.py --samples 20 --interval 1
```

把结果写到文件：

```bash
./diagnose_x11_clients.py --samples 20 --interval 1 --output /tmp/x11_clients.log
```

适合什么时候用：

- 桌面卡顿，但不确定是哪个 GUI 程序导致的
- 想看 RViz、Gazebo GUI、浏览器之类谁占图形资源最多
- 想把 CPU/GPU/X11 内存占用关联起来看

## 快速选择

如果你遇到的是：

- “开 RViz 就卡” -> `./diagnose_viz_cpu.py`
- “环境脏了，想先清场” -> `./cleanup_ros2_env.sh`
- “scan 有了，但地图不更新” -> `./diagnose_slam_scan_gates.py`
- “不知道哪个窗口最耗资源” -> `./diagnose_x11_clients.py`
