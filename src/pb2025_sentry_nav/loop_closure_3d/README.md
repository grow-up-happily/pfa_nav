# 3D Loop Closure Detection for Point-LIO

## 概述

这个包实现了一个基于 **small_gicp** 的3D回环检测节点，用于修正 Point-LIO 的累积漂移（包括Z轴漂移）。

### 功能特性

- ✅ **3D回环检测**：基于点云配准（GICP），检测Z轴方向的回环
- ✅ **关键帧管理**：智能选择关键帧，减少计算量
- ✅ **动态TF发布**：实时发布修正后的 `map→odom` 变换
- ✅ **可视化支持**：在RViz中显示关键帧和回环边

### 系统架构

```
Point-LIO (里程计)
  ├─ 发布: cloud_registered (点云)
  ├─ 发布: aft_mapped_to_init (里程计)
  └─ 发布: odom→base_footprint (TF)
          ↓
Loop Closure Node (本节点)
  ├─ 订阅: cloud_registered, aft_mapped_to_init
  ├─ 功能: 检测回环，计算修正量
  └─ 发布: map→odom (动态TF，修正漂移)
```

## 安装

已经集成到项目中，无需额外安装。确保已构建：

```bash
cd /home/lcy/sight_test/pfa-nav
colcon build --packages-select loop_closure_3d
source install/setup.bash
```

## 使用方法

### 方法1：使用build.sh（推荐）

```bash
cd /home/lcy/sight_test/pfa-nav
./build.sh
```

**说明**：
- `build.sh` 已自动集成 loop_closure_3d 节点
- 静态 `map→odom` 变换已被禁用
- 回环检测将自动运行

### 方法2：单独启动节点（调试用）

```bash
# 终端1：启动Point-LIO和其他节点
ros2 launch pb2025_nav_bringup slam_launch.py

# 终端2（可选）：单独启动回环检测（如果想单独调试）
ros2 launch loop_closure_3d loop_closure_launch.py
```

## 参数配置

编辑 `/home/lcy/sight_test/pfa-nav/src/pb2025_sentry_nav/loop_closure_3d/config/loop_closure_params.yaml`

### 关键帧选择参数

```yaml
keyframe_distance_threshold: 1.0      # 关键帧间最小距离（米）
keyframe_rotation_threshold: 0.3      # 关键帧间最小旋转（弧度，~17度）
```

**建议调整**：
- 场景较大：增大距离阈值（如 `2.0`）
- 场景复杂：减小距离阈值（如 `0.5`）

### 回环检测参数

```yaml
loop_search_radius: 10.0              # 回环搜索半径（米）
loop_closure_score_threshold: 0.5     # GICP误差阈值（越小越严格）
min_keyframes_for_loop: 10            # 最少关键帧数量
skip_recent_keyframes: 30             # 跳过最近N个关键帧
```

**建议调整**：
- 检测太少：增大 `loop_search_radius`（如 `15.0`），放宽 `score_threshold`（如 `0.8`）
- 误检太多：减小 `score_threshold`（如 `0.3`），增加 `skip_recent_keyframes`（如 `50`）

### 点云处理参数

```yaml
downsample_resolution: 0.3            # 下采样体素大小（米）
num_threads: 4                        # GICP线程数
```

## 监控和调试

### 1. 查看节点输出

```bash
# 查看loop_closure_3d节点日志
ros2 topic echo /rosout | grep loop_closure
```

**关键日志信息**：
```
[INFO] Added keyframe 10 at (1.23, 4.56, 0.10), total: 10
[INFO] Found 3 loop candidates for KF 25
[INFO] Loop closure detected! Current KF: 25, Matched KF: 5
[INFO] Updated map->odom: trans=(0.123, -0.045, 0.012), rpy=(0.001, -0.002, 0.015)
```

### 2. RViz可视化

启动RViz：
```bash
rviz2
```

添加显示：
1. **Fixed Frame**: 设置为 `map`
2. **MarkerArray**: 添加topic `/loop_closure_markers`
   - **绿色球体**：关键帧位置
   - **红色线段**：检测到的回环边

### 3. 检查TF树

```bash
# 查看TF树结构
ros2 run tf2_tools view_frames

# 实时监控map->odom变换
ros2 run tf2_ros tf2_echo map odom
```

### 4. 查看关键帧数量

```bash
# 统计关键帧数量（从日志中）
ros2 topic echo /rosout | grep "Added keyframe" | wc -l
```

## 测试步骤

### 测试1：基础功能验证

1. **启动系统**
   ```bash
   ./build.sh
   ```

2. **移动机器人**：
   - 让机器人走一个小回环路径（如绕一圈）
   - 建议路径：前进3米 → 右转90° → 前进3米 → 右转90° → 前进3米 → 右转90° → 前进3米

3. **检查日志**：
   - 应该看到 "Added keyframe" 消息（每隔1米左右）
   - 当回到起点附近时，应该看到 "Loop closure detected!" 消息

4. **验证修正效果**：
   ```bash
   # 在RViz中观察：
   # - 关键帧点（绿色球）应该形成闭合路径
   # - 红色线段连接起点和终点关键帧
   ```

### 测试2：Z轴漂移修正

1. **创建Z轴漂移场景**：
   - 让机器人上下坡，或在不平坦地面行驶
   - 记录初始高度

2. **走回环路径**：
   - 回到起点（相同高度）

3. **观察修正**：
   ```bash
   ros2 run tf2_ros tf2_echo map odom
   ```
   - 观察 `translation.z` 值
   - 检测到回环后，Z轴误差应该减小

### 测试3：长时间运行

1. **长时间建图**（10-15分钟）
2. **多次经过同一地点**
3. **检查**：
   - 日志中回环检测次数
   - RViz中红色回环边数量
   - 地图是否闭合良好

## 常见问题

### Q1: 没有检测到回环

**可能原因**：
- 关键帧不足（`< min_keyframes_for_loop`）
- 搜索半径太小
- 阈值太严格

**解决方案**：
```yaml
# 放宽参数
loop_search_radius: 15.0              # 增大搜索范围
loop_closure_score_threshold: 0.8     # 放宽阈值
skip_recent_keyframes: 20             # 减少跳过数量
```

### Q2: 检测到太多误检

**症状**：日志中频繁出现"Loop closure detected"，但实际没有回环

**解决方案**：
```yaml
# 收紧参数
loop_closure_score_threshold: 0.3     # 更严格的阈值
skip_recent_keyframes: 50             # 增加跳过数量
downsample_resolution: 0.2            # 更精细的点云
```

### Q3: TF发布频率过低

**检查**：
```bash
ros2 topic hz /tf
```

**解决方案**：
修改代码 `loop_closure_node.cpp:98`：
```cpp
tf_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(20),  // 改为20ms（50Hz）
    std::bind(&LoopClosureNode::publishTransform, this));
```

### Q4: 系统卡顿

**原因**：GICP计算量大

**解决方案**：
```yaml
# 优化参数
downsample_resolution: 0.5            # 更粗的下采样
keyframe_distance_threshold: 2.0      # 减少关键帧数量
num_threads: 8                        # 增加线程数（如果CPU允许）
```

## 性能监控

```bash
# CPU使用率
top -p $(pgrep -f loop_closure_node_exe)

# 内存使用
ps aux | grep loop_closure_node_exe

# 节点延迟
ros2 topic delay /loop_closure_markers
```

## 与LIO-SAM对比

| 维度 | Loop Closure 3D (方案1) | LIO-SAM (方案2) |
|------|------------------------|-----------------|
| 兼容性 | ✅ 完全兼容现有系统 | ❌ 需要大量修改 |
| 实现难度 | 简单（300行代码） | 复杂（需要配置和测试）|
| 回环质量 | GICP点云配准 | GTSAM全局优化（更好）|
| Prior PCD | ✅ 保留 | ❌ 需要额外处理 |
| 推荐场景 | 快速验证、小场景 | 大场景、长时间建图 |

## 文件清单

```
loop_closure_3d/
├── package.xml
├── CMakeLists.txt
├── README.md (本文件)
├── include/loop_closure_3d/
│   └── loop_closure_node.hpp
├── src/
│   └── loop_closure_node.cpp
├── launch/
│   └── loop_closure_launch.py
└── config/
    └── loop_closure_params.yaml
```

## 技术支持

如有问题，请检查：
1. 日志输出：`ros2 topic echo /rosout | grep loop_closure`
2. TF树：`ros2 run tf2_tools view_frames`
3. 参数文件：确保路径正确

---

**开发者**: Lihan Chen
**创建日期**: 2026-01-06
**ROS版本**: Humble
**依赖**: small_gicp, Point-LIO
