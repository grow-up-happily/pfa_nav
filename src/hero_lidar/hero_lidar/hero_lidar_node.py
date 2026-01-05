#!/usr/bin/env python3
import os
import yaml
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster, Buffer, TransformListener
from rclpy.duration import Duration


class WaypointStaticPublisher(Node):
    """从 ./waypoints.yaml 中读取第一个 waypoint 的坐标并发布为 map->base 的静态变换，
    并周期性打印 chassis->base 的 TF（如果可用）。
    兼容多种 YAML 键名形式（例如 Pos_x、Pos_x、pos_x、x 等）。
    """

    def __init__(self):
        super().__init__('waypoint_static_publisher')

        # 允许通过 ROS 参数指定 waypoints 文件路径；为空时回退到 CWD/waypoints.yaml
        self.declare_parameter('waypoints_file', '')
        wp_path_param = self.get_parameter('waypoints_file').get_parameter_value().string_value
        wp_path = wp_path_param if wp_path_param else os.path.join(os.getcwd(), 'waypoints.yaml')
        if not os.path.exists(wp_path):
            self.get_logger().error(f'waypoints file not found: {wp_path}')
            raise FileNotFoundError(wp_path)

        with open(wp_path, 'r') as f:
            try:
                data = yaml.safe_load(f)
            except Exception as e:
                self.get_logger().error(f'failed to parse yaml: {e}')
                raise

        first = None

        # 如果顶层是 dict，常见格式为 Waypoint_1: {...}, Waypoint_2: {...}
        if isinstance(data, dict):
            # 优先查找键名以 "waypoint" 开头的项
            for k in sorted(data.keys()):
                if isinstance(k, str) and k.lower().startswith('waypoint') and isinstance(data[k], dict):
                    first = data[k]
                    break

            # 如果没有找到上述格式，尝试从字典的第一个值中取第一个 dict
            if first is None:
                for v in data.values():
                    if isinstance(v, dict):
                        first = v
                        break

        # 如果顶层就是列表（另一种可能），直接取第一个元素
        elif isinstance(data, (list, tuple)) and len(data) > 0:
            first = data[0]

        if not first or not isinstance(first, dict):
            self.get_logger().error('no waypoint entries found in waypoints.yaml')
            raise RuntimeError('no waypoints')

        # 辅助：按 case-insensitive 且忽略下划线的方式查找字段
        def _ci_key_lookup(dct, targets):
            # targets: iterable of candidate normalized names e.g. ('posx','x','px')
            for k, v in dct.items():
                nk = ''.join(k.lower().split('_'))
                if nk in targets:
                    return v
            return None

        # 查找 x,y 值（支持 Pos_x/Pos_y/PosX/x 等）
        raw_x = _ci_key_lookup(first, {'posx', 'x', 'px'})
        raw_y = _ci_key_lookup(first, {'posy', 'y', 'py'})

        if raw_x is None or raw_y is None:
            # 作为后备，尝试从任意含 'pos' 前缀的键中解析
            for k, v in first.items():
                nk = ''.join(k.lower().split('_'))
                if nk.startswith('pos') and raw_x is None:
                    # pos maybe like pos_x or pos1x etc — 尝试拆分数字后缀
                    if 'x' in nk:
                        raw_x = v
                    elif 'y' in nk:
                        raw_y = v

        try:
            x = float(raw_x)
            y = float(raw_y)
        except Exception as e:
            self.get_logger().error(f'failed to parse first waypoint coordinates: {e}')
            raise

        self.get_logger().info(f'first waypoint: x={x}, y={y}')

        # 发布 map -> base 的静态变换（z=0，yaw=0）
        self.broadcaster = StaticTransformBroadcaster(self)
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'base'
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = 0.0
        # 0 yaw 的四元数
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0

        # StaticTransformBroadcaster 在创建后立即发送一次变换
        self.broadcaster.sendTransform([t])
        self.get_logger().info('published static transform map -> base')

        # 用于查询 chassis -> base 的 TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # 定时打印 chassis->base（每秒一次）
        self.create_timer(1.0, self._print_chassis_base)

    def _print_chassis_base(self):
        try:
            # 查询 chassis -> base（target_frame, source_frame）
            trans = self.tf_buffer.lookup_transform('chassis', 'base', rclpy.time.Time(), timeout=Duration(seconds=0.5))
            t = trans.transform.translation
            q = trans.transform.rotation
            self.get_logger().info(
                f'chassis->base: trans=({t.x:.3f}, {t.y:.3f}, {t.z:.3f}), quat=({q.x:.3f}, {q.y:.3f}, {q.z:.3f}, {q.w:.3f})'
            )
        except Exception as e:
            # 常见情况是暂时没有该 TF
            self.get_logger().warning(f'chassis->base not available: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = WaypointStaticPublisher()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()