#!/usr/bin/env python3

import argparse
import os
import re
import subprocess
import sys
import time
from collections import defaultdict
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

import yaml

try:
    import rclpy
    from nav_msgs.msg import OccupancyGrid, Path as NavPath
    from rclpy.node import Node
    from rclpy.qos import (
        DurabilityPolicy,
        HistoryPolicy,
        QoSProfile,
        ReliabilityPolicy,
        qos_profile_sensor_data,
    )
    from rosidl_runtime_py.utilities import get_message
    from sensor_msgs.msg import LaserScan, PointCloud2
    from tf2_msgs.msg import TFMessage
    from visualization_msgs.msg import MarkerArray
except ImportError as exc:
    print(
        "Failed to import ROS 2 Python packages. Source your ROS environment first.\n"
        f"Import error: {exc}",
        file=sys.stderr,
    )
    sys.exit(2)


HEAVY_DISPLAY_CLASSES = {
    "rviz_default_plugins/PointCloud2",
    "rviz_default_plugins/LaserScan",
    "rviz_default_plugins/Map",
    "rviz_default_plugins/MarkerArray",
    "rviz_default_plugins/Path",
}
DEFAULT_EXTRA_TOPICS = [
    "terrain_map_ext",
    "sensor_scan",
    "obstacle_scan",
    "livox/lidar",
    "velodyne_points",
    "map",
    "map_updates",
    "/tf",
    "/tf_static",
]


@dataclass
class DisplayInfo:
    name: str
    class_name: str
    topic: str
    enabled: bool


@dataclass
class TopicWindow:
    count: int = 0
    bytes_total: int = 0


@dataclass
class TopicStats:
    topic: str
    type_name: Optional[str] = None
    count: int = 0
    bytes_total: int = 0
    first_seen: Optional[float] = None
    last_seen: Optional[float] = None
    last_window: TopicWindow = field(default_factory=TopicWindow)
    peak_hz: float = 0.0
    peak_bandwidth: float = 0.0

    def register(self, now: float, payload_size: int) -> None:
        self.count += 1
        self.bytes_total += max(payload_size, 0)
        self.last_seen = now
        if self.first_seen is None:
            self.first_seen = now
        self.last_window.count += 1
        self.last_window.bytes_total += max(payload_size, 0)

    def consume_window(self, interval: float) -> Tuple[float, float]:
        if interval <= 0:
            return 0.0, 0.0
        hz = self.last_window.count / interval
        bandwidth = self.last_window.bytes_total / interval
        self.peak_hz = max(self.peak_hz, hz)
        self.peak_bandwidth = max(self.peak_bandwidth, bandwidth)
        self.last_window = TopicWindow()
        return hz, bandwidth

    def average_hz(self) -> float:
        if self.count == 0:
            return 0.0
        if self.count < 2:
            return 0.0
        if self.first_seen is None or self.last_seen is None:
            return 0.0
        span = max(self.last_seen - self.first_seen, 1e-6)
        return self.count / span

    def average_hz_with_fallback(self, fallback_span: float) -> float:
        if self.count == 0:
            return 0.0
        if self.count < 2:
            return self.count / max(fallback_span, 1e-6)
        return self.average_hz()

    def average_bandwidth(self) -> float:
        if self.count == 0:
            return 0.0
        if self.count < 2:
            return 0.0
        if self.first_seen is None or self.last_seen is None:
            return 0.0
        span = max(self.last_seen - self.first_seen, 1e-6)
        return self.bytes_total / span

    def average_bandwidth_with_fallback(self, fallback_span: float) -> float:
        if self.count == 0:
            return 0.0
        if self.count < 2:
            return self.bytes_total / max(fallback_span, 1e-6)
        return self.average_bandwidth()

    def average_message_bytes(self) -> float:
        return self.bytes_total / self.count if self.count else 0.0


@dataclass
class ProcInfo:
    pid: int
    ppid: int
    comm: str
    args: str


@dataclass
class ThreadSample:
    tid: int
    name: str
    cpu_percent: float


@dataclass
class ProcessSample:
    pid: int
    label: str
    comm: str
    args: str
    cpu_percent: float
    threads: List[ThreadSample]


@dataclass
class ProcessPeak:
    timestamp: str
    sample: ProcessSample


class ReportWriter:
    def __init__(self, output_path: Optional[Path]) -> None:
        self.output_path = output_path
        self.handle = None
        if output_path is not None:
            output_path.parent.mkdir(parents=True, exist_ok=True)
            self.handle = output_path.open("w", encoding="utf-8")

    def close(self) -> None:
        if self.handle is not None:
            self.handle.close()

    def line(self, text: str = "") -> None:
        print(text)
        if self.handle is not None:
            self.handle.write(text + "\n")
            self.handle.flush()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Sample CPU usage by process/thread and correlate it with RViz displays and ROS topics."
        )
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=20.0,
        help="Sampling duration in seconds.",
    )
    parser.add_argument(
        "--interval",
        type=float,
        default=1.0,
        help="Sampling interval in seconds.",
    )
    parser.add_argument(
        "--namespace",
        default="red_standard_robot1",
        help="Namespace used by RViz relative topics.",
    )
    parser.add_argument(
        "--rviz-config",
        default=(
            "src/pb2025_sentry_nav/pb2025_nav_bringup/rviz/nav2_default_view.rviz"
        ),
        help="Path to the RViz config file to inspect.",
    )
    parser.add_argument(
        "--extra-topic",
        action="append",
        default=[],
        help="Additional topic to monitor. May be specified multiple times.",
    )
    parser.add_argument(
        "--top-processes",
        type=int,
        default=8,
        help="How many top CPU processes to show each interval.",
    )
    parser.add_argument(
        "--top-threads",
        type=int,
        default=5,
        help="How many top threads to show for each selected process.",
    )
    parser.add_argument(
        "--process-regex",
        default="rviz2|ruby|gz|ign|pointlio|slam_toolbox|terrainAnalysis|pointcloud_to_laserscan|sensor_scan_generation|loam_interface",
        help=(
            "Regex for processes that should always be included in detailed thread sampling."
        ),
    )
    parser.add_argument(
        "--output",
        default="/tmp/diagnose_viz_cpu.log",
        help="Path to the detailed log file. Use '' to disable file logging.",
    )
    return parser.parse_args()


def normalize_namespace(namespace: str) -> str:
    if not namespace:
        return ""
    return "/" + namespace.strip("/")


def resolve_topic(name: str, namespace: str) -> str:
    if not name:
        return ""
    if name.startswith("/"):
        return name
    normalized_ns = normalize_namespace(namespace)
    if not normalized_ns:
        return "/" + name.lstrip("/")
    return f"{normalized_ns}/{name.lstrip('/')}"


def read_rviz_displays(config_path: Path, namespace: str) -> List[DisplayInfo]:
    if not config_path.exists():
        return []

    with config_path.open("r", encoding="utf-8") as handle:
        data = yaml.safe_load(handle)

    displays_root = []
    viz_manager = data.get("Visualization Manager", {}) if isinstance(data, dict) else {}
    if isinstance(viz_manager, dict):
        displays_root = viz_manager.get("Displays", [])

    displays: List[DisplayInfo] = []

    def walk(items: Iterable[dict]) -> None:
        for item in items:
            if not isinstance(item, dict):
                continue
            class_name = str(item.get("Class", ""))
            name = str(item.get("Name", class_name))
            enabled = (
                item.get("Value")
                if isinstance(item.get("Value"), bool)
                else bool(item.get("Enabled", False))
            )
            topic_value = ""
            topic_field = item.get("Topic")
            if isinstance(topic_field, dict):
                topic_value = str(topic_field.get("Value", ""))
            if class_name in HEAVY_DISPLAY_CLASSES and topic_value:
                displays.append(
                    DisplayInfo(
                        name=name,
                        class_name=class_name,
                        topic=resolve_topic(topic_value, namespace),
                        enabled=enabled,
                    )
                )
            nested = item.get("Displays")
            if isinstance(nested, list):
                walk(nested)

    walk(displays_root)
    return displays


def read_proc_stat_cpu_total() -> Tuple[int, int]:
    with Path("/proc/stat").open("r", encoding="utf-8") as handle:
        fields = handle.readline().strip().split()[1:]
    values = [int(value) for value in fields]
    idle = values[3] + (values[4] if len(values) > 4 else 0)
    total = sum(values)
    return idle, total


def read_proc_stat_ticks(stat_path: Path) -> Optional[int]:
    try:
        payload = stat_path.read_text(encoding="utf-8")
    except FileNotFoundError:
        return None
    right = payload.rfind(")")
    if right < 0:
        return None
    fields = payload[right + 2 :].split()
    if len(fields) < 15:
        return None
    return int(fields[11]) + int(fields[12])


def list_processes() -> Dict[int, ProcInfo]:
    result = subprocess.run(
        ["ps", "-e", "-o", "pid=,ppid=,comm=,args="],
        check=False,
        capture_output=True,
        text=True,
    )
    processes: Dict[int, ProcInfo] = {}
    for raw_line in result.stdout.splitlines():
        line = raw_line.strip()
        if not line:
            continue
        parts = raw_line.split(None, 3)
        if len(parts) < 3:
            continue
        pid = int(parts[0])
        ppid = int(parts[1])
        comm = parts[2]
        args = parts[3] if len(parts) > 3 else comm
        processes[pid] = ProcInfo(pid=pid, ppid=ppid, comm=comm, args=args)
    return processes


def estimate_message_bytes(msg) -> int:
    if isinstance(msg, PointCloud2):
        return len(msg.data)
    if isinstance(msg, LaserScan):
        return 4 * (len(msg.ranges) + len(msg.intensities))
    if isinstance(msg, OccupancyGrid):
        return len(msg.data)
    if isinstance(msg, TFMessage):
        return len(msg.transforms) * 112
    if isinstance(msg, MarkerArray):
        return len(msg.markers) * 256
    if isinstance(msg, NavPath):
        return len(msg.poses) * 64
    if hasattr(msg, "data"):
        try:
            return len(msg.data)
        except TypeError:
            return 0
    return 0


def classify_process(proc: ProcInfo, thread_names: Sequence[str]) -> str:
    args_lower = proc.args.lower()
    joined_thread_names = " ".join(name.lower() for name in thread_names)

    if "diagnose_viz_cpu.py" in args_lower:
        return "diagnostic-script"
    if proc.comm == "rviz2":
        return "rviz2"
    if "component_container_isolated" in args_lower:
        return "nav2_container"
    if "sync_slam_toolbox_node" in args_lower or "sync_slam_toolb" in proc.comm:
        return "slam_toolbox"
    if "pointlio_mapping" in args_lower or "point_lio" in args_lower:
        return "point_lio"
    if "terrainanalysisext" in args_lower:
        return "terrain_analysis_ext"
    if "terrainanalysis" in args_lower:
        return "terrain_analysis"
    if "pointcloud_to_laserscan_node" in args_lower:
        return "pointcloud_to_laserscan"
    if "sensor_scan_generation_node" in args_lower:
        return "sensor_scan_generation"
    if "ign gazebo" in args_lower or "gz sim" in args_lower or proc.comm == "ruby":
        if "gui" in joined_thread_names:
            return "gazebo-gui"
        return "gazebo-server"
    return proc.comm


class TopicMonitor(Node):
    def __init__(self, topics_to_watch: Sequence[str]) -> None:
        super().__init__("diagnose_viz_cpu")
        self.topics_to_watch = list(dict.fromkeys(topic for topic in topics_to_watch if topic))
        self.topic_stats = {topic: TopicStats(topic=topic) for topic in self.topics_to_watch}
        self.topic_types: Dict[str, str] = {}
        self.topic_subscriptions = {}

    def discover_topics(self) -> None:
        topic_map = dict(self.get_topic_names_and_types())
        for topic in self.topics_to_watch:
            if topic in self.topic_subscriptions:
                continue
            type_names = topic_map.get(topic)
            if not type_names:
                continue
            type_name = type_names[0]
            msg_cls = get_message(type_name)
            qos = select_qos(type_name, topic)
            self.topic_subscriptions[topic] = self.create_subscription(
                msg_cls,
                topic,
                self._make_callback(topic),
                qos,
            )
            self.topic_types[topic] = type_name
            self.topic_stats[topic].type_name = type_name

    def _make_callback(self, topic: str):
        def callback(msg) -> None:
            now = time.monotonic()
            self.topic_stats[topic].register(now, estimate_message_bytes(msg))

        return callback

    def interval_snapshot(self, interval: float) -> List[Tuple[str, float, float]]:
        rows = []
        for topic in self.topics_to_watch:
            hz, bandwidth = self.topic_stats[topic].consume_window(interval)
            rows.append((topic, hz, bandwidth))
        return rows


def select_qos(type_name: str, topic: str) -> QoSProfile:
    if type_name in {
        "sensor_msgs/msg/PointCloud2",
        "sensor_msgs/msg/LaserScan",
        "sensor_msgs/msg/Imu",
        "sensor_msgs/msg/JointState",
    }:
        return qos_profile_sensor_data
    if type_name == "tf2_msgs/msg/TFMessage" and topic.endswith("tf_static"):
        return QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=100,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
    if type_name == "tf2_msgs/msg/TFMessage":
        return QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=100,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
    if type_name in {"nav_msgs/msg/OccupancyGrid", "map_msgs/msg/OccupancyGridUpdate"}:
        return QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=10,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
    )


class CpuSampler:
    def __init__(self, top_threads: int, process_regex: str) -> None:
        self.hz = os.sysconf(os.sysconf_names["SC_CLK_TCK"])
        self.top_threads = top_threads
        self.process_regex = re.compile(process_regex) if process_regex else None
        self.self_pid = os.getpid()
        self.prev_proc_ticks: Dict[int, int] = {}
        self.prev_thread_ticks: Dict[Tuple[int, int], int] = {}
        self.prev_wall: Optional[float] = None
        self.prev_total_idle: Optional[int] = None
        self.prev_total_ticks: Optional[int] = None

    def sample(self, top_processes: int) -> Tuple[float, List[ProcessSample]]:
        now = time.monotonic()
        idle_ticks, total_ticks = read_proc_stat_cpu_total()
        processes = list_processes()

        if self.prev_wall is None:
            self.prev_wall = now
            self.prev_total_idle = idle_ticks
            self.prev_total_ticks = total_ticks
            for pid in processes:
                ticks = read_proc_stat_ticks(Path(f"/proc/{pid}/stat"))
                if ticks is not None:
                    self.prev_proc_ticks[pid] = ticks
            return 0.0, []

        wall_delta = max(now - self.prev_wall, 1e-6)
        total_delta = max(total_ticks - self.prev_total_ticks, 1)
        idle_delta = max(idle_ticks - self.prev_total_idle, 0)
        system_cpu = (1.0 - idle_delta / total_delta) * 100.0

        proc_samples: List[Tuple[float, ProcInfo]] = []
        next_proc_ticks: Dict[int, int] = {}

        for pid, proc in processes.items():
            if pid == self.self_pid:
                continue
            ticks = read_proc_stat_ticks(Path(f"/proc/{pid}/stat"))
            if ticks is None:
                continue
            next_proc_ticks[pid] = ticks
            prev_ticks = self.prev_proc_ticks.get(pid)
            if prev_ticks is None:
                continue
            cpu_percent = ((ticks - prev_ticks) / self.hz) / wall_delta * 100.0
            if cpu_percent < 0.01:
                continue
            proc_samples.append((cpu_percent, proc))

        proc_samples.sort(key=lambda item: item[0], reverse=True)

        selected: Dict[int, ProcInfo] = {
            proc.pid: proc for _, proc in proc_samples[:top_processes]
        }
        if self.process_regex is not None:
            for _, proc in proc_samples:
                if self.process_regex.search(proc.comm) or self.process_regex.search(proc.args):
                    selected[proc.pid] = proc

        detailed: List[ProcessSample] = []
        next_thread_ticks: Dict[Tuple[int, int], int] = {}

        for pid, proc in selected.items():
            process_cpu = 0.0
            for cpu_percent, candidate in proc_samples:
                if candidate.pid == pid:
                    process_cpu = cpu_percent
                    break

            task_dir = Path(f"/proc/{pid}/task")
            thread_rows: List[ThreadSample] = []
            all_thread_names: List[str] = []
            try:
                tids = [int(entry.name) for entry in task_dir.iterdir() if entry.name.isdigit()]
            except FileNotFoundError:
                continue

            for tid in tids:
                ticks = read_proc_stat_ticks(task_dir / str(tid) / "stat")
                if ticks is None:
                    continue
                key = (pid, tid)
                next_thread_ticks[key] = ticks
                try:
                    name = (task_dir / str(tid) / "comm").read_text(encoding="utf-8").strip()
                except FileNotFoundError:
                    name = str(tid)
                all_thread_names.append(name)
                prev_ticks = self.prev_thread_ticks.get(key)
                if prev_ticks is None:
                    continue
                cpu_percent = ((ticks - prev_ticks) / self.hz) / wall_delta * 100.0
                if cpu_percent < 0.01:
                    continue
                thread_rows.append(
                    ThreadSample(tid=tid, name=name, cpu_percent=cpu_percent)
                )

            thread_rows.sort(key=lambda item: item.cpu_percent, reverse=True)
            label = classify_process(proc, all_thread_names)
            detailed.append(
                ProcessSample(
                    pid=pid,
                    label=label,
                    comm=proc.comm,
                    args=proc.args,
                    cpu_percent=process_cpu,
                    threads=thread_rows[: self.top_threads],
                )
            )

        detailed.sort(key=lambda item: item.cpu_percent, reverse=True)
        self.prev_wall = now
        self.prev_total_idle = idle_ticks
        self.prev_total_ticks = total_ticks
        self.prev_proc_ticks = next_proc_ticks
        self.prev_thread_ticks = next_thread_ticks
        return system_cpu, detailed


def format_bytes_per_second(value: float) -> str:
    units = ["B/s", "KiB/s", "MiB/s", "GiB/s"]
    idx = 0
    while value >= 1024.0 and idx < len(units) - 1:
        value /= 1024.0
        idx += 1
    return f"{value:.1f}{units[idx]}"


def format_bytes(value: float) -> str:
    units = ["B", "KiB", "MiB", "GiB"]
    idx = 0
    while value >= 1024.0 and idx < len(units) - 1:
        value /= 1024.0
        idx += 1
    return f"{value:.1f}{units[idx]}"


def format_cores(value: float) -> str:
    cores = value / 100.0
    if cores < 1.0:
        return f"{cores:.2f} cores"
    return f"{cores:.1f} cores"


def process_explanation(label: str) -> str:
    explanations = {
        "gazebo-gui": "Ignition/Gazebo 图形界面，通常对应 3D 视图、Qt/QML 界面和 GUI 插件刷新。",
        "gazebo-server": "Gazebo 仿真本体，通常对应世界推进、传感器生成、插件执行和 transport 消息处理。",
        "nav2_container": "Nav2 组合容器，里面可能同时装着 controller/planner/bt_navigator/velocity_smoother 等节点。",
        "slam_toolbox": "2D SLAM 建图节点，通常对应 scan 处理、地图更新和优化。",
        "rviz2": "RViz 可视化进程，通常对应显示项刷新、TF 变换和渲染准备。",
        "point_lio": "Point-LIO 点云里程计/建图链路。",
        "terrain_analysis": "地形分析节点，处理局部障碍点云。",
        "terrain_analysis_ext": "扩展地形分析节点，生成 terrain_map_ext 点云。",
        "pointcloud_to_laserscan": "点云转 2D LaserScan 节点。",
        "sensor_scan_generation": "点云与里程计同步、转换与 TF 发布节点。",
    }
    return explanations.get(label, "需要结合参数和线程名进一步判断。")


def thread_explanation(thread_name: str) -> str:
    lowered = thread_name.lower()
    if "qsgrenderthread" in lowered:
        return "Qt Quick 渲染线程，偏向 GUI 绘制和提交渲染命令。"
    if "ignition::gui::" in lowered or "gz::gui" in lowered:
        return "Ignition GUI 线程，偏向场景更新、GUI 插件逻辑或界面刷新。"
    if "zmqbg/io" in lowered:
        return "Gazebo transport 后台消息线程。"
    if "component_conta" in lowered:
        return "Nav2 组合容器工作线程。"
    if "rviz2" == lowered:
        return "RViz 主工作线程。"
    if "ruby" == lowered:
        return "Gazebo / ign 启动与运行线程名，通常落在 server 或 gui 主逻辑。"
    return ""


def print_displays(writer: ReportWriter, displays: Sequence[DisplayInfo]) -> None:
    writer.line("RViz heavy displays from config:")
    if not displays:
        writer.line("  (none found)")
        writer.line()
        return
    for display in displays:
        state = "enabled" if display.enabled else "disabled"
        writer.line(
            f"  - {display.name}: {display.class_name} | {display.topic} | {state}"
        )
    writer.line()


def build_topics_to_watch(
    displays: Sequence[DisplayInfo], namespace: str, extra_topics: Sequence[str]
) -> List[str]:
    topics = []
    for display in displays:
        if display.enabled:
            topics.append(display.topic)
    for topic in DEFAULT_EXTRA_TOPICS:
        topics.append(resolve_topic(topic, namespace))
    for topic in extra_topics:
        topics.append(resolve_topic(topic, namespace))
    return list(dict.fromkeys(topic for topic in topics if topic))


def print_interval_snapshot(
    writer: ReportWriter,
    system_cpu: float,
    processes: Sequence[ProcessSample],
    topic_rows: Sequence[Tuple[str, float, float]],
) -> None:
    timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    proc_parts = []
    for proc in processes[:5]:
        thread_desc = ", ".join(
            f"{thread.name}:{thread.cpu_percent:.1f}%"
            for thread in proc.threads[:3]
        )
        if thread_desc:
            proc_parts.append(
                f"{proc.label}:{proc.pid} {proc.cpu_percent:.1f}% [{thread_desc}]"
            )
        else:
            proc_parts.append(f"{proc.label}:{proc.pid} {proc.cpu_percent:.1f}%")
    active_topics = [
        f"{topic} {hz:.2f}Hz {format_bytes_per_second(bps)}"
        for topic, hz, bps in sorted(topic_rows, key=lambda item: item[1], reverse=True)
        if hz > 0.0
    ][:5]
    writer.line(
        f"{timestamp} | system {system_cpu:.1f}% | "
        f"processes: {' | '.join(proc_parts) if proc_parts else '(warming up)'}"
    )
    if active_topics:
        writer.line(f"  topics: {' | '.join(active_topics)}")


def print_peak_snapshot(writer: ReportWriter, process_peaks: Dict[str, ProcessPeak]) -> None:
    writer.line("  Peak snapshots:")
    ranked = sorted(
        process_peaks.items(),
        key=lambda item: item[1].sample.cpu_percent,
        reverse=True,
    )
    for label, peak in ranked[:6]:
        sample = peak.sample
        writer.line(
            f"    - {label}: peak {sample.cpu_percent:.1f}% ({format_cores(sample.cpu_percent)}) at {peak.timestamp}"
        )
        writer.line(f"      why it matters: {process_explanation(label)}")
        if sample.threads:
            writer.line("      hottest threads:")
            for thread in sample.threads[:3]:
                extra = thread_explanation(thread.name)
                suffix = f" | {extra}" if extra else ""
                writer.line(
                    f"        * {thread.name}: {thread.cpu_percent:.1f}% ({format_cores(thread.cpu_percent)}){suffix}"
                )


def print_disabled_but_active_displays(
    writer: ReportWriter,
    displays: Sequence[DisplayInfo],
    topic_monitor: TopicMonitor,
    monitor_span: float,
) -> None:
    rows = []
    for display in displays:
        if display.enabled:
            continue
        stats = topic_monitor.topic_stats.get(display.topic)
        if stats is None or stats.count == 0:
            continue
        rows.append((display, stats))
    if not rows:
        return

    writer.line("  Disabled in RViz but still actively published:")
    for display, stats in rows:
        writer.line(
            "    - "
            f"{display.name} ({display.topic}): "
            f"avg {stats.average_hz_with_fallback(monitor_span):.2f}Hz, peak {stats.peak_hz:.2f}Hz, "
            f"avg msg {format_bytes(stats.average_message_bytes())}"
        )
    writer.line(
        "      这表示关掉 RViz 显示只减少了渲染 CPU，不会停止后台生产这条数据。"
    )


def print_final_summary(
    writer: ReportWriter,
    displays: Sequence[DisplayInfo],
    topic_monitor: TopicMonitor,
    monitor_span: float,
    process_history: Dict[str, List[float]],
    thread_history: Dict[Tuple[str, str], List[float]],
    process_peaks: Dict[str, ProcessPeak],
) -> None:
    writer.line()
    writer.line("Summary:")

    if process_history:
        writer.line("  Peak process CPU:")
        ranked_processes = sorted(
            (
                (label, max(values), sum(values) / len(values))
                for label, values in process_history.items()
                if values
            ),
            key=lambda item: item[1],
            reverse=True,
        )
        for label, peak, avg in ranked_processes[:8]:
            writer.line(
                f"    - {label}: peak {peak:.1f}% ({format_cores(peak)}) | avg {avg:.1f}%"
            )
    else:
        writer.line("  No process CPU samples were collected.")

    if process_peaks:
        print_peak_snapshot(writer, process_peaks)

    if thread_history:
        writer.line("  Peak thread CPU:")
        ranked_threads = sorted(
            (
                (process_label, thread_name, max(values))
                for (process_label, thread_name), values in thread_history.items()
                if values
            ),
            key=lambda item: item[2],
            reverse=True,
        )
        for process_label, thread_name, peak in ranked_threads[:10]:
            extra = thread_explanation(thread_name)
            suffix = f" | {extra}" if extra else ""
            writer.line(
                f"    - {process_label} / {thread_name}: peak {peak:.1f}% ({format_cores(peak)}){suffix}"
            )

    writer.line("  Topic rates and payload sizes:")
    topic_rows = sorted(
        topic_monitor.topic_stats.values(),
        key=lambda item: item.average_hz_with_fallback(monitor_span),
        reverse=True,
    )
    for stats in topic_rows:
        if stats.count == 0:
            continue
        writer.line(
            "    - "
            f"{stats.topic}: {stats.average_hz_with_fallback(monitor_span):.2f}Hz | "
            f"peak {stats.peak_hz:.2f}Hz | "
            f"{format_bytes_per_second(stats.average_bandwidth_with_fallback(monitor_span))} | "
            f"avg msg {format_bytes(stats.average_message_bytes())} | "
            f"type {stats.type_name or 'unknown'}"
        )

    print_disabled_but_active_displays(writer, displays, topic_monitor, monitor_span)

    writer.line("  Heuristic conclusion:")
    rviz_peak = max(process_history.get("rviz2", [0.0]), default=0.0)
    gazebo_server_peak = max(process_history.get("gazebo-server", [0.0]), default=0.0)
    gazebo_gui_peak = max(process_history.get("gazebo-gui", [0.0]), default=0.0)
    nav2_peak = max(process_history.get("nav2_container", [0.0]), default=0.0)
    slam_peak = max(process_history.get("slam_toolbox", [0.0]), default=0.0)
    enabled_pointcloud_displays = [
        display for display in displays
        if display.enabled and display.class_name == "rviz_default_plugins/PointCloud2"
    ]

    ranked_labels = sorted(
        (
            (label, max(values))
            for label, values in process_history.items()
            if values
        ),
        key=lambda item: item[1],
        reverse=True,
    )
    if ranked_labels:
        top_three = ", ".join(
            f"{label} {peak:.1f}% ({format_cores(peak)})"
            for label, peak in ranked_labels[:3]
        )
        writer.line(f"    - Highest CPU consumers in this run: {top_three}.")

    if rviz_peak > 100.0 and enabled_pointcloud_displays:
        topic_fragments = []
        for display in enabled_pointcloud_displays:
            stats = topic_monitor.topic_stats.get(display.topic)
            if stats is None or stats.count == 0:
                continue
            topic_fragments.append(
                f"{display.name}({display.topic}, {stats.average_hz_with_fallback(monitor_span):.2f}Hz, avg {format_bytes(stats.average_message_bytes())})"
            )
        if topic_fragments:
            writer.line(
                "    - RViz is a confirmed CPU consumer. Enabled PointCloud2 displays are receiving data: "
                + "; ".join(topic_fragments)
            )
        else:
            writer.line(
                "    - RViz is a confirmed CPU consumer. Check the enabled PointCloud2 displays in the RViz config."
            )
    if gazebo_server_peak > 100.0:
        writer.line(
            "    - Gazebo server is also consuming significant CPU. Check its top threads in the interval log."
        )
    if gazebo_gui_peak > 100.0:
        writer.line(
            "    - Gazebo GUI is also consuming significant CPU. GUI rendering is not free even when GPU is not saturated."
        )
    if nav2_peak > 100.0:
        writer.line(
            "    - nav2_container crossed 100%, so navigation nodes are materially contributing CPU on top of simulation."
        )
    if slam_peak > 80.0:
        writer.line(
            "    - slam_toolbox is using close to one full core by itself."
        )
    if rviz_peak <= 100.0 and gazebo_server_peak <= 100.0 and gazebo_gui_peak <= 100.0:
        writer.line(
            "    - No single process crossed 100%% in this run. Increase --duration or reproduce during the stutter window."
        )


def main() -> int:
    args = parse_args()

    rviz_config = Path(args.rviz_config)
    output_path = Path(args.output) if args.output else None
    writer = ReportWriter(output_path)

    displays = read_rviz_displays(rviz_config, args.namespace)
    topics_to_watch = build_topics_to_watch(displays, args.namespace, args.extra_topic)

    writer.line(f"Sampling duration: {args.duration:.1f}s")
    writer.line(f"Sampling interval: {args.interval:.1f}s")
    writer.line(f"Namespace: {normalize_namespace(args.namespace) or '/'}")
    writer.line(f"RViz config: {rviz_config}")
    writer.line(f"Detailed log: {output_path if output_path else '(stdout only)'}")
    writer.line()
    print_displays(writer, displays)
    writer.line("Watching topics:")
    for topic in topics_to_watch:
        writer.line(f"  - {topic}")
    writer.line()

    rclpy.init(args=None)
    node = TopicMonitor(topics_to_watch)
    sampler = CpuSampler(
        top_threads=args.top_threads,
        process_regex=args.process_regex,
    )

    process_history: Dict[str, List[float]] = defaultdict(list)
    thread_history: Dict[Tuple[str, str], List[float]] = defaultdict(list)
    process_peaks: Dict[str, ProcessPeak] = {}

    start = time.monotonic()
    end_time = start + max(args.duration, args.interval)
    next_discovery = start
    next_sample = start

    try:
        while time.monotonic() < end_time and rclpy.ok():
            now = time.monotonic()
            if now >= next_discovery:
                node.discover_topics()
                next_discovery = now + 2.0

            timeout = max(0.0, min(0.2, next_sample - now))
            rclpy.spin_once(node, timeout_sec=timeout)

            now = time.monotonic()
            if now < next_sample:
                continue

            system_cpu, process_rows = sampler.sample(args.top_processes)
            topic_rows = node.interval_snapshot(args.interval)
            next_sample = now + args.interval

            if not process_rows and system_cpu == 0.0:
                continue

            for proc in process_rows:
                process_history[proc.label].append(proc.cpu_percent)
                existing_peak = process_peaks.get(proc.label)
                if existing_peak is None or proc.cpu_percent > existing_peak.sample.cpu_percent:
                    process_peaks[proc.label] = ProcessPeak(
                        timestamp=datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
                        sample=ProcessSample(
                            pid=proc.pid,
                            label=proc.label,
                            comm=proc.comm,
                            args=proc.args,
                            cpu_percent=proc.cpu_percent,
                            threads=list(proc.threads),
                        ),
                    )
                for thread in proc.threads:
                    thread_history[(proc.label, thread.name)].append(thread.cpu_percent)

            print_interval_snapshot(writer, system_cpu, process_rows, topic_rows)

        monitor_span = max(time.monotonic() - start, 1e-6)
        print_final_summary(
            writer,
            displays,
            node,
            monitor_span,
            process_history,
            thread_history,
            process_peaks,
        )
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()
        writer.close()


if __name__ == "__main__":
    raise SystemExit(main())
