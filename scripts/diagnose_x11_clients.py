#!/usr/bin/env python3

import argparse
import re
import subprocess
import sys
import time
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple


@dataclass
class XrestopEntry:
    title: str
    pid: Optional[int]
    windows: int = 0
    pixmaps: int = 0
    pixmap_bytes: int = 0
    total_bytes: int = 0


@dataclass
class WindowInfo:
    wid: str
    pid: Optional[int]
    wm_class: str
    title: str
    geometry: str
    mapped: bool


@dataclass
class ClientPeak:
    key: str
    pid: Optional[int]
    title: str
    peak_total_bytes: int = 0
    peak_pixmap_bytes: int = 0
    peak_cpu: float = 0.0
    peak_gpu_sm: float = 0.0
    peak_gpu_mem: float = 0.0
    windows: int = 0
    samples: int = 0
    visible_windows: List[WindowInfo] = field(default_factory=list)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Sample X11 clients and correlate xrestop, window owner, CPU and GPU usage."
    )
    parser.add_argument("--samples", type=int, default=10, help="How many samples to take.")
    parser.add_argument("--interval", type=float, default=1.0, help="Seconds between samples.")
    parser.add_argument("--top", type=int, default=12, help="How many clients to print.")
    parser.add_argument(
        "--output",
        default="",
        help="Optional log file path. Leave empty to print only to stdout.",
    )
    return parser.parse_args()


def run_command(cmd: List[str]) -> str:
    result = subprocess.run(cmd, check=False, capture_output=True, text=True)
    if result.returncode != 0 and result.stdout == "" and result.stderr:
        return result.stderr
    return result.stdout


def parse_human_int(value: str) -> int:
    match = re.search(r"(-?\d+)", value.replace(",", ""))
    return int(match.group(1)) if match else 0


def sample_xrestop() -> List[XrestopEntry]:
    text = run_command(["xrestop", "-b", "-m", "1"])
    entries: List[XrestopEntry] = []
    current: Optional[XrestopEntry] = None
    header_re = re.compile(r"^\d+\s+-\s+(.*?)\s+\(\s*PID:\s*(\?|\d+)\s*\):\s*$")
    for raw_line in text.splitlines():
        line = raw_line.rstrip()
        header_match = header_re.match(line)
        if header_match:
            if current is not None:
                entries.append(current)
            pid_text = header_match.group(2)
            current = XrestopEntry(
                title=header_match.group(1).strip() or "<unknown>",
                pid=(None if pid_text == "?" else int(pid_text)),
            )
            continue
        if current is None:
            continue
        if "windows" in line:
            current.windows = parse_human_int(line)
        elif "pixmaps" in line and "pixmap bytes" not in line:
            current.pixmaps = parse_human_int(line)
        elif "pixmap bytes" in line:
            current.pixmap_bytes = parse_human_int(line)
        elif "total bytes" in line:
            current.total_bytes = parse_human_int(line)
    if current is not None:
        entries.append(current)
    return entries


def parse_root_windows() -> List[str]:
    text = run_command(["xprop", "-root", "_NET_CLIENT_LIST_STACKING"])
    return re.findall(r"0x[0-9a-fA-F]+", text)


def parse_window_info(wid: str) -> Optional[WindowInfo]:
    xprop_text = run_command(["xprop", "-id", wid, "_NET_WM_PID", "WM_CLASS", "_NET_WM_NAME"])
    if "BadWindow" in xprop_text:
        return None

    pid_match = re.search(r"_NET_WM_PID\(CARDINAL\)\s*=\s*(\d+)", xprop_text)
    class_match = re.search(r'WM_CLASS\(STRING\)\s*=\s*(.*)', xprop_text)
    title_match = re.search(r'_NET_WM_NAME\(UTF8_STRING\)\s*=\s*"(.*)"', xprop_text)
    if title_match is None:
        title_match = re.search(r"_NET_WM_NAME\(UTF8_STRING\)\s*=\s*(.*)", xprop_text)

    xwininfo_text = run_command(["xwininfo", "-id", wid, "-stats"])
    if "xwininfo: Window id:" not in xwininfo_text:
        return None
    width_match = re.search(r"Width:\s+(\d+)", xwininfo_text)
    height_match = re.search(r"Height:\s+(\d+)", xwininfo_text)
    map_state_match = re.search(r"Map State:\s+(\S+)", xwininfo_text)

    width = width_match.group(1) if width_match else "?"
    height = height_match.group(1) if height_match else "?"
    mapped = map_state_match is not None and map_state_match.group(1) == "IsViewable"
    return WindowInfo(
        wid=wid,
        pid=int(pid_match.group(1)) if pid_match else None,
        wm_class=class_match.group(1).strip() if class_match else "",
        title=(title_match.group(1).strip().strip('"') if title_match else ""),
        geometry=f"{width}x{height}",
        mapped=mapped,
    )


def collect_windows_by_pid() -> Dict[int, List[WindowInfo]]:
    by_pid: Dict[int, List[WindowInfo]] = {}
    for wid in parse_root_windows():
        info = parse_window_info(wid)
        if info is None or info.pid is None:
            continue
        by_pid.setdefault(info.pid, []).append(info)
    return by_pid


def sample_cpu() -> Dict[int, float]:
    text = run_command(["ps", "-eo", "pid=,pcpu="])
    values: Dict[int, float] = {}
    for line in text.splitlines():
        parts = line.split()
        if len(parts) != 2:
            continue
        try:
            values[int(parts[0])] = float(parts[1])
        except ValueError:
            continue
    return values


def sample_gpu() -> Dict[int, Tuple[float, float]]:
    text = run_command(["nvidia-smi", "pmon", "-c", "1"])
    values: Dict[int, Tuple[float, float]] = {}
    for line in text.splitlines():
        if not line or line.startswith("#"):
            continue
        parts = line.split()
        if len(parts) < 8:
            continue
        try:
            pid = int(parts[1])
        except ValueError:
            continue
        sm = 0.0 if parts[3] == "-" else float(parts[3])
        mem = 0.0 if parts[4] == "-" else float(parts[4])
        values[pid] = (sm, mem)
    return values


def format_bytes(value: int) -> str:
    units = ["B", "KiB", "MiB", "GiB"]
    amount = float(max(value, 0))
    for unit in units:
        if amount < 1024.0 or unit == units[-1]:
            return f"{amount:.1f}{unit}"
        amount /= 1024.0
    return f"{amount:.1f}GiB"


def writer_factory(path: str):
    handle = open(path, "w", encoding="utf-8") if path else None

    def emit(line: str = "") -> None:
        print(line)
        if handle is not None:
            handle.write(line + "\n")
            handle.flush()

    return emit, handle


def choose_name(entry: XrestopEntry, windows: List[WindowInfo]) -> str:
    for window in windows:
        if window.title:
            return window.title
    return entry.title


def main() -> int:
    args = parse_args()
    emit, handle = writer_factory(args.output)

    try:
        peaks: Dict[str, ClientPeak] = {}
        emit(f"Sampling X11 clients: {args.samples} samples, interval {args.interval:.1f}s")
        emit()
        for index in range(args.samples):
            timestamp = time.strftime("%F %T")
            windows_by_pid = collect_windows_by_pid()
            cpu_by_pid = sample_cpu()
            gpu_by_pid = sample_gpu()
            entries = sample_xrestop()

            emit(f"[{index + 1}/{args.samples}] {timestamp}")
            for entry in sorted(entries, key=lambda item: item.total_bytes, reverse=True)[: min(args.top, 6)]:
                windows = windows_by_pid.get(entry.pid or -1, [])
                visible_count = sum(1 for window in windows if window.mapped)
                cpu = cpu_by_pid.get(entry.pid or -1, 0.0)
                gpu_sm, gpu_mem = gpu_by_pid.get(entry.pid or -1, (0.0, 0.0))
                emit(
                    f"  pid={entry.pid or '?':>5} total={format_bytes(entry.total_bytes):>9} "
                    f"pixmap={format_bytes(entry.pixmap_bytes):>9} cpu={cpu:>5.1f}% "
                    f"gpu_sm={gpu_sm:>5.1f}% gpu_mem={gpu_mem:>5.1f}% "
                    f"visible={visible_count} title={choose_name(entry, windows)}"
                )

                key = f"{entry.pid}:{entry.title}" if entry.pid is not None else f"?:{entry.title}"
                peak = peaks.setdefault(
                    key,
                    ClientPeak(
                        key=key,
                        pid=entry.pid,
                        title=choose_name(entry, windows),
                    ),
                )
                peak.samples += 1
                peak.peak_total_bytes = max(peak.peak_total_bytes, entry.total_bytes)
                peak.peak_pixmap_bytes = max(peak.peak_pixmap_bytes, entry.pixmap_bytes)
                peak.peak_cpu = max(peak.peak_cpu, cpu)
                peak.peak_gpu_sm = max(peak.peak_gpu_sm, gpu_sm)
                peak.peak_gpu_mem = max(peak.peak_gpu_mem, gpu_mem)
                peak.windows = max(peak.windows, entry.windows)
                if windows:
                    peak.visible_windows = windows
            emit()
            if index + 1 != args.samples:
                time.sleep(args.interval)

        emit("Peak X11 clients:")
        ranked = sorted(
            peaks.values(),
            key=lambda item: (item.peak_total_bytes, item.peak_pixmap_bytes, item.peak_cpu),
            reverse=True,
        )
        for peak in ranked[: args.top]:
            emit(
                f"- pid={peak.pid or '?':>5} total_peak={format_bytes(peak.peak_total_bytes):>9} "
                f"pixmap_peak={format_bytes(peak.peak_pixmap_bytes):>9} "
                f"cpu_peak={peak.peak_cpu:>5.1f}% gpu_sm_peak={peak.peak_gpu_sm:>5.1f}% "
                f"gpu_mem_peak={peak.peak_gpu_mem:>5.1f}% title={peak.title}"
            )
            for window in peak.visible_windows[:3]:
                state = "mapped" if window.mapped else "hidden"
                emit(
                    f"    window {window.wid} {window.geometry} {state} "
                    f"class={window.wm_class} title={window.title}"
                )
    except KeyboardInterrupt:
        emit()
        emit("Interrupted.")
    finally:
        if handle is not None:
            handle.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
