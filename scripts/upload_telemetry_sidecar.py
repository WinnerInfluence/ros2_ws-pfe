#!/usr/bin/env python3
"""ROS /scan + /odom → JSON file for upload_server /lidar_live (separate from Flask)."""
from __future__ import annotations

import json
import math
import os
import sys
import time
from pathlib import Path

WS = Path(os.environ.get("WS_PATH", Path(__file__).resolve().parents[1]))
OUT = Path(os.environ.get("TELEMETRY_FILE", WS / "pfe_logs" / "telemetry_live.json"))
OUT.parent.mkdir(parents=True, exist_ok=True)


def _yaw(q) -> float:
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


def _pool(ranges, n_bins: int, range_max: float) -> list[float]:
    n = len(ranges)
    if n == 0:
        return [range_max] * n_bins
    out: list[float] = []
    for i in range(n_bins):
        lo = int(i * n / n_bins)
        hi = int((i + 1) * n / n_bins)
        seg = [r if math.isfinite(r) and r > 0 else range_max for r in ranges[lo:hi]]
        out.append(round(min(seg) if seg else range_max, 3))
    return out


def _write(snap: dict) -> None:
    try:
        OUT.write_text(json.dumps(snap), encoding="utf-8")
    except OSError:
        OUT.parent.mkdir(parents=True, exist_ok=True)
        OUT.write_text(json.dumps(snap), encoding="utf-8")


def main() -> int:
    import rclpy
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
    from sensor_msgs.msg import LaserScan

    class TNode(Node):
        def __init__(self) -> None:
            super().__init__("upload_telemetry_sidecar")
            self._bins36: list[float] = [3.5] * 36
            self._x, self._y, self._yaw = 0.0, 0.0, 0.0
            scan_qos = QoSProfile(
                depth=10,
                reliability=ReliabilityPolicy.RELIABLE,
                history=HistoryPolicy.KEEP_LAST,
            )
            self.create_subscription(LaserScan, "/scan", self._on_scan, scan_qos)
            self.create_subscription(Odometry, "/odom", self._on_odom, 10)

        def _viz24(self) -> list[float]:
            out: list[float] = []
            for i in range(24):
                lo = int(i * 36 / 24)
                hi = int((i + 1) * 36 / 24)
                seg = self._bins36[lo:hi]
                out.append(round(min(seg), 3))
            return out

        def _on_scan(self, msg: LaserScan) -> None:
            self._bins36 = _pool(list(msg.ranges), 36, float(msg.range_max))
            self._flush()

        def _on_odom(self, msg: Odometry) -> None:
            p = msg.pose.pose.position
            self._x = round(float(p.x), 3)
            self._y = round(float(p.y), 3)
            self._yaw = round(float(_yaw(msg.pose.pose.orientation)), 4)
            self._flush()

        def _flush(self) -> None:
            snap = {
                "ok": True,
                "updated_at": time.time(),
                "t": "s",
                "x": self._x,
                "y": self._y,
                "yaw": self._yaw,
                "scan": self._viz24(),
                "ep": 0,
                "step": 0,
                "rew": 0.0,
                "tot": 0.0,
                "hit": False,
            }
            _write(snap)

    rclpy.init()
    node = TNode()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass
    return 0


if __name__ == "__main__":
    sys.exit(main())
