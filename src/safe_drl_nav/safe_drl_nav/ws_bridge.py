#!/usr/bin/env python3
"""
ROS2 → WebSocket bridge for the safe_drl_nav robot.

Topics bridged
--------------
  /scan   (sensor_msgs/LaserScan)   → broadcast to all WS clients as JSON
  /odom   (nav_msgs/Odometry)       → broadcast to all WS clients as JSON
  /cmd_vel (geometry_msgs/Twist)    ← publish when WS client sends a drive cmd

Usage
-----
  # Install the websockets package once:
  pip install websockets

  # In a sourced workspace terminal:
  python3 src/safe_drl_nav/safe_drl_nav/ws_bridge.py [--host 0.0.0.0] [--port 9090]
"""

from __future__ import annotations

import argparse
import asyncio
import json
import math
import queue
import threading
from typing import Any, Set

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

try:
    import websockets
except ImportError:
    raise SystemExit(
        "websockets library not found — run:  pip install websockets"
    )

# Keep WebSocket connections off the ROS node (avoids rclpy / websockets clashes).
_WS_CLIENTS: Set[Any] = set()
_WS_CLIENTS_LOCK = threading.Lock()
_OUTBOUND_Q: queue.Queue[str | None] = queue.Queue()

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _yaw_from_quaternion(q) -> float:
    """Extract yaw (radians) from a geometry_msgs quaternion."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def _pool_ranges(ranges: list[float], n_bins: int, range_max: float) -> list[float]:
    """Min-pool raw LaserScan ranges into n_bins for compact transmission."""
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


# ---------------------------------------------------------------------------
# ROS2 node (runs in a dedicated thread)
# ---------------------------------------------------------------------------

class BridgeNode(Node):
    """Subscribes to robot sensors; pushes JSON strings to _OUTBOUND_Q for asyncio."""

    LIDAR_BINS = 36

    def __init__(self) -> None:
        super().__init__("ws_bridge_node")

        # Publisher (/cmd_vel)
        self._cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        # Match Gazebo turtlebot3_laserscan (RELIABLE); sensor_data QoS never receives scans.
        scan_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
        )
        self._last_scan_bins: list[float] = [0.0] * self.LIDAR_BINS

        self.create_subscription(LaserScan, "/scan", self._on_scan, scan_qos)
        self.create_subscription(Odometry, "/odom", self._on_odom, 10)

        self.get_logger().info("BridgeNode ready — waiting for sensor data …")

    # ------------------------------------------------------------------
    # Public helpers called from the asyncio thread
    # ------------------------------------------------------------------

    @staticmethod
    def register_client(ws: Any) -> None:
        with _WS_CLIENTS_LOCK:
            _WS_CLIENTS.add(ws)
        print(f"[ws_bridge] client connected ({len(_WS_CLIENTS)} total)", flush=True)

    @staticmethod
    def unregister_client(ws: Any) -> None:
        with _WS_CLIENTS_LOCK:
            _WS_CLIENTS.discard(ws)
        print(f"[ws_bridge] client disconnected ({len(_WS_CLIENTS)} total)", flush=True)

    def publish_twist(self, linear_x: float, angular_z: float) -> None:
        msg = Twist()
        msg.linear.x = float(linear_x)
        msg.angular.z = float(angular_z)
        self._cmd_pub.publish(msg)

    # ------------------------------------------------------------------
    # ROS callbacks (called from the rclpy spin thread)
    # ------------------------------------------------------------------

    def _on_scan(self, msg: LaserScan) -> None:
        bins = _pool_ranges(
            list(msg.ranges), self.LIDAR_BINS, float(msg.range_max)
        )
        self._last_scan_bins = bins
        self._broadcast(
            json.dumps(
                {
                    "type": "scan",
                    "range_max": round(float(msg.range_max), 2),
                    "bins": bins,
                }
            )
        )

    def _viz_scan_24(self) -> list[float]:
        """Web dashboard index.html uses 24 bins for the radar wedge."""
        n_out = 24
        n_in = len(self._last_scan_bins) or 1
        out: list[float] = []
        for i in range(n_out):
            lo = int(i * n_in / n_out)
            hi = int((i + 1) * n_in / n_out)
            seg = self._last_scan_bins[lo:hi] or [self._last_scan_bins[lo]]
            out.append(round(min(seg), 3))
        return out

    def _on_odom(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        v = msg.twist.twist
        yaw = _yaw_from_quaternion(q)
        self._broadcast(
            json.dumps(
                {
                    "type": "odom",
                    "x": round(float(p.x), 3),
                    "y": round(float(p.y), 3),
                    "yaw_deg": round(math.degrees(yaw), 1),
                    "lin_vel": round(float(v.linear.x), 3),
                    "ang_vel": round(float(v.angular.z), 3),
                }
            )
        )
        # Deployed /ros/index.html expects { t: 's', x, y, yaw, scan, ... } (live viz).
        self._broadcast(
            json.dumps(
                {
                    "t": "s",
                    "x": round(float(p.x), 3),
                    "y": round(float(p.y), 3),
                    "yaw": round(float(yaw), 4),
                    "scan": self._viz_scan_24(),
                    "ep": 0,
                    "step": 0,
                    "rew": 0.0,
                    "tot": 0.0,
                    "hit": False,
                    "ok": True,
                }
            )
        )

    def _broadcast(self, payload: str) -> None:
        with _WS_CLIENTS_LOCK:
            if not _WS_CLIENTS:
                return
        _OUTBOUND_Q.put(payload)


# ---------------------------------------------------------------------------
# Async WebSocket helpers
# ---------------------------------------------------------------------------

async def _outbound_pump() -> None:
    """Drain ROS-thread queue and send to WebSocket clients (asyncio thread only)."""
    while True:
        try:
            msg = _OUTBOUND_Q.get_nowait()
        except queue.Empty:
            await asyncio.sleep(0.02)
            continue
        if msg is None:
            return
        with _WS_CLIENTS_LOCK:
            targets = list(_WS_CLIENTS)
        if targets:
            await _broadcast_all(targets, msg)


async def _broadcast_all(clients: list[Any], message: str) -> None:
    for ws in clients:
        try:
            await ws.send(message)
        except Exception:
            pass  # client already gone; unregister happens in handler


async def ws_handler(ws: Any, node: BridgeNode) -> None:
    """Handle one WebSocket connection lifetime."""
    node.register_client(ws)
    try:
        async for raw in ws:
            try:
                data = json.loads(raw)
            except json.JSONDecodeError:
                continue

            if data.get("type") == "cmd_vel":
                node.publish_twist(
                    float(data.get("linear_x", 0.0)),
                    float(data.get("angular_z", 0.0)),
                )
            elif data.get("type") == "stop":
                node.publish_twist(0.0, 0.0)
    except Exception:
        pass
    finally:
        node.unregister_client(ws)
        # Send a zero-velocity command when client disconnects for safety
        node.publish_twist(0.0, 0.0)


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main() -> None:
    parser = argparse.ArgumentParser(description="ROS2 ↔ WebSocket bridge")
    parser.add_argument("--host", default="0.0.0.0", help="WebSocket bind host")
    parser.add_argument("--port", type=int, default=9090, help="WebSocket port")
    args = parser.parse_args()

    rclpy.init()
    node = BridgeNode()

    from rclpy.executors import MultiThreadedExecutor

    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)

    ros_thread = threading.Thread(
        target=executor.spin, daemon=True, name="rclpy-spin"
    )
    ros_thread.start()

    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)

    async def _serve() -> None:
        async with websockets.serve(
            lambda ws, _path=None: ws_handler(ws, node),
            args.host,
            args.port,
        ):
            asyncio.create_task(_outbound_pump())
            print(
                f"\n  WebSocket bridge listening on  ws://{args.host}:{args.port}\n"
                "  Open the web dashboard to connect.\n"
                "  Press Ctrl-C to stop.\n"
            )
            await asyncio.Future()  # run forever

    try:
        loop.run_until_complete(_serve())
    except KeyboardInterrupt:
        print("\nBridge stopped.")
    finally:
        _OUTBOUND_Q.put(None)
        node.publish_twist(0.0, 0.0)
        executor.shutdown()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
