#!/usr/bin/env python3
"""ROS service /reset_simulation — teleport or respawn my_robot at spawn."""
from __future__ import annotations

import os
import subprocess
import threading
import time

import rclpy
from gazebo_msgs.msg import EntityState
from gazebo_msgs.srv import DeleteEntity, SetEntityState
from geometry_msgs.msg import Pose, Quaternion, Twist
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from std_srvs.srv import Empty


def _default_turtlebot_sdf() -> str:
    ros_d = os.environ.get("ROS_DISTRO", "humble")
    return (
        os.environ.get("TURTLEBOT_SDF")
        or f"/opt/ros/{ros_d}/share/turtlebot3_gazebo/models/turtlebot3_burger/model.sdf"
    )


class ResetSimulation(Node):
    def __init__(self) -> None:
        super().__init__("pfe_reset_simulation")
        self._lock = threading.Lock()
        self._entity = os.environ.get("PFE_ROBOT_ENTITY", "my_robot")
        self._x = float(os.environ.get("SPAWN_X", "-2.0"))
        self._y = float(os.environ.get("SPAWN_Y", "-2.0"))
        self._z = float(os.environ.get("SPAWN_Z", "0.15"))
        self._sdf_path = _default_turtlebot_sdf()
        self._cb = ReentrantCallbackGroup()
        self._set_state = self.create_client(
            SetEntityState, "/set_entity_state", callback_group=self._cb
        )
        self._delete = self.create_client(
            DeleteEntity, "/delete_entity", callback_group=self._cb
        )
        self.create_service(
            Empty, "/reset_simulation", self._on_reset, callback_group=self._cb
        )
        self.get_logger().info(
            f"/reset_simulation ready → {self._entity} at "
            f"({self._x}, {self._y}, {self._z})"
        )

    def _spin_call(self, client, req, timeout: float = 10.0):
        fut = client.call_async(req)
        deadline = time.time() + timeout
        while rclpy.ok() and not fut.done() and time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
        if not fut.done():
            return None
        return fut.result()

    def _teleport(self) -> bool:
        if not self._set_state.wait_for_service(timeout_sec=10.0):
            self.get_logger().warning("/set_entity_state not available yet")
            return False
        state = EntityState()
        state.name = self._entity
        state.reference_frame = "world"
        state.pose = Pose()
        state.pose.position.x = self._x
        state.pose.position.y = self._y
        state.pose.position.z = self._z
        state.pose.orientation = Quaternion(w=1.0)
        state.twist = Twist()
        req = SetEntityState.Request()
        req.state = state
        res = self._spin_call(self._set_state, req, timeout=10.0)
        if res is None or not res.success:
            return False
        time.sleep(0.4)
        return True

    def _respawn_cli(self) -> bool:
        """Delete + spawn via CLI (robust; avoids SpawnEntity XML issues)."""
        if self._delete.wait_for_service(timeout_sec=5.0):
            dreq = DeleteEntity.Request()
            dreq.name = self._entity
            self._spin_call(self._delete, dreq, timeout=8.0)
            time.sleep(0.5)
        if not os.path.isfile(self._sdf_path):
            self.get_logger().error(f"SDF missing: {self._sdf_path}")
            return False
        cmd = [
            "ros2",
            "run",
            "gazebo_ros",
            "spawn_entity.py",
            "-timeout",
            "120",
            "-entity",
            self._entity,
            "-file",
            self._sdf_path,
            "-x",
            str(self._x),
            "-y",
            str(self._y),
            "-z",
            str(self._z),
        ]
        try:
            proc = subprocess.run(
                cmd,
                capture_output=True,
                text=True,
                timeout=90,
                check=False,
            )
        except subprocess.TimeoutExpired:
            self.get_logger().error("spawn_entity.py timed out")
            return False
        if proc.returncode != 0:
            self.get_logger().error(
                f"spawn_entity failed ({proc.returncode}): "
                f"{(proc.stderr or proc.stdout)[-400:]}"
            )
            return False
        time.sleep(1.0)
        return True

    def _on_reset(self, _req: Empty.Request, _res: Empty.Response) -> Empty.Response:
        with self._lock:
            ok = self._teleport()
            if not ok:
                self.get_logger().info("teleport failed — respawning (fresh /odom)")
                ok = self._respawn_cli()
            if ok:
                self.get_logger().info(
                    f"reset OK → ({self._x}, {self._y}, {self._z})"
                )
            else:
                self.get_logger().error(
                    "reset failed — waypoints will not score correctly"
                )
        return _res


def main() -> None:
    rclpy.init()
    node = ResetSimulation()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
