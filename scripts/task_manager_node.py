#!/usr/bin/env python3

import os
from typing import Any, Dict

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String, UInt8
import yaml


class TaskManagerNode(Node):
    MISSION_ACTIVE_STATES = {
        "ARMING",
        "ARM_IDLE",
        "SEND_GOAL",
        "NAVIGATING",
        "WAIT_GATES",
        "LANDING",
        "DISARMING",
    }

    def __init__(self):
        super().__init__("task_manager_node")
        self.declare_parameter("task_map_file", "")
        self.declare_parameter("task_request_topic", "/fc/task_request")
        self.declare_parameter("mission_load_file_topic", "/mission/load_file")
        self.declare_parameter("mission_start_topic", "/mission/start")
        self.declare_parameter("mission_cancel_topic", "/mission/cancel")
        self.declare_parameter("mission_state_topic", "/mission/state")
        self.declare_parameter("mission_current_waypoint_topic", "/mission/current_waypoint")
        self.declare_parameter("task_state_topic", "/task/state")
        self.declare_parameter("task_current_topic", "/task/current_task")
        self.declare_parameter("task_running_topic", "/task/running")
        self.declare_parameter("start_delay", 0.5)
        self.declare_parameter("feedback_hz", 2.0)

        self.task_map_file = self.get_parameter("task_map_file").as_string()
        self.task_map = self._load_task_map(self.task_map_file)

        self.mission_load_pub = self.create_publisher(
            String, self.get_parameter("mission_load_file_topic").as_string(), 10
        )
        self.mission_start_pub = self.create_publisher(
            Bool, self.get_parameter("mission_start_topic").as_string(), 10
        )
        self.mission_cancel_pub = self.create_publisher(
            Bool, self.get_parameter("mission_cancel_topic").as_string(), 10
        )
        self.task_state_pub = self.create_publisher(
            String, self.get_parameter("task_state_topic").as_string(), 10
        )
        self.task_current_pub = self.create_publisher(
            UInt8, self.get_parameter("task_current_topic").as_string(), 10
        )
        self.task_running_pub = self.create_publisher(
            Bool, self.get_parameter("task_running_topic").as_string(), 10
        )

        self.task_request_sub = self.create_subscription(
            UInt8,
            self.get_parameter("task_request_topic").as_string(),
            self._task_request_callback,
            10,
        )
        self.mission_state_sub = self.create_subscription(
            String,
            self.get_parameter("mission_state_topic").as_string(),
            self._mission_state_callback,
            10,
        )
        self.current_waypoint_sub = self.create_subscription(
            String,
            self.get_parameter("mission_current_waypoint_topic").as_string(),
            self._current_waypoint_callback,
            10,
        )

        self.active_task_id = 0
        self.task_state = "IDLE"
        self.mission_state = "IDLE"
        self.current_waypoint = ""
        self.current_waypoint_index = 0
        self.pending_start = False
        self.pending_start_at = None

        feedback_hz = max(0.1, float(self.get_parameter("feedback_hz").value))
        self.timer = self.create_timer(1.0 / feedback_hz, self._tick)

        self.get_logger().info(
            f"Loaded {len(self.task_map)} task mappings from {self.task_map_file}"
        )

    def _load_task_map(self, path: str) -> Dict[int, Dict[str, Any]]:
        if not path:
            raise RuntimeError("task_map_file parameter is required")
        with open(path, "r", encoding="utf-8") as f:
            raw = yaml.safe_load(f) or {}

        base_dir = os.path.dirname(os.path.abspath(path))
        tasks = raw.get("tasks", {})
        parsed = {}
        for key, value in tasks.items():
            task_id = int(key)
            if isinstance(value, str):
                mission_file = value
                name = f"task_{task_id}"
            else:
                name = str(value.get("name", f"task_{task_id}"))
                mission_file = str(value.get("mission_file", ""))
            if mission_file and not os.path.isabs(mission_file):
                mission_file = os.path.join(base_dir, mission_file)
            parsed[task_id] = {"name": name, "mission_file": mission_file}
        return parsed

    def _task_request_callback(self, msg: UInt8):
        task_id = int(msg.data)
        if self.mission_state in self.MISSION_ACTIVE_STATES:
            self.get_logger().warn(
                f"Reject task {task_id}: mission is active ({self.mission_state})"
            )
            return

        task = self.task_map.get(task_id)
        if task is None or not task.get("mission_file"):
            self.get_logger().error(f"Unknown task id: {task_id}")
            return

        self.active_task_id = task_id
        self.task_state = "LOADING"
        self.current_waypoint = ""
        self.current_waypoint_index = 0
        self.pending_start = True
        self.pending_start_at = self.get_clock().now()

        load_msg = String()
        load_msg.data = task["mission_file"]
        self.mission_load_pub.publish(load_msg)
        self.get_logger().info(
            f"Task {task_id} ({task['name']}) selected: {task['mission_file']}"
        )

    def _mission_state_callback(self, msg: String):
        self.mission_state = msg.data
        if self.active_task_id == 0:
            return
        if msg.data in self.MISSION_ACTIVE_STATES:
            self.task_state = "RUNNING"
        elif msg.data == "FINISHED":
            self.task_state = "FINISHED"
        elif msg.data == "ERROR":
            self.task_state = "ERROR"
        elif msg.data == "CANCELED":
            self.task_state = "CANCELED"

    def _current_waypoint_callback(self, msg: String):
        if msg.data and msg.data != self.current_waypoint:
            self.current_waypoint_index = min(self.current_waypoint_index + 1, 255)
        self.current_waypoint = msg.data

    def _tick(self):
        if self.pending_start and self.pending_start_at is not None:
            elapsed = (self.get_clock().now() - self.pending_start_at).nanoseconds * 1e-9
            if elapsed >= float(self.get_parameter("start_delay").value):
                start_msg = Bool()
                start_msg.data = True
                self.mission_start_pub.publish(start_msg)
                self.pending_start = False
                self.pending_start_at = None
                self.task_state = "RUNNING"
                self.get_logger().info(f"Task {self.active_task_id} start requested")

        self._publish_status()

    def _publish_status(self):
        state_msg = String()
        state_msg.data = self.task_state
        self.task_state_pub.publish(state_msg)

        task_msg = UInt8()
        task_msg.data = self.active_task_id
        self.task_current_pub.publish(task_msg)

        running_msg = Bool()
        running_msg.data = self.task_state in ("LOADING", "RUNNING")
        self.task_running_pub.publish(running_msg)


def main():
    rclpy.init()
    node = TaskManagerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
