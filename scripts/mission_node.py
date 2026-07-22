#!/usr/bin/env python3

import math
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
import yaml


@dataclass
class Gate:
    gate_type: str
    topic: str = ""
    value: Any = None
    duration: float = 0.0
    timeout: float = 0.0
    on_timeout: str = "error"
    done: bool = False
    started_at: Optional[rclpy.time.Time] = None
    subscription: Any = None


@dataclass
class Waypoint:
    name: str
    x: float
    y: float
    z: float
    yaw_deg: float
    gates: List[Gate] = field(default_factory=list)


class MissionNode(Node):
    def __init__(self):
        super().__init__("mission_node")
        self.declare_parameter("mission_file", "")
        self.declare_parameter("goal_topic", "/move_base_simple/goal")
        self.declare_parameter("goal_reached_topic", "/mission/goal_reached")
        self.declare_parameter("state_topic", "/mission/state")
        self.declare_parameter("current_waypoint_topic", "/mission/current_waypoint")
        self.declare_parameter("gate_status_topic", "/mission/gate_status")
        self.declare_parameter("auto_start", True)
        self.declare_parameter("tick_hz", 10.0)

        self.mission_file = self.get_parameter("mission_file").value
        self.goal_topic = self.get_parameter("goal_topic").value
        self.auto_start = bool(self.get_parameter("auto_start").value)

        self.config = self._load_mission(self.mission_file)
        self.frame_id = self.config.get("mission", {}).get("frame_id", "camera_init")
        self.auto_arm = bool(self.config.get("mission", {}).get("auto_arm", False))
        self.auto_land = bool(self.config.get("mission", {}).get("auto_land", False))
        self.auto_disarm = bool(self.config.get("mission", {}).get("auto_disarm", False))
        self.arm_idle_duration = float(
            self.config.get("mission", {}).get("arm_idle_duration", 0.0)
        )
        self.reach_timeout = float(
            self.config.get("mission", {}).get("default_reach_timeout", 30.0)
        )
        self.waypoints = self._parse_waypoints(self.config.get("waypoints", []))

        self.goal_pub = self.create_publisher(PoseStamped, self.goal_topic, 10)
        self.state_pub = self.create_publisher(
            String, self.get_parameter("state_topic").value, 10
        )
        self.current_waypoint_pub = self.create_publisher(
            String, self.get_parameter("current_waypoint_topic").value, 10
        )
        self.gate_status_pub = self.create_publisher(
            String, self.get_parameter("gate_status_topic").value, 10
        )

        self.goal_reached = False
        self.goal_reached_sub = self.create_subscription(
            Bool,
            self.get_parameter("goal_reached_topic").value,
            self._goal_reached_callback,
            10,
        )

        self.arm_client = self.create_client(Trigger, "/serial_protocol_node/arm")
        self.land_client = self.create_client(Trigger, "/serial_protocol_node/land")
        self.disarm_client = self.create_client(Trigger, "/serial_protocol_node/disarm")

        self.state = "IDLE"
        self.wp_index = 0
        self.state_started_at = self.get_clock().now()
        self.active_future = None
        self.active_service_name = ""
        self.gates_active = False
        self.gates_started_at: Optional[rclpy.time.Time] = None

        tick_hz = float(self.get_parameter("tick_hz").value)
        self.timer = self.create_timer(1.0 / tick_hz, self._tick)

        self.get_logger().info(
            f"Loaded {len(self.waypoints)} waypoints from {self.mission_file}"
        )
        if not self.waypoints:
            self._transition("ERROR")
            self.get_logger().error("Mission file has no waypoints")
            return
        if self.auto_start:
            self._transition("ARMING" if self.auto_arm else "SEND_GOAL")

    def _load_mission(self, path: str) -> Dict[str, Any]:
        if not path:
            raise RuntimeError("mission_file parameter is required")
        with open(path, "r", encoding="utf-8") as f:
            return yaml.safe_load(f) or {}

    def _parse_waypoints(self, raw_waypoints: List[Dict[str, Any]]) -> List[Waypoint]:
        waypoints = []
        for i, raw in enumerate(raw_waypoints):
            pose = raw.get("pose", {})
            gates = [self._parse_gate(g) for g in raw.get("gates", [])]
            waypoints.append(
                Waypoint(
                    name=str(raw.get("name", f"waypoint_{i}")),
                    x=float(pose.get("x", 0.0)),
                    y=float(pose.get("y", 0.0)),
                    z=float(pose.get("z", 1.0)),
                    yaw_deg=float(pose.get("yaw_deg", 0.0)),
                    gates=gates,
                )
            )
        return waypoints

    def _parse_gate(self, raw: Dict[str, Any]) -> Gate:
        return Gate(
            gate_type=str(raw.get("type", "hold")),
            topic=str(raw.get("topic", "")),
            value=raw.get("value", None),
            duration=float(raw.get("duration", 0.0)),
            timeout=float(raw.get("timeout", 0.0)),
            on_timeout=str(raw.get("on_timeout", "error")),
        )

    def _goal_reached_callback(self, msg: Bool):
        self.goal_reached = msg.data

    def _tick(self):
        self._publish_status()

        if self.state == "IDLE":
            return
        if self.state == "ARMING":
            self._handle_service_state(
                self.arm_client, "/serial_protocol_node/arm", self._after_arm_state()
            )
            return
        if self.state == "ARM_IDLE":
            if self._state_age() >= self.arm_idle_duration:
                self._transition("SEND_GOAL")
            return
        if self.state == "SEND_GOAL":
            self._send_current_goal()
            self._transition("NAVIGATING")
            return
        if self.state == "NAVIGATING":
            self._handle_navigating()
            return
        if self.state == "WAIT_GATES":
            self._handle_gates()
            return
        if self.state == "LANDING":
            self._handle_service_state(
                self.land_client,
                "/serial_protocol_node/land",
                "DISARMING" if self.auto_disarm else "FINISHED",
            )
            return
        if self.state == "DISARMING":
            self._handle_service_state(
                self.disarm_client, "/serial_protocol_node/disarm", "FINISHED"
            )
            return

    def _handle_service_state(self, client, service_name: str, next_state: str):
        if self.active_future is None:
            if not client.wait_for_service(timeout_sec=0.0):
                if self._state_age() > 5.0:
                    self._error(f"Service not available: {service_name}")
                return
            self.active_service_name = service_name
            self.active_future = client.call_async(Trigger.Request())
            self.get_logger().info(f"Calling {service_name}")
            return

        if not self.active_future.done():
            return

        try:
            result = self.active_future.result()
            if not result.success:
                self._error(f"{service_name} failed: {result.message}")
                return
        except Exception as exc:
            self._error(f"{service_name} exception: {exc}")
            return

        self.active_future = None
        self.active_service_name = ""
        self._transition(next_state)

    def _after_arm_state(self) -> str:
        return "ARM_IDLE" if self.arm_idle_duration > 0.0 else "SEND_GOAL"

    def _handle_navigating(self):
        if self.goal_reached:
            self.get_logger().info(f"Reached {self._current_wp().name}")
            self._transition("WAIT_GATES")
            return
        if self._state_age() > self.reach_timeout:
            self._error(f"Reach timeout at {self._current_wp().name}")

    def _handle_gates(self):
        if not self.gates_active:
            self._start_gates()

        if not self._current_wp().gates:
            self._advance_waypoint()
            return

        now = self.get_clock().now()
        all_done = True
        for gate in self._current_wp().gates:
            if gate.done:
                continue
            all_done = False
            if gate.gate_type == "hold":
                if gate.started_at is None:
                    gate.started_at = now
                if (now - gate.started_at).nanoseconds * 1e-9 >= gate.duration:
                    gate.done = True
                    self.get_logger().info(f"Gate done: hold {gate.duration}s")
                    continue

            if gate.timeout > 0.0 and self.gates_started_at is not None:
                elapsed = (now - self.gates_started_at).nanoseconds * 1e-9
                if elapsed >= gate.timeout:
                    self._handle_gate_timeout(gate)
                    return

        if all_done or all(g.done for g in self._current_wp().gates):
            self._advance_waypoint()

    def _start_gates(self):
        self.gates_active = True
        self.gates_started_at = self.get_clock().now()
        for gate in self._current_wp().gates:
            gate.done = False
            gate.started_at = None
            if gate.gate_type == "topic_string":
                gate.subscription = self.create_subscription(
                    String,
                    gate.topic,
                    lambda msg, g=gate: self._topic_string_callback(g, msg),
                    10,
                )
            elif gate.gate_type == "topic_bool":
                gate.subscription = self.create_subscription(
                    Bool,
                    gate.topic,
                    lambda msg, g=gate: self._topic_bool_callback(g, msg),
                    10,
                )
            elif gate.gate_type != "hold":
                self._error(f"Unsupported gate type: {gate.gate_type}")
                return
        self.get_logger().info(f"Waiting gates at {self._current_wp().name}")

    def _topic_string_callback(self, gate: Gate, msg: String):
        if str(msg.data) == str(gate.value):
            gate.done = True
            self.get_logger().info(f"Gate done: {gate.topic} == {gate.value}")

    def _topic_bool_callback(self, gate: Gate, msg: Bool):
        expected = self._as_bool(gate.value)
        if msg.data == expected:
            gate.done = True
            self.get_logger().info(f"Gate done: {gate.topic} == {expected}")

    def _as_bool(self, value: Any) -> bool:
        if isinstance(value, bool):
            return value
        if isinstance(value, str):
            return value.strip().lower() in ("1", "true", "yes", "on")
        return bool(value)

    def _handle_gate_timeout(self, gate: Gate):
        msg = f"Gate timeout at {self._current_wp().name}: {gate.gate_type} {gate.topic}"
        if gate.on_timeout == "skip":
            self.get_logger().warn(msg + ", skipping gate")
            gate.done = True
            return
        if gate.on_timeout == "next":
            self.get_logger().warn(msg + ", going next waypoint")
            self._advance_waypoint()
            return
        if gate.on_timeout == "land":
            self.get_logger().error(msg + ", landing")
            self._transition("LANDING")
            return
        self._error(msg)

    def _advance_waypoint(self):
        self._cleanup_gates()
        self.wp_index += 1
        if self.wp_index >= len(self.waypoints):
            self._transition("LANDING" if self.auto_land else "FINISHED")
            return
        self._transition("SEND_GOAL")

    def _cleanup_gates(self):
        if self.wp_index < len(self.waypoints):
            for gate in self._current_wp().gates:
                if gate.subscription is not None:
                    self.destroy_subscription(gate.subscription)
                    gate.subscription = None
        self.gates_active = False
        self.gates_started_at = None

    def _send_current_goal(self):
        wp = self._current_wp()
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.pose.position.x = wp.x
        msg.pose.position.y = wp.y
        msg.pose.position.z = wp.z
        yaw = math.radians(wp.yaw_deg)
        msg.pose.orientation.z = math.sin(yaw * 0.5)
        msg.pose.orientation.w = math.cos(yaw * 0.5)
        self.goal_reached = False
        self.goal_pub.publish(msg)
        self.get_logger().info(
            f"Sent goal {wp.name}: x={wp.x:.2f}, y={wp.y:.2f}, z={wp.z:.2f}, yaw={wp.yaw_deg:.1f}"
        )

    def _current_wp(self) -> Waypoint:
        return self.waypoints[self.wp_index]

    def _transition(self, state: str):
        self.state = state
        self.state_started_at = self.get_clock().now()
        self.active_future = None
        if state in ("SEND_GOAL", "FINISHED", "ERROR"):
            self._cleanup_gates()
        self.get_logger().info(f"Mission state: {state}")

    def _state_age(self) -> float:
        return (self.get_clock().now() - self.state_started_at).nanoseconds * 1e-9

    def _error(self, message: str):
        self.get_logger().error(message)
        self._transition("ERROR")

    def _publish_status(self):
        state_msg = String()
        state_msg.data = self.state
        self.state_pub.publish(state_msg)

        wp_msg = String()
        wp_msg.data = self._current_wp().name if self.wp_index < len(self.waypoints) else ""
        self.current_waypoint_pub.publish(wp_msg)

        gate_msg = String()
        if self.wp_index < len(self.waypoints):
            parts = []
            for gate in self._current_wp().gates:
                label = gate.gate_type
                if gate.topic:
                    label += f":{gate.topic}"
                parts.append(f"{label}={'done' if gate.done else 'waiting'}")
            gate_msg.data = ",".join(parts)
        self.gate_status_pub.publish(gate_msg)


def main():
    rclpy.init()
    node = MissionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
