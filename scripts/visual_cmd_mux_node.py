#!/usr/bin/env python3

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from rclpy.node import Node
from std_msgs.msg import Bool, String


class VisualCmdMuxNode(Node):
    def __init__(self):
        super().__init__("visual_cmd_mux_node")

        self.declare_parameter("ego_cmd_topic", "/ego_cmd_vel")
        self.declare_parameter("visual_cmd_topic", "/visual_track/suggested_cmd_vel")
        self.declare_parameter("visual_seen_topic", "/visual_track/target_seen")
        self.declare_parameter("visual_state_topic", "/visual_track/state")
        self.declare_parameter("mission_current_waypoint_topic", "/mission/current_waypoint")
        self.declare_parameter("visual_disabled_waypoints", "return_home_xy,landing_approach")
        self.declare_parameter("goal_topic", "/move_base_simple/goal")
        self.declare_parameter("output_cmd_topic", "/cmd_vel")
        self.declare_parameter("output_rate_hz", 50.0)
        self.declare_parameter("visual_timeout", 0.2)
        self.declare_parameter("ego_timeout", 0.3)
        self.declare_parameter("xy_accel_limit", 0.25)
        self.declare_parameter("use_visual_xy_only", True)
        self.declare_parameter("require_visual_seen", True)
        self.declare_parameter("hold_current_after_visual_loss", True)
        self.declare_parameter("reset_visual_latch_state", "IDLE")

        self.ego_cmd = Twist()
        self.visual_cmd = Twist()
        self.visual_seen = False
        self.visual_state = "NO_TARGET"
        self.current_waypoint = ""
        self.last_ego_time = None
        self.last_visual_cmd_time = None
        self.last_visual_seen_time = None
        self.last_mode = ""
        self.visual_has_taken_over = False
        self.last_output_cmd = None
        self.last_publish_time = None

        self.ego_sub = self.create_subscription(
            Twist,
            self.get_parameter("ego_cmd_topic").value,
            self.ego_cmd_callback,
            20,
        )
        self.visual_cmd_sub = self.create_subscription(
            Twist,
            self.get_parameter("visual_cmd_topic").value,
            self.visual_cmd_callback,
            20,
        )
        self.visual_seen_sub = self.create_subscription(
            Bool,
            self.get_parameter("visual_seen_topic").value,
            self.visual_seen_callback,
            20,
        )
        self.visual_state_sub = self.create_subscription(
            String,
            self.get_parameter("visual_state_topic").value,
            self.visual_state_callback,
            20,
        )
        self.current_waypoint_sub = self.create_subscription(
            String,
            self.get_parameter("mission_current_waypoint_topic").value,
            self.current_waypoint_callback,
            10,
        )
        self.goal_sub = self.create_subscription(
            PoseStamped,
            self.get_parameter("goal_topic").value,
            self.goal_callback,
            10,
        )
        self.cmd_pub = self.create_publisher(
            Twist,
            self.get_parameter("output_cmd_topic").value,
            20,
        )

        rate = max(1.0, float(self.get_parameter("output_rate_hz").value))
        self.timer = self.create_timer(1.0 / rate, self.publish_cmd)

        self.get_logger().info(
            "visual_cmd_mux ready: "
            f"ego={self.get_parameter('ego_cmd_topic').value}, "
            f"visual={self.get_parameter('visual_cmd_topic').value}, "
            f"seen={self.get_parameter('visual_seen_topic').value}, "
            f"disabled_waypoints={sorted(self.visual_disabled_waypoints())}, "
            f"goal={self.get_parameter('goal_topic').value}, "
            f"xy_accel_limit={self.get_parameter('xy_accel_limit').value}, "
            f"output={self.get_parameter('output_cmd_topic').value}"
        )

    def ego_cmd_callback(self, msg):
        self.ego_cmd = msg
        self.last_ego_time = self.get_clock().now()

    def visual_cmd_callback(self, msg):
        self.visual_cmd = msg
        self.last_visual_cmd_time = self.get_clock().now()

    def visual_seen_callback(self, msg):
        self.visual_seen = bool(msg.data)
        self.last_visual_seen_time = self.get_clock().now()

    def visual_state_callback(self, msg):
        self.visual_state = msg.data
        if msg.data == self.get_parameter("reset_visual_latch_state").value:
            self.visual_has_taken_over = False

    def current_waypoint_callback(self, msg):
        self.current_waypoint = msg.data

    def goal_callback(self, msg):
        self.visual_has_taken_over = False
        self.get_logger().info(
            "Goal received; release visual-loss XY zero hold: "
            f"x={msg.pose.position.x:.3f}, y={msg.pose.position.y:.3f}, "
            f"z={msg.pose.position.z:.3f}"
        )

    def is_fresh(self, stamp, timeout):
        if stamp is None:
            return False
        return (self.get_clock().now() - stamp).nanoseconds * 1e-9 <= timeout

    def visual_is_active(self):
        if self.visual_is_disabled_by_waypoint():
            return False
        visual_timeout = float(self.get_parameter("visual_timeout").value)
        if not self.is_fresh(self.last_visual_cmd_time, visual_timeout):
            return False
        if not bool(self.get_parameter("require_visual_seen").value):
            return True
        return (
            self.visual_seen
            and self.is_fresh(self.last_visual_seen_time, visual_timeout)
        )

    def visual_is_disabled_by_waypoint(self):
        return self.current_waypoint in self.visual_disabled_waypoints()

    def visual_disabled_waypoints(self):
        raw = self.get_parameter("visual_disabled_waypoints").value
        if isinstance(raw, str):
            names = raw.split(",")
        else:
            names = raw
        return {str(name).strip() for name in names if str(name).strip()}

    def ego_is_fresh(self):
        return self.is_fresh(
            self.last_ego_time,
            float(self.get_parameter("ego_timeout").value),
        )

    def copy_twist(self, msg):
        cmd = Twist()
        cmd.linear.x = msg.linear.x
        cmd.linear.y = msg.linear.y
        cmd.linear.z = msg.linear.z
        cmd.angular.x = msg.angular.x
        cmd.angular.y = msg.angular.y
        cmd.angular.z = msg.angular.z
        return cmd

    def limit_axis_step(self, current, target, max_step):
        delta = target - current
        if delta > max_step:
            return current + max_step
        if delta < -max_step:
            return current - max_step
        return target

    def smooth_xy(self, desired):
        now = self.get_clock().now()
        if self.last_output_cmd is None or self.last_publish_time is None:
            self.last_output_cmd = self.copy_twist(desired)
            self.last_publish_time = now
            return desired

        dt = (now - self.last_publish_time).nanoseconds * 1e-9
        self.last_publish_time = now
        if dt <= 0.0 or dt > 0.5:
            dt = 1.0 / max(1.0, float(self.get_parameter("output_rate_hz").value))

        accel_limit = max(0.0, float(self.get_parameter("xy_accel_limit").value))
        if accel_limit <= 0.0:
            self.last_output_cmd = self.copy_twist(desired)
            return desired

        smoothed = self.copy_twist(desired)
        max_step = accel_limit * dt
        smoothed.linear.x = self.limit_axis_step(
            self.last_output_cmd.linear.x,
            desired.linear.x,
            max_step,
        )
        smoothed.linear.y = self.limit_axis_step(
            self.last_output_cmd.linear.y,
            desired.linear.y,
            max_step,
        )
        self.last_output_cmd = self.copy_twist(smoothed)
        return smoothed

    def publish_cmd(self):
        output = Twist()
        ego_fresh = self.ego_is_fresh()

        if ego_fresh:
            output = self.copy_twist(self.ego_cmd)

        visual_active = self.visual_is_active()
        visual_disabled = self.visual_is_disabled_by_waypoint()
        if visual_active:
            self.visual_has_taken_over = True
            if bool(self.get_parameter("use_visual_xy_only").value):
                output.linear.x = self.visual_cmd.linear.x
                output.linear.y = self.visual_cmd.linear.y
            else:
                output = self.copy_twist(self.visual_cmd)
            mode = f"VISUAL_XY ({self.visual_state})"
        elif visual_disabled and ego_fresh:
            self.visual_has_taken_over = False
            mode = f"EGO_PRIORITY ({self.current_waypoint})"
        elif (
            self.visual_has_taken_over
            and bool(self.get_parameter("hold_current_after_visual_loss").value)
        ):
            output.linear.x = 0.0
            output.linear.y = 0.0
            mode = f"LOST_ZERO_XY ({self.visual_state})"
        elif ego_fresh:
            mode = "EGO_HOLD"
        else:
            mode = "ZERO"

        output = self.smooth_xy(output)
        self.cmd_pub.publish(output)
        if mode != self.last_mode:
            self.last_mode = mode
            self.get_logger().info(
                f"cmd mux mode: {mode}, "
                f"cmd=({output.linear.x:.3f}, {output.linear.y:.3f}, "
                f"{output.linear.z:.3f}, yaw={output.angular.z:.3f})"
            )


def main(args=None):
    rclpy.init(args=args)
    node = VisualCmdMuxNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
