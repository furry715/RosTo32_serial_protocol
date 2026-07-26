#include <algorithm>
#include <cmath>
#include <memory>
#include <string>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <quadrotor_msgs/msg/position_command.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <visualization_msgs/msg/marker.hpp>

namespace {

constexpr double kPi = 3.14159265358979323846;

double clamp(double value, double lower, double upper) {
  return std::max(lower, std::min(value, upper));
}

double wrap_angle(double angle) {
  while (angle > kPi) {
    angle -= 2.0 * kPi;
  }
  while (angle < -kPi) {
    angle += 2.0 * kPi;
  }
  return angle;
}

}  // namespace

class EgoTrackerNode : public rclcpp::Node {
 public:
  EgoTrackerNode() : Node("ego_tracker_node") {
    declare_parameter<std::string>("odom_topic", "/odometry/filtered");
    declare_parameter<std::string>("position_cmd_topic", "/position_cmd");
    declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
    declare_parameter<std::string>("goal_topic", "/move_base_simple/goal");
    declare_parameter<std::string>("goal_reached_topic", "/mission/goal_reached");
    declare_parameter<double>("feedforward_gain", 0.8);
    declare_parameter<double>("px", 1.0);
    declare_parameter<double>("py", 1.0);
    declare_parameter<double>("pz", 1.0);
    declare_parameter<double>("dx", 0.0);
    declare_parameter<double>("dy", 0.0);
    declare_parameter<double>("dz", 0.0);
    declare_parameter<double>("yaw_rate_gain", 2.0);
    declare_parameter<double>("max_yaw_rate_deg", 45.0);
    declare_parameter<bool>("follow_traj_yaw", false);
    declare_parameter<std::string>("goal_yaw_mode", "hold");
    declare_parameter<double>("yaw_done_deg", 5.0);
    declare_parameter<double>("max_vel_xy", 0.5);
    declare_parameter<double>("max_vel_z", 1.0);
    declare_parameter<double>("command_timeout", 0.5);
    declare_parameter<double>("stop_dist", 0.3);
    declare_parameter<double>("reach_dist", 0.25);
    declare_parameter<double>("reach_vel", 0.15);
    declare_parameter<double>("reach_hold_time", 1.0);
    declare_parameter<bool>("reach_check_yaw", true);
    declare_parameter<bool>("goal_hold_requires_position_cmd", true);
    declare_parameter<bool>("auto_takeoff_hover", false);
    declare_parameter<double>("hover_altitude", 1.0);
    declare_parameter<double>("hover_gain", 1.0);

    odom_topic_ = get_parameter("odom_topic").as_string();
    position_cmd_topic_ = get_parameter("position_cmd_topic").as_string();
    cmd_vel_topic_ = get_parameter("cmd_vel_topic").as_string();
    goal_topic_ = get_parameter("goal_topic").as_string();
    goal_reached_topic_ = get_parameter("goal_reached_topic").as_string();

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        odom_topic_, rclcpp::QoS(20),
        std::bind(&EgoTrackerNode::odom_callback, this, std::placeholders::_1));

    position_cmd_sub_ =
        create_subscription<quadrotor_msgs::msg::PositionCommand>(
            position_cmd_topic_, rclcpp::QoS(20),
            std::bind(&EgoTrackerNode::position_cmd_callback, this,
                      std::placeholders::_1));
    goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        goal_topic_, rclcpp::QoS(10),
        std::bind(&EgoTrackerNode::goal_callback, this, std::placeholders::_1));

    cmd_vel_pub_ =
        create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 20);
    goal_reached_pub_ =
        create_publisher<std_msgs::msg::Bool>(goal_reached_topic_, 10);
    trackpoint_pub_ =
        create_publisher<visualization_msgs::msg::Marker>("/track_drone_point", 10);

    control_timer_ = create_wall_timer(
        std::chrono::milliseconds(20),
        std::bind(&EgoTrackerNode::control_loop, this));

    RCLCPP_INFO(get_logger(),
                "ego_tracker_node listening to %s and %s, publishing %s",
                odom_topic_.c_str(), position_cmd_topic_.c_str(),
                cmd_vel_topic_.c_str());
  }

 private:
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    latest_odom_ = msg;
    has_odom_ = true;
    if (!has_initial_pose_) {
      init_x_ = msg->pose.pose.position.x;
      init_y_ = msg->pose.pose.position.y;
      has_initial_pose_ = true;
    }

    tf2::Quaternion quat;
    tf2::fromMsg(msg->pose.pose.orientation, quat);
    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;
    tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    current_yaw_ = yaw;
  }

  void position_cmd_callback(
      const quadrotor_msgs::msg::PositionCommand::SharedPtr msg) {
    latest_cmd_ = msg;
    last_cmd_time_ = now();
    has_cmd_ = true;
    if (has_goal_) {
      has_cmd_for_goal_ = true;
    }
    publish_trackpoint(*msg);
  }

  void goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    latest_goal_ = msg;
    has_goal_ = true;
    has_cmd_for_goal_ = false;
    goal_reached_ = false;
    reach_timer_active_ = false;
    update_target_yaw_from_goal(*msg);
  }

  void control_loop() {
    if (!has_odom_) {
      return;
    }

    geometry_msgs::msg::Twist cmd_vel;
    if (!has_cmd_ || (now() - last_cmd_time_).seconds() >
                         get_parameter("command_timeout").as_double()) {
      const bool can_hold_goal =
          has_goal_ &&
          (!get_parameter("goal_hold_requires_position_cmd").as_bool() ||
           has_cmd_for_goal_);
      if (can_hold_goal) {
        cmd_vel = compute_goal_hold_cmd();
      } else if (get_parameter("auto_takeoff_hover").as_bool() && has_initial_pose_ &&
          !has_goal_) {
        const double hover_gain = get_parameter("hover_gain").as_double();
        const double hover_altitude = get_parameter("hover_altitude").as_double();
        const double dx_world = init_x_ - latest_odom_->pose.pose.position.x;
        const double dy_world = init_y_ - latest_odom_->pose.pose.position.y;
        const double dz_world = hover_altitude - latest_odom_->pose.pose.position.z;
        const double body_error_x =
            dx_world * std::cos(current_yaw_) + dy_world * std::sin(current_yaw_);
        const double body_error_y =
            -dx_world * std::sin(current_yaw_) + dy_world * std::cos(current_yaw_);
        cmd_vel.linear.x = clamp(
            hover_gain * body_error_x,
            -get_parameter("max_vel_xy").as_double(),
            get_parameter("max_vel_xy").as_double());
        cmd_vel.linear.y = clamp(
            hover_gain * body_error_y,
            -get_parameter("max_vel_xy").as_double(),
            get_parameter("max_vel_xy").as_double());
        cmd_vel.linear.z = clamp(
            hover_gain * dz_world,
            -get_parameter("max_vel_z").as_double(),
            get_parameter("max_vel_z").as_double());
      }
      const std::string goal_yaw_mode =
          get_parameter("goal_yaw_mode").as_string();
      if (goal_yaw_mode == "hold") {
        double yaw_error = 0.0;
        double yaw_rate = 0.0;
        if (compute_target_yaw_rate(yaw_error, yaw_rate)) {
          cmd_vel.angular.z = yaw_rate;
        }
      }
      update_goal_reached();
      cmd_vel_pub_->publish(cmd_vel);
      return;
    }

    const auto &odom = latest_odom_;
    const auto &cmd = latest_cmd_;

    const double px = get_parameter("px").as_double();
    const double py = get_parameter("py").as_double();
    const double pz = get_parameter("pz").as_double();
    const double dx = get_parameter("dx").as_double();
    const double dy = get_parameter("dy").as_double();
    const double dz = get_parameter("dz").as_double();
    double dynamic_feedforward_gain =
        get_parameter("feedforward_gain").as_double();

    const double dx_world = cmd->position.x - odom->pose.pose.position.x;
    const double dy_world = cmd->position.y - odom->pose.pose.position.y;
    const double dz_world = cmd->position.z - odom->pose.pose.position.z;

    const double body_error_x =
        dx_world * std::cos(current_yaw_) + dy_world * std::sin(current_yaw_);
    const double body_error_y =
        -dx_world * std::sin(current_yaw_) + dy_world * std::cos(current_yaw_);

    const double feedforward_x =
        cmd->velocity.x * std::cos(current_yaw_) +
        cmd->velocity.y * std::sin(current_yaw_);
    const double feedforward_y =
        -cmd->velocity.x * std::sin(current_yaw_) +
        cmd->velocity.y * std::cos(current_yaw_);

    if (has_goal_) {
      const double dist_to_goal = std::hypot(
          latest_goal_->pose.position.x - odom->pose.pose.position.x,
          latest_goal_->pose.position.y - odom->pose.pose.position.y);
      const double dist_to_track =
          std::hypot(dx_world, dy_world);
      if (dist_to_goal <
          get_parameter("stop_dist").as_double() + dist_to_track) {
        dynamic_feedforward_gain = 0.0;
      }
    }

    double vel_x = dynamic_feedforward_gain * feedforward_x + px * body_error_x -
                   dx * odom->twist.twist.linear.x;
    double vel_y = dynamic_feedforward_gain * feedforward_y + py * body_error_y -
                   dy * odom->twist.twist.linear.y;
    double vel_z = dynamic_feedforward_gain * cmd->velocity.z + pz * dz_world -
                   dz * odom->twist.twist.linear.z;

    const double max_vel_xy = get_parameter("max_vel_xy").as_double();
    const double max_vel_z = get_parameter("max_vel_z").as_double();
    vel_x = clamp(vel_x, -max_vel_xy, max_vel_xy);
    vel_y = clamp(vel_y, -max_vel_xy, max_vel_xy);
    vel_z = clamp(vel_z, -max_vel_z, max_vel_z);

    const bool follow_traj_yaw =
        get_parameter("follow_traj_yaw").as_bool();
    const std::string goal_yaw_mode =
        get_parameter("goal_yaw_mode").as_string();
    double yaw_error = 0.0;
    double yaw_rate = 0.0;
    const double max_yaw_rate =
        get_parameter("max_yaw_rate_deg").as_double() * kPi / 180.0;
    if (follow_traj_yaw) {
      yaw_error = wrap_angle(cmd->yaw - current_yaw_);
      yaw_rate = clamp(
          get_parameter("yaw_rate_gain").as_double() * yaw_error, -max_yaw_rate,
          max_yaw_rate);
    } else if (goal_yaw_mode == "hold") {
      compute_target_yaw_rate(yaw_error, yaw_rate);
      if (is_yaw_done(yaw_error)) {
        yaw_rate = 0.0;
      }
    }

    RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "yaw debug | follow_traj_yaw=%s cmd_frame=%s odom_frame=%s "
        "goal_yaw_mode=%s cmd_yaw=%.3f current_yaw=%.3f "
        "yaw_error=%.3f yaw_rate=%.3f",
        follow_traj_yaw ? "true" : "false",
        cmd->header.frame_id.c_str(),
        odom->header.frame_id.c_str(),
        goal_yaw_mode.c_str(),
        cmd->yaw,
        current_yaw_,
        yaw_error,
        yaw_rate);

    cmd_vel.linear.x = vel_x;
    cmd_vel.linear.y = vel_y;
    cmd_vel.linear.z = vel_z;
    cmd_vel.angular.z = yaw_rate;
    update_goal_reached();
    cmd_vel_pub_->publish(cmd_vel);
  }

  geometry_msgs::msg::Twist compute_goal_hold_cmd() {
    geometry_msgs::msg::Twist cmd_vel;
    if (!latest_goal_ || !latest_odom_) {
      return cmd_vel;
    }

    const double px = get_parameter("px").as_double();
    const double py = get_parameter("py").as_double();
    const double pz = get_parameter("pz").as_double();
    const double dx = get_parameter("dx").as_double();
    const double dy = get_parameter("dy").as_double();
    const double dz = get_parameter("dz").as_double();

    const double dx_world =
        latest_goal_->pose.position.x - latest_odom_->pose.pose.position.x;
    const double dy_world =
        latest_goal_->pose.position.y - latest_odom_->pose.pose.position.y;
    const double dz_world =
        latest_goal_->pose.position.z - latest_odom_->pose.pose.position.z;

    const double body_error_x =
        dx_world * std::cos(current_yaw_) + dy_world * std::sin(current_yaw_);
    const double body_error_y =
        -dx_world * std::sin(current_yaw_) + dy_world * std::cos(current_yaw_);

    const double max_vel_xy = get_parameter("max_vel_xy").as_double();
    const double max_vel_z = get_parameter("max_vel_z").as_double();
    cmd_vel.linear.x =
        clamp(px * body_error_x - dx * latest_odom_->twist.twist.linear.x,
              -max_vel_xy, max_vel_xy);
    cmd_vel.linear.y =
        clamp(py * body_error_y - dy * latest_odom_->twist.twist.linear.y,
              -max_vel_xy, max_vel_xy);
    cmd_vel.linear.z =
        clamp(pz * dz_world - dz * latest_odom_->twist.twist.linear.z,
              -max_vel_z, max_vel_z);

    const std::string goal_yaw_mode =
        get_parameter("goal_yaw_mode").as_string();
    if (goal_yaw_mode == "hold") {
      double yaw_error = 0.0;
      double yaw_rate = 0.0;
      if (compute_target_yaw_rate(yaw_error, yaw_rate) &&
          !is_yaw_done(yaw_error)) {
        cmd_vel.angular.z = yaw_rate;
      }
    }

    return cmd_vel;
  }

  void publish_trackpoint(const quadrotor_msgs::msg::PositionCommand & cmd) {
    visualization_msgs::msg::Marker marker;
    marker.header.stamp = now();
    marker.header.frame_id =
        latest_odom_ ? latest_odom_->header.frame_id : "camera_init";
    marker.ns = "track_drone";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.scale.x = 0.3;
    marker.scale.y = 0.3;
    marker.scale.z = 0.3;
    marker.color.a = 1.0;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.pose.position.x = cmd.position.x;
    marker.pose.position.y = cmd.position.y;
    marker.pose.position.z = cmd.position.z;
    marker.pose.orientation.w = 1.0;
    trackpoint_pub_->publish(marker);
  }

  void update_target_yaw_from_goal(const geometry_msgs::msg::PoseStamped & goal) {
    tf2::Quaternion goal_quat;
    tf2::fromMsg(goal.pose.orientation, goal_quat);
    double goal_roll = 0.0;
    double goal_pitch = 0.0;
    double goal_yaw = 0.0;
    tf2::Matrix3x3(goal_quat).getRPY(goal_roll, goal_pitch, goal_yaw);
    target_yaw_ = goal_yaw;
    has_target_yaw_ = true;
  }

  bool compute_target_yaw_rate(double &yaw_error, double &yaw_rate) {
    if (!has_target_yaw_) {
      yaw_error = 0.0;
      yaw_rate = 0.0;
      return false;
    }

    yaw_error = wrap_angle(target_yaw_ - current_yaw_);
    const double max_yaw_rate =
        get_parameter("max_yaw_rate_deg").as_double() * kPi / 180.0;
    yaw_rate = clamp(
        get_parameter("yaw_rate_gain").as_double() * yaw_error, -max_yaw_rate,
        max_yaw_rate);
    return true;
  }

  bool is_yaw_done(double yaw_error) {
    const double yaw_done =
        get_parameter("yaw_done_deg").as_double() * kPi / 180.0;
    return std::abs(yaw_error) <= yaw_done;
  }

  void update_goal_reached() {
    bool reached_now = false;
    if (has_goal_ && latest_goal_ && latest_odom_) {
      const double ex =
          latest_goal_->pose.position.x - latest_odom_->pose.pose.position.x;
      const double ey =
          latest_goal_->pose.position.y - latest_odom_->pose.pose.position.y;
      const double ez =
          latest_goal_->pose.position.z - latest_odom_->pose.pose.position.z;
      const double pos_error = std::sqrt(ex * ex + ey * ey + ez * ez);
      const auto &vel = latest_odom_->twist.twist.linear;
      const double vel_norm =
          std::sqrt(vel.x * vel.x + vel.y * vel.y + vel.z * vel.z);

      bool yaw_ok = true;
      if (get_parameter("reach_check_yaw").as_bool() && has_target_yaw_) {
        const double yaw_error = wrap_angle(target_yaw_ - current_yaw_);
        yaw_ok = is_yaw_done(yaw_error);
      }

      reached_now =
          pos_error <= get_parameter("reach_dist").as_double() &&
          vel_norm <= get_parameter("reach_vel").as_double() &&
          yaw_ok;
    }

    const auto stamp = now();
    if (reached_now) {
      if (!reach_timer_active_) {
        reach_start_time_ = stamp;
        reach_timer_active_ = true;
      }
      goal_reached_ =
          (stamp - reach_start_time_).seconds() >=
          get_parameter("reach_hold_time").as_double();
    } else {
      reach_timer_active_ = false;
      goal_reached_ = false;
    }

    std_msgs::msg::Bool msg;
    msg.data = goal_reached_;
    goal_reached_pub_->publish(msg);
  }

  std::string odom_topic_;
  std::string position_cmd_topic_;
  std::string cmd_vel_topic_;
  std::string goal_topic_;
  std::string goal_reached_topic_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<quadrotor_msgs::msg::PositionCommand>::SharedPtr
      position_cmd_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr goal_reached_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr trackpoint_pub_;
  rclcpp::TimerBase::SharedPtr control_timer_;

  nav_msgs::msg::Odometry::SharedPtr latest_odom_;
  quadrotor_msgs::msg::PositionCommand::SharedPtr latest_cmd_;
  geometry_msgs::msg::PoseStamped::SharedPtr latest_goal_;
  rclcpp::Time last_cmd_time_{0, 0, RCL_ROS_TIME};

  bool has_odom_{false};
  bool has_cmd_{false};
  bool has_goal_{false};
  bool has_initial_pose_{false};
  bool has_target_yaw_{false};
  bool has_cmd_for_goal_{false};
  bool goal_reached_{false};
  bool reach_timer_active_{false};
  double current_yaw_{0.0};
  double target_yaw_{0.0};
  double init_x_{0.0};
  double init_y_{0.0};
  rclcpp::Time reach_start_time_{0, 0, RCL_ROS_TIME};
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EgoTrackerNode>());
  rclcpp::shutdown();
  return 0;
}
