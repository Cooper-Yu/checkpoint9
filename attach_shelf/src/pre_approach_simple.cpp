#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

using namespace std::chrono_literals;

class PreApproachSimple : public rclcpp::Node
{
public:
  PreApproachSimple()
      : Node("pre_approach_simple"),
        obstacle_(0.4),
        degrees_(-90.0),
        forward_speed_(0.4),
        angular_speed_(0.5),
        rotation_scale_(0.5),
        planned_drive_distance_(0.0),
        planned_drive_time_(0.0),
        planned_rotate_time_(0.0),
        state_(State::WAITING_FOR_FIRST_SCAN),
        last_logged_state_(State::WAITING_FOR_FIRST_SCAN),
        shutdown_requested_(false)
  {
    declare_parameter<double>("obstacle", obstacle_);
    declare_parameter<double>("degrees", degrees_);
    declare_parameter<double>("forward_speed", forward_speed_);
    declare_parameter<double>("angular_speed", angular_speed_);
    declare_parameter<double>("rotation_scale", rotation_scale_);

    obstacle_ = get_parameter("obstacle").as_double();
    degrees_ = get_parameter("degrees").as_double();
    forward_speed_ = get_parameter("forward_speed").as_double();
    angular_speed_ = get_parameter("angular_speed").as_double();
    rotation_scale_ = get_parameter("rotation_scale").as_double();

    cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", rclcpp::SensorDataQoS(),
        std::bind(&PreApproachSimple::scan_callback, this, std::placeholders::_1));
    control_timer_ =
        create_wall_timer(100ms, std::bind(&PreApproachSimple::timer_callback, this));

    RCLCPP_INFO(get_logger(),
                "pre_approach_simple started: obstacle=%.2f m, degrees=%.2f, "
                "forward_speed=%.2f m/s, angular_speed=%.2f rad/s, rotation_scale=%.2f",
                obstacle_, degrees_, forward_speed_, angular_speed_, rotation_scale_);
  }

private:
  enum class State
  {
    WAITING_FOR_FIRST_SCAN,
    MOVING_OPEN_LOOP,
    STOP_BEFORE_ROTATE,
    ROTATING_OPEN_LOOP,
    DONE,
    SAFE_STOP
  };

  static constexpr double kPi = 3.14159265358979323846;

  bool get_front_distance(const sensor_msgs::msg::LaserScan & scan, double window_degrees,
                          double & front_distance)
  {
    std::vector<double> valid_ranges;
    const double half_window = window_degrees / 2.0 * kPi / 180.0;

    for (double angle = -half_window; angle <= half_window; angle += scan.angle_increment) {
      const int index = static_cast<int>(std::round((angle - scan.angle_min) / scan.angle_increment));
      if (index < 0 || index >= static_cast<int>(scan.ranges.size())) {
        continue;
      }

      const double distance = scan.ranges[index];
      if (std::isfinite(distance) && distance >= scan.range_min && distance <= scan.range_max) {
        valid_ranges.push_back(distance);
      }
    }

    if (valid_ranges.empty()) {
      return false;
    }

    front_distance = *std::min_element(valid_ranges.begin(), valid_ranges.end());
    return true;
  }

  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    if (first_front_distance_.has_value() || state_ != State::WAITING_FOR_FIRST_SCAN) {
      return;
    }

    double front_distance = 0.0;
    if (!get_front_distance(*msg, 10.0, front_distance)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                           "Waiting for first valid front scan window");
      return;
    }

    first_front_distance_ = front_distance;
    planned_drive_distance_ = std::max(front_distance - obstacle_, 0.0);

    if (forward_speed_ <= 0.0) {
      enter_safe_stop("forward_speed must be positive");
      return;
    }
    planned_drive_time_ = planned_drive_distance_ / forward_speed_;

    const double target_angle = degrees_ * kPi / 180.0;
    if (std::abs(target_angle) > 1e-6) {
      if (std::abs(angular_speed_) < 1e-6 || rotation_scale_ <= 0.0) {
        enter_safe_stop("angular_speed and rotation_scale must be valid for rotation");
        return;
      }
      planned_rotate_time_ = std::abs(target_angle) / std::abs(angular_speed_) * rotation_scale_;
    }

    RCLCPP_INFO(get_logger(),
                "First valid front scan captured: front_distance=%.3f m, obstacle=%.3f m, "
                "planned_drive_distance=%.3f m, planned_drive_time=%.3f s, rotate_time=%.3f s",
                front_distance, obstacle_, planned_drive_distance_, planned_drive_time_,
                planned_rotate_time_);

    move_start_time_ = now();
    set_state(State::MOVING_OPEN_LOOP, "first scan captured; using planned open-loop distance");
  }

  void timer_callback()
  {
    log_state_if_changed();

    switch (state_) {
      case State::WAITING_FOR_FIRST_SCAN:
        publish_stop();
        return;

      case State::MOVING_OPEN_LOOP: {
        const double elapsed = (now() - move_start_time_).seconds();
        if (elapsed < planned_drive_time_) {
          publish_forward();
          return;
        }

        publish_stop();
        stop_start_time_ = now();
        set_state(State::STOP_BEFORE_ROTATE, "planned drive time complete");
        return;
      }

      case State::STOP_BEFORE_ROTATE: {
        publish_stop();
        if ((now() - stop_start_time_).seconds() < 0.2) {
          return;
        }

        if (std::abs(degrees_) < 1e-6) {
          set_state(State::DONE, "degrees is zero");
          return;
        }

        rotate_start_time_ = now();
        set_state(State::ROTATING_OPEN_LOOP, "settling stop complete");
        return;
      }

      case State::ROTATING_OPEN_LOOP: {
        const double elapsed = (now() - rotate_start_time_).seconds();
        if (elapsed < planned_rotate_time_) {
          publish_rotate();
          return;
        }

        publish_stop();
        set_state(State::DONE, "planned rotation complete");
        return;
      }

      case State::DONE:
        publish_stop();
        request_shutdown("pre_approach_simple complete");
        return;

      case State::SAFE_STOP:
        publish_stop();
        return;
    }
  }

  const char * state_name(State state) const
  {
    switch (state) {
      case State::WAITING_FOR_FIRST_SCAN:
        return "WAITING_FOR_FIRST_SCAN";
      case State::MOVING_OPEN_LOOP:
        return "MOVING_OPEN_LOOP";
      case State::STOP_BEFORE_ROTATE:
        return "STOP_BEFORE_ROTATE";
      case State::ROTATING_OPEN_LOOP:
        return "ROTATING_OPEN_LOOP";
      case State::DONE:
        return "DONE";
      case State::SAFE_STOP:
        return "SAFE_STOP";
    }
    return "UNKNOWN";
  }

  void set_state(State next_state, const std::string & reason)
  {
    if (state_ == next_state) {
      return;
    }
    RCLCPP_INFO(get_logger(), "State transition: %s -> %s (%s)", state_name(state_),
                state_name(next_state), reason.c_str());
    state_ = next_state;
  }

  void log_state_if_changed()
  {
    if (last_logged_state_ == state_) {
      return;
    }
    RCLCPP_INFO(get_logger(), "Current state: %s", state_name(state_));
    last_logged_state_ = state_;
  }

  void publish_stop()
  {
    geometry_msgs::msg::Twist cmd;
    cmd_vel_pub_->publish(cmd);
  }

  void publish_forward()
  {
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = forward_speed_;
    cmd_vel_pub_->publish(cmd);
  }

  void publish_rotate()
  {
    geometry_msgs::msg::Twist cmd;
    const double direction = degrees_ >= 0.0 ? 1.0 : -1.0;
    cmd.angular.z = direction * std::abs(angular_speed_);
    cmd_vel_pub_->publish(cmd);
  }

  void enter_safe_stop(const std::string & reason)
  {
    RCLCPP_ERROR(get_logger(), "SAFE_STOP: %s", reason.c_str());
    set_state(State::SAFE_STOP, reason);
    publish_stop();
  }

  void request_shutdown(const std::string & reason)
  {
    if (shutdown_requested_) {
      return;
    }
    shutdown_requested_ = true;
    RCLCPP_INFO(get_logger(), "%s", reason.c_str());
    rclcpp::shutdown();
  }

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::TimerBase::SharedPtr control_timer_;

  double obstacle_;
  double degrees_;
  double forward_speed_;
  double angular_speed_;
  double rotation_scale_;
  double planned_drive_distance_;
  double planned_drive_time_;
  double planned_rotate_time_;

  std::optional<double> first_front_distance_;
  rclcpp::Time move_start_time_;
  rclcpp::Time stop_start_time_;
  rclcpp::Time rotate_start_time_;
  State state_;
  State last_logged_state_;
  bool shutdown_requested_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PreApproachSimple>());
  rclcpp::shutdown();
  return 0;
}
