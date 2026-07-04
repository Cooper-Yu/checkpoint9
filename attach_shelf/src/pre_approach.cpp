#include <chrono>
#include <cmath>
#include <memory>
#include <optional>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

using namespace std::chrono_literals;

class PreApproach : public rclcpp::Node
{
public:
  PreApproach()
      : Node("pre_approach"),
        obstacle_(0.4),
        degrees_(-90.0),
        forward_speed_(0.4),
        angular_speed_(0.5),
        rotation_scale_(0.5),
        rotate_time_(0.0),
        invalid_scan_count_(0),
        state_(State::WAITING_FOR_SCAN),
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

    if (obstacle_ <= 0.0) {
      state_ = State::SAFE_STOP;
      RCLCPP_ERROR(get_logger(), "Invalid obstacle parameter: %.3f", obstacle_);
    }

    if (forward_speed_ <= 0.0) {
      state_ = State::SAFE_STOP;
      RCLCPP_ERROR(get_logger(), "Invalid forward_speed parameter: %.3f", forward_speed_);
    }

    if (std::abs(degrees_) > 1e-6 && std::abs(angular_speed_) < 1e-6) {
      state_ = State::SAFE_STOP;
      RCLCPP_ERROR(get_logger(), "Invalid angular_speed parameter: %.3f while degrees is %.3f",
                   angular_speed_, degrees_);
    }

    if (rotation_scale_ <= 0.0) {
      state_ = State::SAFE_STOP;
      RCLCPP_ERROR(get_logger(), "Invalid rotation_scale parameter: %.3f", rotation_scale_);
    }

    // The rotation is open-loop: publish angular velocity for a calibrated duration.
    // rotation_scale compensates for the simulator's actual yaw response.
    const double target_angle_rad = degrees_ * kPi / 180.0;
    if (std::abs(target_angle_rad) < 1e-6) {
      rotate_time_ = 0.0;
    } else {
      rotate_time_ = std::abs(target_angle_rad) / std::abs(angular_speed_) * rotation_scale_;
    }

    cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", rclcpp::SensorDataQoS(),
        std::bind(&PreApproach::scan_callback, this, std::placeholders::_1));

    control_timer_ = create_wall_timer(100ms, std::bind(&PreApproach::timer_callback, this));

    RCLCPP_INFO(get_logger(),
                "pre_approach started: obstacle=%.2f m, degrees=%.2f, forward_speed=%.2f m/s, "
                "angular_speed=%.2f rad/s, rotation_scale=%.2f",
                obstacle_, degrees_, forward_speed_, angular_speed_, rotation_scale_);
  }

private:
  enum class State
  {
    WAITING_FOR_SCAN,
    MOVING_FORWARD,
    STOP_BEFORE_ROTATE,
    ROTATING,
    DONE,
    SAFE_STOP
  };

  static constexpr double kPi = 3.14159265358979323846;
  static constexpr double kFrontWindowDegrees = 20.0;
  static constexpr double kScanPauseTimeout = 1.0;
  static constexpr double kScanSafeStopTimeout = 10.0;
  static constexpr int kInvalidScanLimit = 30;

  bool get_front_distance(const sensor_msgs::msg::LaserScan & scan, double window_degrees,
                          double & front_distance)
  {
    std::vector<double> valid_ranges;
    bool saw_clear_ray = false;

    // Use a small window around 0 rad instead of a single ray to reduce noise.
    double half_window = window_degrees / 2 * kPi / 180;

    for (double i = -half_window; i < half_window; i += scan.angle_increment) {
      // Convert the desired angle into the matching ranges[] index.
      int index = static_cast<int>(std::round((i - scan.angle_min) / scan.angle_increment));

      if (index < 0 || index >= static_cast<int>(scan.ranges.size())) {
        continue;
      }

      double distance = scan.ranges[index];

      if (std::isinf(distance) && distance > 0.0) {
        saw_clear_ray = true;
        continue;
      }

      // LaserScan can contain nan or readings outside the sensor's valid range.
      if (!std::isfinite(distance) || distance < scan.range_min || distance > scan.range_max) {
        continue;
      }

      valid_ranges.push_back(distance);
    }

    // The closest valid ray in the front window is the conservative obstacle distance.
    if (!valid_ranges.empty()) {
      front_distance = *std::min_element(valid_ranges.begin(), valid_ranges.end());
      return true;
    }

    if (saw_clear_ray) {
      front_distance = scan.range_max;
      return true;
    }

    return false;
  }

  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    double distance = 0.0;

    // Keep the last valid front distance so the timer callback can make one
    // consistent control decision per cycle.
    if (get_front_distance(*msg, kFrontWindowDegrees, distance)) {
      front_distance_ = distance;
      last_valid_scan_time_ = now();
      invalid_scan_count_ = 0;
    } else {
      ++invalid_scan_count_;
    }
  }

  void timer_callback()
  {
    switch (state_) {
      case State::WAITING_FOR_SCAN: {
        publish_stop();

        if (!front_distance_.has_value()) {
          return;
        }

        if (front_distance_.value() > obstacle_) {
          state_ = State::MOVING_FORWARD;
          return;
        }

        stop_start_time_ = this->now();
        state_ = State::STOP_BEFORE_ROTATE;
        return;
      }

      // MOVING_FORWARD -> STOP_BEFORE_ROTATE when front_distance <= obstacle
      case State::MOVING_FORWARD: {
        if (!check_runtime_safety()) {
          return;
        }

        // Move forward until the front obstacle reaches the requested distance.
        publish_forward();

        if (front_distance_.value() <= obstacle_) {
          publish_stop();
          stop_start_time_ = this->now();
          state_ = State::STOP_BEFORE_ROTATE;
        }

        return;
      }

      // STOP_BEFORE_ROTATE -> ROTATING or DONE
      case State::STOP_BEFORE_ROTATE: {
        if (!check_runtime_safety()) {
          return;
        }

        // Publish zero velocity for a short settling window before rotating.
        publish_stop();
        double elapsed_stop = (this->now() - stop_start_time_).seconds();

        if (elapsed_stop < 0.2) {
          return;
        }

        if (std::abs(degrees_) < 1e-6) {
          state_ = State::DONE;
          return;
        }

        rotation_start_time_ = this->now();
        state_ = State::ROTATING;
        return;
      }

      // ROTATING -> DONE after rotate_time_
      case State::ROTATING: {
        // During open-loop rotation, the front scan window may point away from the wall.
        // Do not require a fresh front-distance reading here; bound the motion by time.
        double elapsed = (this->now() - rotation_start_time_).seconds();
        if (elapsed < rotate_time_) {
          publish_rotate();
        } else {
          publish_stop();
          state_ = State::DONE;
        }

        return;
      }

      // SAFE_STOP and DONE should publish_stop().
      case State::SAFE_STOP: {
        publish_stop();
        return;
      }

      case State::DONE: {
        publish_stop();
        // End the launch cleanly after the final stop command has been sent.
        request_shutdown("pre_approach complete");
        return;
      }
    }
  }

  bool check_runtime_safety()
  {
    if (!front_distance_.has_value()) {
      enter_safe_stop("front distance is unavailable");
      return false;
    }

    const double scan_age = (now() - last_valid_scan_time_).seconds();
    if (scan_age > kScanSafeStopTimeout) {
      enter_safe_stop("latest valid scan exceeded the safe stop timeout");
      return false;
    }

    if (scan_age > kScanPauseTimeout) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "Waiting for a fresh front scan before continuing forward; latest is %.2f seconds old",
          scan_age);
      publish_stop();
      return false;
    }

    if (invalid_scan_count_ >= kInvalidScanLimit) {
      enter_safe_stop("too many consecutive invalid scan windows");
      return false;
    }

    return true;
  }

  void publish_stop()
  {
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = 0.0;
    cmd.angular.z = 0.0;
    cmd_vel_pub_->publish(cmd);
  }

  void publish_forward()
  {
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = forward_speed_;
    cmd.angular.z = 0.0;
    cmd_vel_pub_->publish(cmd);
  }

  void publish_rotate()
  {
    geometry_msgs::msg::Twist cmd;
    const double direction = degrees_ >= 0.0 ? 1.0 : -1.0;
    cmd.linear.x = 0.0;
    cmd.angular.z = direction * std::abs(angular_speed_);
    cmd_vel_pub_->publish(cmd);
  }

  void enter_safe_stop(const std::string & reason)
  {
    safety_stop_reason_ = reason;
    RCLCPP_ERROR(get_logger(), "SAFE_STOP: %s", reason.c_str());
    state_ = State::SAFE_STOP;
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
  double rotate_time_;

  std::optional<double> front_distance_;
  rclcpp::Time last_valid_scan_time_;
  int invalid_scan_count_;

  State state_;
  rclcpp::Time rotation_start_time_;
  rclcpp::Time stop_start_time_;
  std::string safety_stop_reason_;
  bool shutdown_requested_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PreApproach>());
  rclcpp::shutdown();
  return 0;
}
