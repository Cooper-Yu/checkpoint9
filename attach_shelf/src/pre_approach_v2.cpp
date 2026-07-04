#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "attach_shelf/srv/go_to_loading.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

using namespace std::chrono_literals;

class PreApproachV2 : public rclcpp::Node
{
public:
  PreApproachV2()
      : Node("pre_approach_v2"),
        obstacle_(0.4),
        degrees_(-90.0),
        forward_speed_(0.4),
        angular_speed_(0.5),
        rotation_scale_(0.5),
        final_approach_(false),
        rotate_time_(0.0),
        invalid_scan_count_(0),
        state_(State::WAITING_FOR_SCAN),
        service_call_started_(false),
        shutdown_requested_(false)
  {
    declare_parameter<double>("obstacle", obstacle_);
    declare_parameter<double>("degrees", degrees_);
    declare_parameter<double>("forward_speed", forward_speed_);
    declare_parameter<double>("angular_speed", angular_speed_);
    declare_parameter<double>("rotation_scale", rotation_scale_);
    // Task 2 maps this launch parameter directly to GoToLoading.attach_to_shelf.
    declare_parameter<bool>("final_approach", final_approach_);

    obstacle_ = get_parameter("obstacle").as_double();
    degrees_ = get_parameter("degrees").as_double();
    forward_speed_ = get_parameter("forward_speed").as_double();
    angular_speed_ = get_parameter("angular_speed").as_double();
    rotation_scale_ = get_parameter("rotation_scale").as_double();
    final_approach_ = get_parameter("final_approach").as_bool();

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

    // Keep the same calibrated open-loop rotation used by Task 1 before calling the service.
    const double target_angle_rad = degrees_ * kPi / 180.0;
    if (std::abs(target_angle_rad) < 1e-6) {
      rotate_time_ = 0.0;
    } else {
      rotate_time_ = std::abs(target_angle_rad) / std::abs(angular_speed_) * rotation_scale_;
    }

    cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    approach_client_ = create_client<attach_shelf::srv::GoToLoading>("/approach_shelf");

    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", rclcpp::SensorDataQoS(),
        std::bind(&PreApproachV2::scan_callback, this, std::placeholders::_1));

    control_timer_ = create_wall_timer(100ms, std::bind(&PreApproachV2::timer_callback, this));

    RCLCPP_INFO(get_logger(),
                "pre_approach_v2 started: obstacle=%.2f m, degrees=%.2f, final_approach=%s",
                obstacle_, degrees_, final_approach_ ? "true" : "false");
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
  static constexpr double kScanSafeStopTimeout = 3.0;
  static constexpr int kInvalidScanLimit = 30;

  bool get_front_distance(const sensor_msgs::msg::LaserScan & scan, double window_degrees,
                          double & front_distance)
  {
    std::vector<double> valid_ranges;
    bool saw_clear_ray = false;
    const double half_window = window_degrees / 2.0 * kPi / 180.0;

    for (double angle = -half_window; angle < half_window; angle += scan.angle_increment) {
      const int index =
          static_cast<int>(std::round((angle - scan.angle_min) / scan.angle_increment));

      if (index < 0 || index >= static_cast<int>(scan.ranges.size())) {
        continue;
      }

      const double distance = scan.ranges[index];
      if (std::isinf(distance) && distance > 0.0) {
        saw_clear_ray = true;
        continue;
      }

      if (!std::isfinite(distance) || distance < scan.range_min || distance > scan.range_max) {
        continue;
      }

      valid_ranges.push_back(distance);
    }

    if (valid_ranges.empty()) {
      if (saw_clear_ray) {
        front_distance = scan.range_max;
        return true;
      }

      return false;
    }

    front_distance = *std::min_element(valid_ranges.begin(), valid_ranges.end());
    return true;
  }

  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    double distance = 0.0;
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
    // V2 keeps the Task 1 pre-approach state machine; the only new transition is
    // DONE -> /approach_shelf service call.
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

        stop_start_time_ = now();
        state_ = State::STOP_BEFORE_ROTATE;
        return;
      }

      case State::MOVING_FORWARD: {
        if (!check_runtime_safety()) {
          return;
        }

        publish_forward();
        if (front_distance_.value() <= obstacle_) {
          publish_stop();
          stop_start_time_ = now();
          state_ = State::STOP_BEFORE_ROTATE;
        }
        return;
      }

      case State::STOP_BEFORE_ROTATE: {
        if (!check_runtime_safety()) {
          return;
        }

        publish_stop();
        const double elapsed_stop = (now() - stop_start_time_).seconds();
        if (elapsed_stop < 0.2) {
          return;
        }

        if (std::abs(degrees_) < 1e-6) {
          state_ = State::DONE;
          return;
        }

        rotation_start_time_ = now();
        state_ = State::ROTATING;
        return;
      }

      case State::ROTATING: {
        // During open-loop rotation, the front scan window may point away from the wall.
        // Do not require a fresh front-distance reading here; bound the motion by time.
        const double elapsed = (now() - rotation_start_time_).seconds();
        if (elapsed < rotate_time_) {
          publish_rotate();
        } else {
          publish_stop();
          state_ = State::DONE;
        }
        return;
      }

      case State::SAFE_STOP: {
        publish_stop();
        request_shutdown("pre_approach_v2 stopped safely");
        return;
      }

      case State::DONE: {
        call_approach_service_once();
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

  void call_approach_service_once()
  {
    publish_stop();

    // DONE is visited by the timer repeatedly, so guard against sending duplicate requests.
    if (service_call_started_) {
      if ((now() - service_request_time_).seconds() > 20.0) {
        enter_safe_stop("/approach_shelf timed out");
      }
      return;
    }
    service_call_started_ = true;
    service_request_time_ = now();

    if (!approach_client_->wait_for_service(3s)) {
      enter_safe_stop("/approach_shelf is not available");
      return;
    }

    auto request = std::make_shared<attach_shelf::srv::GoToLoading::Request>();
    request->attach_to_shelf = final_approach_;

    RCLCPP_INFO(get_logger(), "Calling /approach_shelf with attach_to_shelf=%s",
                request->attach_to_shelf ? "true" : "false");

    // Use an async response callback so the single-threaded executor can still process the reply.
    approach_client_->async_send_request(
        request, [this](rclcpp::Client<attach_shelf::srv::GoToLoading>::SharedFuture future) {
          const auto response = future.get();
          if (!response->complete) {
            enter_safe_stop("/approach_shelf returned complete=false");
            return;
          }

          request_shutdown("pre_approach_v2 complete");
        });
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
  rclcpp::Client<attach_shelf::srv::GoToLoading>::SharedPtr approach_client_;
  rclcpp::TimerBase::SharedPtr control_timer_;

  double obstacle_;
  double degrees_;
  double forward_speed_;
  double angular_speed_;
  double rotation_scale_;
  bool final_approach_;
  double rotate_time_;

  std::optional<double> front_distance_;
  rclcpp::Time last_valid_scan_time_;
  int invalid_scan_count_;

  State state_;
  rclcpp::Time rotation_start_time_;
  rclcpp::Time stop_start_time_;
  rclcpp::Time service_request_time_;
  std::string safety_stop_reason_;
  bool service_call_started_;
  bool shutdown_requested_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PreApproachV2>());
  rclcpp::shutdown();
  return 0;
}
