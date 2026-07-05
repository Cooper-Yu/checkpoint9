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
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

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
        use_tf_rotation_(false),
        rotation_tolerance_(0.03),
        rotation_reference_frame_("odom"),
        rotation_base_frame_("robot_base_footprint"),
        final_approach_(false),
        rotate_time_(0.0),
        target_yaw_(0.0),
        invalid_scan_count_(0),
        state_(State::WAITING_FOR_SCAN),
        last_logged_state_(State::WAITING_FOR_SCAN),
        service_call_started_(false),
        shutdown_requested_(false),
        tf_buffer_(this->get_clock()),
        tf_listener_(tf_buffer_)
  {
    declare_parameter<double>("obstacle", obstacle_);
    declare_parameter<double>("degrees", degrees_);
    declare_parameter<double>("forward_speed", forward_speed_);
    declare_parameter<double>("angular_speed", angular_speed_);
    declare_parameter<double>("rotation_scale", rotation_scale_);
    declare_parameter<bool>("use_tf_rotation", use_tf_rotation_);
    declare_parameter<double>("rotation_tolerance", rotation_tolerance_);
    declare_parameter<std::string>("rotation_reference_frame", rotation_reference_frame_);
    declare_parameter<std::string>("rotation_base_frame", rotation_base_frame_);
    declare_parameter<bool>("final_approach", final_approach_);

    obstacle_ = get_parameter("obstacle").as_double();
    degrees_ = get_parameter("degrees").as_double();
    forward_speed_ = get_parameter("forward_speed").as_double();
    angular_speed_ = get_parameter("angular_speed").as_double();
    rotation_scale_ = get_parameter("rotation_scale").as_double();
    use_tf_rotation_ = get_parameter("use_tf_rotation").as_bool();
    rotation_tolerance_ = get_parameter("rotation_tolerance").as_double();
    rotation_reference_frame_ = get_parameter("rotation_reference_frame").as_string();
    rotation_base_frame_ = get_parameter("rotation_base_frame").as_string();
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

    if (rotation_tolerance_ <= 0.0) {
      state_ = State::SAFE_STOP;
      RCLCPP_ERROR(get_logger(), "Invalid rotation_tolerance parameter: %.3f", rotation_tolerance_);
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
    approach_client_ = create_client<attach_shelf::srv::GoToLoading>("/approach_shelf");

    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", rclcpp::SensorDataQoS(),
        std::bind(&PreApproachV2::scan_callback, this, std::placeholders::_1));

    control_timer_ = create_wall_timer(100ms, std::bind(&PreApproachV2::timer_callback, this));

    RCLCPP_INFO(get_logger(),
                "pre_approach_v2 started: obstacle=%.2f m, degrees=%.2f, forward_speed=%.2f m/s, "
                "angular_speed=%.2f rad/s, rotation_scale=%.2f, rotate_time=%.2f s, "
                "use_tf_rotation=%s, final_approach=%s",
                obstacle_, degrees_, forward_speed_, angular_speed_, rotation_scale_, rotate_time_,
                use_tf_rotation_ ? "true" : "false", final_approach_ ? "true" : "false");
  }

private:
  // Task 2 uses the same staged pre-approach as Task 1, then hands control to
  // /approach_shelf only after the robot has stopped and rotated.
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
  static constexpr double kScanPauseTimeout = 1.0;

  bool get_front_distance(const sensor_msgs::msg::LaserScan & scan, double window_degrees,
                          double & front_distance)
  {
    std::vector<double> valid_ranges;

    // Use a small window around 0 rad instead of a single ray to reduce noise.
    double half_window = window_degrees / 2 * kPi / 180;

    for (double i = -half_window; i < half_window; i += scan.angle_increment) {
      // Convert the desired angle into the matching ranges[] index.
      int index = static_cast<int>(std::round((i - scan.angle_min) / scan.angle_increment));

      if (index < 0 || index >= static_cast<int>(scan.ranges.size())) {
        continue;
      }

      double distance = scan.ranges[index];

      // LaserScan can contain inf/nan or readings outside the sensor's valid range.
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

    return false;
  }

  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    double distance = 0.0;

    // Keep the last valid front distance so the timer callback can make one
    // consistent control decision per cycle.
    if (get_front_distance(*msg, 10.0, distance)) {
      front_distance_ = distance;
      last_valid_scan_time_ = now();
      invalid_scan_count_ = 0;
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
                           "Valid front scan: distance=%.3f m, obstacle=%.3f m", distance,
                           obstacle_);
    } else {
      ++invalid_scan_count_;
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                           "Invalid front scan window: invalid_count=%d", invalid_scan_count_);
    }
  }

  void timer_callback()
  {
    log_state_if_changed();

    switch (state_) {
      case State::WAITING_FOR_SCAN: {
        publish_stop();

        if (!front_distance_.has_value()) {
          return;
        }

        if (front_distance_.value() > obstacle_) {
          set_state(State::MOVING_FORWARD, "front distance is greater than obstacle threshold");
          return;
        }

        stop_start_time_ = this->now();
        set_state(State::STOP_BEFORE_ROTATE, "already within obstacle threshold");
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
          set_state(State::STOP_BEFORE_ROTATE, "front distance reached obstacle threshold");
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
          set_state(State::DONE, "degrees is zero");
          return;
        }

        rotation_start_time_ = this->now();
        if (use_tf_rotation_ && !start_tf_rotation()) {
          enter_safe_stop("cannot start TF rotation");
          return;
        }
        set_state(State::ROTATING, "settling stop complete");
        return;
      }

      // ROTATING -> DONE after rotate_time_
      case State::ROTATING: {
        if (use_tf_rotation_) {
          rotate_with_tf_feedback();
          return;
        }

        // Continue publishing angular velocity; a single Twist message is not enough.
        double elapsed = (this->now() - rotation_start_time_).seconds();
        if (elapsed < rotate_time_) {
          publish_rotate();
        } else {
          publish_stop();
          set_state(State::DONE, "rotation duration complete");
        }

        return;
      }

      // SAFE_STOP keeps the robot stopped; DONE hands /cmd_vel ownership to the service.
      case State::SAFE_STOP: {
        publish_stop();
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
    if (scan_age > kScanPauseTimeout) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                           "Stopping until a fresh front scan arrives; latest is %.2f seconds old",
                           scan_age);
      publish_stop();
      return false;
    }

    if (invalid_scan_count_ >= 10) {
      enter_safe_stop("too many consecutive invalid scan windows");
      return false;
    }

    return true;
  }

  const char * state_name(State state) const
  {
    switch (state) {
      case State::WAITING_FOR_SCAN:
        return "WAITING_FOR_SCAN";
      case State::MOVING_FORWARD:
        return "MOVING_FORWARD";
      case State::STOP_BEFORE_ROTATE:
        return "STOP_BEFORE_ROTATE";
      case State::ROTATING:
        return "ROTATING";
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

  std::optional<double> current_yaw()
  {
    try {
      const auto transform = tf_buffer_.lookupTransform(rotation_reference_frame_,
                                                        rotation_base_frame_, tf2::TimePointZero);
      return tf2::getYaw(transform.transform.rotation);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "TF yaw lookup failed: %s", ex.what());
      return std::nullopt;
    }
  }

  double normalize_angle(double angle) const
  {
    while (angle > kPi) {
      angle -= 2.0 * kPi;
    }
    while (angle < -kPi) {
      angle += 2.0 * kPi;
    }
    return angle;
  }

  bool start_tf_rotation()
  {
    auto yaw = current_yaw();
    if (!yaw.has_value()) {
      return false;
    }

    const double target_angle_rad = degrees_ * kPi / 180.0;
    target_yaw_ = normalize_angle(yaw.value() + target_angle_rad);
    RCLCPP_INFO(get_logger(),
                "TF rotation started: current_yaw=%.3f rad, target_delta=%.3f rad, "
                "target_yaw=%.3f rad, tolerance=%.3f rad",
                yaw.value(), target_angle_rad, target_yaw_, rotation_tolerance_);
    return true;
  }

  void rotate_with_tf_feedback()
  {
    const double elapsed = (now() - rotation_start_time_).seconds();
    const double rotation_timeout = std::max(3.0, rotate_time_ * 3.0);
    if (elapsed > rotation_timeout) {
      enter_safe_stop("TF rotation timed out");
      return;
    }

    auto yaw = current_yaw();
    if (!yaw.has_value()) {
      publish_stop();
      return;
    }

    const double error = normalize_angle(target_yaw_ - yaw.value());
    if (std::abs(error) <= rotation_tolerance_) {
      publish_stop();
      RCLCPP_INFO(get_logger(),
                  "TF rotation complete: yaw=%.3f rad, target=%.3f rad, error=%.3f rad",
                  yaw.value(), target_yaw_, error);
      set_state(State::DONE, "TF rotation reached target yaw");
      return;
    }

    geometry_msgs::msg::Twist cmd;
    const double commanded_speed =
        std::min(std::abs(angular_speed_), std::max(0.08, std::abs(error)));
    cmd.angular.z = error > 0.0 ? commanded_speed : -commanded_speed;
    cmd_vel_pub_->publish(cmd);

    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
                         "TF rotating: yaw=%.3f rad, target=%.3f rad, error=%.3f rad, "
                         "angular_z=%.3f rad/s",
                         yaw.value(), target_yaw_, error, cmd.angular.z);
  }

  void call_approach_service_once()
  {
    if (!final_approach_) {
      publish_stop();
      request_shutdown("pre_approach_v2 complete; final_approach=false");
      return;
    }

    // DONE is visited by the timer repeatedly. Once the service owns /cmd_vel,
    // this node must not keep publishing stop commands in parallel.
    if (service_call_started_) {
      if ((now() - service_request_time_).seconds() > 45.0) {
        enter_safe_stop("/approach_shelf timed out");
        request_shutdown("pre_approach_v2 stopped safely");
      }
      return;
    }

    service_call_started_ = true;
    service_request_time_ = now();
    publish_stop();

    // The service server is launched only for final_approach=true, so the
    // client waits here instead of assuming the service is already available.
    if (!approach_client_->wait_for_service(3s)) {
      enter_safe_stop("/approach_shelf is not available");
      request_shutdown("pre_approach_v2 stopped safely");
      return;
    }

    auto request = std::make_shared<attach_shelf::srv::GoToLoading::Request>();
    request->attach_to_shelf = true;

    RCLCPP_INFO(get_logger(), "Calling /approach_shelf with attach_to_shelf=true");
    approach_client_->async_send_request(
        request, [this](rclcpp::Client<attach_shelf::srv::GoToLoading>::SharedFuture future) {
          const auto response = future.get();
          RCLCPP_INFO(get_logger(), "/approach_shelf response: complete=%s",
                      response->complete ? "true" : "false");
          if (!response->complete) {
            enter_safe_stop("/approach_shelf returned complete=false");
            request_shutdown("pre_approach_v2 stopped safely");
            return;
          }

          request_shutdown("pre_approach_v2 complete");
        });
  }

  void enter_safe_stop(const std::string & reason)
  {
    safety_stop_reason_ = reason;
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
  rclcpp::Client<attach_shelf::srv::GoToLoading>::SharedPtr approach_client_;
  rclcpp::TimerBase::SharedPtr control_timer_;

  double obstacle_;
  double degrees_;
  double forward_speed_;
  double angular_speed_;
  double rotation_scale_;
  bool use_tf_rotation_;
  double rotation_tolerance_;
  std::string rotation_reference_frame_;
  std::string rotation_base_frame_;
  bool final_approach_;
  double rotate_time_;
  double target_yaw_;

  std::optional<double> front_distance_;
  rclcpp::Time last_valid_scan_time_;
  int invalid_scan_count_;

  State state_;
  State last_logged_state_;
  rclcpp::Time rotation_start_time_;
  rclcpp::Time stop_start_time_;
  rclcpp::Time service_request_time_;
  std::string safety_stop_reason_;
  bool service_call_started_;
  bool shutdown_requested_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PreApproachV2>());
  rclcpp::shutdown();
  return 0;
}
