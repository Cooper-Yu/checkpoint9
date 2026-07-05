#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "attach_shelf/srv/go_to_loading.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2_ros/transform_broadcaster.h"

using namespace std::chrono_literals;

class ApproachServiceServerSimple : public rclcpp::Node
{
public:
  ApproachServiceServerSimple()
      : Node("approach_service_server_simple"),
        intensity_threshold_(8000.0),
        min_cluster_size_(2),
        max_x_difference_(0.75),
        min_leg_separation_(0.25),
        rotate_speed_(0.3),
        forward_speed_(0.2),
        yaw_tolerance_(0.05),
        center_distance_tolerance_(0.20),
        forward_step_distance_(0.20),
        cart_frame_retry_count_(6),
        movement_timeout_(30.0),
        conservative_offset_(0.0),
        final_drive_distance_(0.30),
        service_straight_test_(false)
  {
    declare_parameter<double>("rotate_speed", rotate_speed_);
    declare_parameter<double>("forward_speed", forward_speed_);
    declare_parameter<double>("conservative_offset", conservative_offset_);
    declare_parameter<double>("final_drive_distance", final_drive_distance_);
    declare_parameter<double>("center_distance_tolerance", center_distance_tolerance_);
    declare_parameter<double>("forward_step_distance", forward_step_distance_);
    declare_parameter<double>("movement_timeout", movement_timeout_);
    declare_parameter<bool>("service_straight_test", service_straight_test_);

    rotate_speed_ = get_parameter("rotate_speed").as_double();
    forward_speed_ = get_parameter("forward_speed").as_double();
    conservative_offset_ = get_parameter("conservative_offset").as_double();
    final_drive_distance_ = get_parameter("final_drive_distance").as_double();
    center_distance_tolerance_ = get_parameter("center_distance_tolerance").as_double();
    forward_step_distance_ = get_parameter("forward_step_distance").as_double();
    movement_timeout_ = get_parameter("movement_timeout").as_double();
    service_straight_test_ = get_parameter("service_straight_test").as_bool();

    scan_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    service_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    rclcpp::SubscriptionOptions scan_options;
    scan_options.callback_group = scan_callback_group_;
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", rclcpp::SensorDataQoS(),
        std::bind(&ApproachServiceServerSimple::scan_callback, this, std::placeholders::_1),
        scan_options);

    cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    elevator_up_pub_ = create_publisher<std_msgs::msg::String>("/elevator_up", 10);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    approach_service_ = create_service<attach_shelf::srv::GoToLoading>(
        "/approach_shelf", std::bind(&ApproachServiceServerSimple::handle_approach_request, this,
                                     std::placeholders::_1, std::placeholders::_2),
        rmw_qos_profile_services_default, service_callback_group_);

    RCLCPP_INFO(get_logger(), "approach_service_server_simple ready on /approach_shelf");
  }

private:
  struct CartFrame
  {
    double x;
    double y;
    std::string frame_id;
  };

  struct LegCandidate
  {
    double x;
    double y;
    double angle;
    double range;
    size_t index;
    size_t size;
  };

  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    latest_scan_ = *msg;
  }

  void handle_approach_request(
      const std::shared_ptr<attach_shelf::srv::GoToLoading::Request> request,
      std::shared_ptr<attach_shelf::srv::GoToLoading::Response> response)
  {
    RCLCPP_INFO(get_logger(), "Received /approach_shelf request: attach_to_shelf=%s",
                request->attach_to_shelf ? "true" : "false");

    auto cart_frame = wait_for_cart_frame(3.0);
    if (!cart_frame.has_value()) {
      publish_stop();
      response->complete = false;
      RCLCPP_WARN(get_logger(),
                  "/approach_shelf response complete=false: one-shot cart_frame detection failed");
      return;
    }

    publish_cart_frame(cart_frame.value());
    RCLCPP_INFO(get_logger(), "Published one-shot cart_frame TF in frame '%s'",
                cart_frame->frame_id.c_str());

    if (!request->attach_to_shelf) {
      response->complete = true;
      RCLCPP_INFO(get_logger(), "/approach_shelf response complete=true: detection-only request");
      return;
    }

    if (service_straight_test_) {
      response->complete = perform_straight_test_final_approach(cart_frame.value());
    } else {
      response->complete = perform_stepwise_final_approach(cart_frame.value());
    }
    if (!response->complete) {
      publish_stop();
      RCLCPP_WARN(get_logger(),
                  "/approach_shelf response complete=false: one-shot final approach failed");
      return;
    }

    RCLCPP_INFO(get_logger(),
                "/approach_shelf response complete=true: one-shot final approach finished");
  }

  std::optional<CartFrame> wait_for_cart_frame(double timeout_seconds)
  {
    const auto start_time = now();
    rclcpp::Rate rate(20.0);

    while (rclcpp::ok() && (now() - start_time).seconds() < timeout_seconds) {
      auto cart_frame = detect_cart_frame();
      if (cart_frame.has_value()) {
        return cart_frame;
      }
      rate.sleep();
    }

    return std::nullopt;
  }

  std::optional<CartFrame> detect_cart_frame()
  {
    if (!latest_scan_.has_value()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                           "Cannot detect cart_frame: no /scan received yet");
      return std::nullopt;
    }

    const auto & scan = latest_scan_.value();
    if (scan.ranges.empty() || scan.intensities.empty()) {
      RCLCPP_WARN(get_logger(), "Cannot detect cart_frame: scan ranges or intensities are empty");
      return std::nullopt;
    }

    std::vector<std::vector<size_t>> clusters;
    std::vector<size_t> current_cluster;
    const size_t n = std::min(scan.ranges.size(), scan.intensities.size());
    size_t high_intensity_ray_count = 0;
    float max_intensity = 0.0F;

    for (size_t i = 0; i < n; ++i) {
      const float intensity = scan.intensities[i];
      const float range = scan.ranges[i];
      if (std::isfinite(intensity)) {
        max_intensity = std::max(max_intensity, intensity);
      }

      if (std::isfinite(intensity) && intensity >= intensity_threshold_ && std::isfinite(range) &&
          range >= scan.range_min && range <= scan.range_max) {
        ++high_intensity_ray_count;
        current_cluster.push_back(i);
        continue;
      }

      if (current_cluster.size() >= static_cast<size_t>(min_cluster_size_)) {
        clusters.push_back(current_cluster);
      }
      current_cluster.clear();
    }

    if (current_cluster.size() >= static_cast<size_t>(min_cluster_size_)) {
      clusters.push_back(current_cluster);
    }

    std::vector<LegCandidate> candidates;
    for (const auto & cluster : clusters) {
      const size_t index = cluster[cluster.size() / 2];
      const double angle = scan.angle_min + index * scan.angle_increment;
      const double range = scan.ranges[index];
      const double x = range * std::cos(angle);
      const double y = range * std::sin(angle);

      if (x <= 0.0) {
        RCLCPP_INFO(get_logger(),
                    "Ignoring reflective cluster behind robot: x=%.3f, y=%.3f, rays=%zu", x, y,
                    cluster.size());
        continue;
      }

      candidates.push_back(LegCandidate{x, y, angle, range, index, cluster.size()});
    }

    log_reflective_candidates(candidates, clusters.size(), high_intensity_ray_count, max_intensity);

    if (candidates.size() < 2) {
      RCLCPP_WARN(get_logger(),
                  "Cannot detect cart_frame: found %zu front reflective candidates from %zu clusters, "
                  "need at least 2 (high_intensity_rays=%zu, max_intensity=%.1f, threshold=%.1f)",
                  candidates.size(), clusters.size(), high_intensity_ray_count, max_intensity,
                  intensity_threshold_);
      return std::nullopt;
    }

    std::optional<std::pair<LegCandidate, LegCandidate>> best_pair;
    double best_score = std::numeric_limits<double>::max();
    double largest_rejected_midpoint_y = 0.0;
    for (size_t i = 0; i < candidates.size(); ++i) {
      for (size_t j = i + 1; j < candidates.size(); ++j) {
        const auto & a = candidates[i];
        const auto & b = candidates[j];
        const double leg_separation = std::abs(a.y - b.y);
        const double x_difference = std::abs(a.x - b.x);
        const double midpoint_y = (a.y + b.y) / 2.0;
        const bool accepted =
            leg_separation >= min_leg_separation_ && x_difference <= max_x_difference_;
        largest_rejected_midpoint_y =
            std::max(largest_rejected_midpoint_y, std::abs(midpoint_y));

        RCLCPP_INFO(get_logger(),
                    "Reflective pair candidate %zu-%zu: midpoint_y=%.3f, separation=%.3f, "
                    "x_difference=%.3f, accepted=%s",
                    i, j, midpoint_y, leg_separation, x_difference, accepted ? "true" : "false");

        if (!accepted) {
          continue;
        }

        const double score = std::abs(midpoint_y) + x_difference;
        if (score < best_score) {
          best_score = score;
          best_pair = std::make_pair(a, b);
        }
      }
    }

    if (!best_pair.has_value()) {
      RCLCPP_WARN(get_logger(),
                  "Cannot detect cart_frame: %zu candidates but no valid centered leg pair "
                  "(max_seen_midpoint_y=%.3f)",
                  candidates.size(), largest_rejected_midpoint_y);
      return std::nullopt;
    }

    const auto leg_1 = best_pair->first;
    const auto leg_2 = best_pair->second;
    const double x = (leg_1.x + leg_2.x) / 2.0;
    const double y = (leg_1.y + leg_2.y) / 2.0;
    const double leg_separation = std::abs(leg_1.y - leg_2.y);
    const double x_difference = std::abs(leg_1.x - leg_2.x);

    RCLCPP_INFO(get_logger(),
                "One-shot cart_frame: x=%.3f, y=%.3f, leg1=(%.3f, %.3f), leg2=(%.3f, %.3f), "
                "separation=%.3f, x_difference=%.3f, front_candidates=%zu",
                x, y, leg_1.x, leg_1.y, leg_2.x, leg_2.y, leg_separation, x_difference,
                candidates.size());

    return CartFrame{x, y, scan.header.frame_id};
  }

  void log_reflective_candidates(const std::vector<LegCandidate> & candidates, size_t cluster_count,
                                 size_t high_intensity_ray_count, float max_intensity)
  {
    RCLCPP_INFO(get_logger(),
                "Reflective scan summary: clusters=%zu, front_candidates=%zu, "
                "high_intensity_rays=%zu, max_intensity=%.1f, threshold=%.1f",
                cluster_count, candidates.size(), high_intensity_ray_count, max_intensity,
                intensity_threshold_);

    for (size_t i = 0; i < candidates.size(); ++i) {
      const auto & candidate = candidates[i];
      RCLCPP_INFO(get_logger(),
                  "Reflective candidate %zu: index=%zu, angle=%.3f rad, angle_deg=%.1f, "
                  "range=%.3f m, x=%.3f, y=%.3f, rays=%zu",
                  i, candidate.index, candidate.angle, candidate.angle * 180.0 / kPi,
                  candidate.range, candidate.x, candidate.y, candidate.size);
    }
  }

  bool perform_stepwise_final_approach(const CartFrame & initial_cart_frame)
  {
    CartFrame cart_frame = initial_cart_frame;
    const auto start_time = now();
    int step = 0;

    RCLCPP_INFO(get_logger(),
                "Stepwise final approach started: center_tolerance=%.3f m, "
                "forward_step=%.3f m, movement_timeout=%.3f s, final_push=%.3f m",
                center_distance_tolerance_, forward_step_distance_, movement_timeout_,
                final_drive_distance_);

    while (rclcpp::ok() && (now() - start_time).seconds() < movement_timeout_) {
      const double target_yaw = std::atan2(cart_frame.y, cart_frame.x);
      const double distance_to_center = std::hypot(cart_frame.x, cart_frame.y);
      RCLCPP_INFO(get_logger(),
                  "Stepwise center step %d: cart_frame=(%.3f, %.3f), distance=%.3f m, "
                  "target_yaw=%.3f rad",
                  step, cart_frame.x, cart_frame.y, distance_to_center, target_yaw);

      if (distance_to_center <= center_distance_tolerance_) {
        RCLCPP_INFO(get_logger(), "Reached cart center tolerance: %.3f <= %.3f",
                    distance_to_center, center_distance_tolerance_);
        break;
      }

      if (!rotate_by_yaw_open_loop(target_yaw, "Stepwise yaw correction")) {
        return false;
      }

      const double drive_distance =
          std::min(forward_step_distance_, std::max(distance_to_center - conservative_offset_, 0.0));
      if (!drive_forward_open_loop(drive_distance, "Stepwise drive toward cart center")) {
        return false;
      }

      auto updated_cart_frame = wait_for_cart_frame(1.0);
      if (!updated_cart_frame.has_value()) {
        RCLCPP_WARN(get_logger(),
                    "Stepwise final approach failed: cannot detect cart_frame after step %d", step);
        return false;
      }
      auto recovered_cart_frame =
          recover_cart_frame_after_motion(cart_frame, updated_cart_frame.value(), drive_distance);
      if (!recovered_cart_frame.has_value()) {
        RCLCPP_WARN(get_logger(), "Stepwise final approach failed: cart_frame stayed unstable");
        return false;
      }
      cart_frame = recovered_cart_frame.value();
      ++step;
    }

    if ((now() - start_time).seconds() >= movement_timeout_) {
      RCLCPP_WARN(get_logger(), "Stepwise final approach timed out after %.3f seconds",
                  movement_timeout_);
      return false;
    }

    if (!drive_forward_open_loop(final_drive_distance_, "Final shelf push")) {
      return false;
    }

    std_msgs::msg::String elevator_msg;
    elevator_up_pub_->publish(elevator_msg);
    RCLCPP_INFO(get_logger(), "One-shot final approach complete; published /elevator_up");
    return true;
  }

  bool perform_straight_test_final_approach(const CartFrame & cart_frame)
  {
    const double straight_distance = std::max(cart_frame.x - conservative_offset_, 0.0);
    RCLCPP_WARN(get_logger(),
                "SERVICE STRAIGHT TEST MODE: skipping service yaw correction. "
                "cart_frame=(%.3f, %.3f), straight_distance=%.3f m, final_push=%.3f m",
                cart_frame.x, cart_frame.y, straight_distance, final_drive_distance_);

    if (!drive_forward_open_loop(straight_distance, "Straight test drive to detected cart x")) {
      return false;
    }

    log_cart_frame_diagnostic("after straight test drive");

    if (!drive_forward_open_loop(final_drive_distance_, "Straight test final shelf push")) {
      return false;
    }

    std_msgs::msg::String elevator_msg;
    elevator_up_pub_->publish(elevator_msg);
    RCLCPP_INFO(get_logger(), "Straight test final approach complete; published /elevator_up");
    return true;
  }

  std::optional<CartFrame> recover_cart_frame_after_motion(const CartFrame & previous,
                                                           const CartFrame & first_detection,
                                                           double drive_distance)
  {
    if (cart_frame_progress_is_plausible(previous, first_detection, drive_distance)) {
      return first_detection;
    }

    publish_stop();
    RCLCPP_WARN(get_logger(),
                "Suspicious cart_frame update after motion; stopping and re-detecting before "
                "continuing. previous=(%.3f, %.3f), first_detection=(%.3f, %.3f)",
                previous.x, previous.y, first_detection.x, first_detection.y);

    for (int retry = 1; retry <= cart_frame_retry_count_; ++retry) {
      rclcpp::sleep_for(300ms);
      auto retry_cart_frame = wait_for_cart_frame(0.5);
      if (!retry_cart_frame.has_value()) {
        RCLCPP_WARN(get_logger(), "cart_frame recovery retry %d/%d: detection failed", retry,
                    cart_frame_retry_count_);
        continue;
      }

      if (cart_frame_progress_is_plausible(previous, retry_cart_frame.value(), drive_distance)) {
        RCLCPP_INFO(get_logger(),
                    "cart_frame recovery retry %d/%d accepted: recovered=(%.3f, %.3f)",
                    retry, cart_frame_retry_count_, retry_cart_frame->x, retry_cart_frame->y);
        return retry_cart_frame;
      }

      RCLCPP_WARN(get_logger(),
                  "cart_frame recovery retry %d/%d still suspicious: recovered=(%.3f, %.3f)",
                  retry, cart_frame_retry_count_, retry_cart_frame->x, retry_cart_frame->y);
    }

    return std::nullopt;
  }

  bool cart_frame_progress_is_plausible(const CartFrame & previous, const CartFrame & current,
                                        double drive_distance)
  {
    const double previous_distance = std::hypot(previous.x, previous.y);
    const double current_distance = std::hypot(current.x, current.y);
    const double previous_lateral_error = std::abs(previous.y);
    const double current_lateral_error = std::abs(current.y);
    const bool moved_closer = current_distance < previous_distance;
    const bool lateral_error_improved = current_lateral_error <= previous_lateral_error;
    const bool accepted = moved_closer && lateral_error_improved;

    RCLCPP_INFO(get_logger(),
                "cart_frame progress check: previous_distance=%.3f, current_distance=%.3f, "
                "previous_abs_y=%.3f, current_abs_y=%.3f, drive_distance=%.3f, "
                "moved_closer=%s, lateral_error_improved=%s, accepted=%s",
                previous_distance, current_distance, previous_lateral_error, current_lateral_error,
                drive_distance, moved_closer ? "true" : "false",
                lateral_error_improved ? "true" : "false", accepted ? "true" : "false");

    return accepted;
  }

  void log_cart_frame_diagnostic(const std::string & label)
  {
    rclcpp::sleep_for(300ms);
    auto cart_frame = detect_cart_frame();
    if (!cart_frame.has_value()) {
      RCLCPP_WARN(get_logger(), "Cart-frame diagnostic %s: detection failed", label.c_str());
      return;
    }

    const double remaining_distance = std::hypot(cart_frame->x, cart_frame->y);
    const double remaining_yaw = std::atan2(cart_frame->y, cart_frame->x);
    RCLCPP_INFO(get_logger(),
                "Cart-frame diagnostic %s: remaining cart_frame=(%.3f, %.3f), "
                "remaining_distance=%.3f m, remaining_yaw=%.3f rad",
                label.c_str(), cart_frame->x, cart_frame->y, remaining_distance, remaining_yaw);
  }

  bool rotate_by_yaw_open_loop(double target_yaw, const std::string & label)
  {
    if (std::abs(target_yaw) < yaw_tolerance_) {
      RCLCPP_INFO(get_logger(), "%s skipped: target_yaw %.3f is within tolerance", label.c_str(),
                  target_yaw);
      publish_stop();
      return true;
    }

    if (rotate_speed_ <= 0.0) {
      RCLCPP_WARN(get_logger(), "%s failed: rotate_speed %.3f is not positive", label.c_str(),
                  rotate_speed_);
      return false;
    }

    const double rotate_time = std::abs(target_yaw) / rotate_speed_;
    if (rotate_time > movement_timeout_) {
      RCLCPP_WARN(get_logger(), "%s failed: rotate_time %.3f exceeds movement_timeout %.3f",
                  label.c_str(), rotate_time, movement_timeout_);
      return false;
    }

    geometry_msgs::msg::Twist cmd;
    cmd.angular.z = target_yaw > 0.0 ? rotate_speed_ : -rotate_speed_;
    const auto start_time = now();
    rclcpp::Rate rate(20.0);

    RCLCPP_INFO(get_logger(), "%s: target_yaw=%.3f rad, angular_z=%.3f rad/s, duration=%.3f s",
                label.c_str(), target_yaw, cmd.angular.z, rotate_time);

    while (rclcpp::ok() && (now() - start_time).seconds() < rotate_time) {
      cmd_vel_pub_->publish(cmd);
      rate.sleep();
    }

    publish_stop();
    rclcpp::sleep_for(200ms);
    return true;
  }

  bool drive_forward_open_loop(double distance, const std::string & label)
  {
    if (distance <= 0.0) {
      RCLCPP_INFO(get_logger(), "%s skipped: distance %.3f is not positive", label.c_str(),
                  distance);
      publish_stop();
      return true;
    }

    if (forward_speed_ <= 0.0) {
      RCLCPP_WARN(get_logger(), "%s failed: forward_speed %.3f is not positive", label.c_str(),
                  forward_speed_);
      return false;
    }

    const double drive_time = distance / forward_speed_;
    if (drive_time > movement_timeout_) {
      RCLCPP_WARN(get_logger(), "%s failed: drive_time %.3f exceeds movement_timeout %.3f",
                  label.c_str(), drive_time, movement_timeout_);
      return false;
    }

    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = forward_speed_;
    const auto start_time = now();
    rclcpp::Rate rate(20.0);

    RCLCPP_INFO(get_logger(), "%s: distance=%.3f m, speed=%.3f m/s, duration=%.3f s",
                label.c_str(), distance, forward_speed_, drive_time);

    while (rclcpp::ok() && (now() - start_time).seconds() < drive_time) {
      cmd_vel_pub_->publish(cmd);
      rate.sleep();
    }

    publish_stop();
    rclcpp::sleep_for(200ms);
    return true;
  }

  void publish_cart_frame(const CartFrame & cart_frame)
  {
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = now();
    transform.header.frame_id = cart_frame.frame_id;
    transform.child_frame_id = "cart_frame";
    transform.transform.translation.x = cart_frame.x;
    transform.transform.translation.y = cart_frame.y;
    transform.transform.translation.z = 0.0;
    transform.transform.rotation.w = 1.0;
    tf_broadcaster_->sendTransform(transform);
  }

  void publish_stop()
  {
    geometry_msgs::msg::Twist cmd;
    cmd_vel_pub_->publish(cmd);
  }

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr elevator_up_pub_;
  rclcpp::Service<attach_shelf::srv::GoToLoading>::SharedPtr approach_service_;
  rclcpp::CallbackGroup::SharedPtr scan_callback_group_;
  rclcpp::CallbackGroup::SharedPtr service_callback_group_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  std::optional<sensor_msgs::msg::LaserScan> latest_scan_;

  double intensity_threshold_;
  int min_cluster_size_;
  double max_x_difference_;
  double min_leg_separation_;
  double rotate_speed_;
  double forward_speed_;
  double yaw_tolerance_;
  double center_distance_tolerance_;
  double forward_step_distance_;
  int cart_frame_retry_count_;
  double movement_timeout_;
  double conservative_offset_;
  double final_drive_distance_;
  bool service_straight_test_;

  static constexpr double kPi = 3.14159265358979323846;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ApproachServiceServerSimple>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
