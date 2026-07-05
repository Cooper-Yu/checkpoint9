#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "attach_shelf/srv/go_to_loading.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

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
        min_rotate_speed_(0.05),
        rotate_speed_gain_(1.0),
        forward_speed_(0.2),
        yaw_tolerance_(0.005),
        center_lateral_tolerance_(0.05),
        center_distance_tolerance_(0.20),
        center_lock_distance_(0.35),
        center_lock_min_steps_(2),
        center_drive_scale_(1.5),
        center_extra_forward_distance_(0.0),
        yaw_correction_steps_(3),
        lateral_yaw_gain_(0.4),
        min_yaw_correction_distance_(0.55),
        restore_yaw_after_correction_(false),
        forward_step_distance_(0.20),
        cart_frame_retry_count_(6),
        movement_timeout_(45.0),
        conservative_offset_(0.0),
        final_drive_distance_(0.30),
        enable_final_push_(true),
        service_straight_test_(false),
        straight_sample_count_(5),
        straight_sample_max_spread_(0.15),
        target_base_frame_("robot_base_link")
  {
    declare_parameter<double>("rotate_speed", rotate_speed_);
    declare_parameter<double>("min_rotate_speed", min_rotate_speed_);
    declare_parameter<double>("rotate_speed_gain", rotate_speed_gain_);
    declare_parameter<double>("forward_speed", forward_speed_);
    declare_parameter<double>("yaw_tolerance", yaw_tolerance_);
    declare_parameter<double>("conservative_offset", conservative_offset_);
    declare_parameter<double>("final_drive_distance", final_drive_distance_);
    declare_parameter<double>("center_lateral_tolerance", center_lateral_tolerance_);
    declare_parameter<double>("center_distance_tolerance", center_distance_tolerance_);
    declare_parameter<double>("center_lock_distance", center_lock_distance_);
    declare_parameter<int>("center_lock_min_steps", center_lock_min_steps_);
    declare_parameter<double>("center_drive_scale", center_drive_scale_);
    declare_parameter<double>("center_extra_forward_distance", center_extra_forward_distance_);
    declare_parameter<int>("yaw_correction_steps", yaw_correction_steps_);
    declare_parameter<double>("lateral_yaw_gain", lateral_yaw_gain_);
    declare_parameter<double>("min_yaw_correction_distance", min_yaw_correction_distance_);
    declare_parameter<bool>("restore_yaw_after_correction", restore_yaw_after_correction_);
    declare_parameter<double>("forward_step_distance", forward_step_distance_);
    declare_parameter<double>("movement_timeout", movement_timeout_);
    declare_parameter<bool>("enable_final_push", enable_final_push_);
    declare_parameter<bool>("service_straight_test", service_straight_test_);
    declare_parameter<int>("straight_sample_count", straight_sample_count_);
    declare_parameter<double>("straight_sample_max_spread", straight_sample_max_spread_);
    declare_parameter<std::string>("target_base_frame", target_base_frame_);

    rotate_speed_ = get_parameter("rotate_speed").as_double();
    min_rotate_speed_ = get_parameter("min_rotate_speed").as_double();
    rotate_speed_gain_ = get_parameter("rotate_speed_gain").as_double();
    forward_speed_ = get_parameter("forward_speed").as_double();
    yaw_tolerance_ = get_parameter("yaw_tolerance").as_double();
    conservative_offset_ = get_parameter("conservative_offset").as_double();
    final_drive_distance_ = get_parameter("final_drive_distance").as_double();
    center_lateral_tolerance_ = get_parameter("center_lateral_tolerance").as_double();
    center_distance_tolerance_ = get_parameter("center_distance_tolerance").as_double();
    center_lock_distance_ = get_parameter("center_lock_distance").as_double();
    center_lock_min_steps_ = get_parameter("center_lock_min_steps").as_int();
    center_drive_scale_ = get_parameter("center_drive_scale").as_double();
    center_extra_forward_distance_ = get_parameter("center_extra_forward_distance").as_double();
    yaw_correction_steps_ = get_parameter("yaw_correction_steps").as_int();
    lateral_yaw_gain_ = get_parameter("lateral_yaw_gain").as_double();
    min_yaw_correction_distance_ = get_parameter("min_yaw_correction_distance").as_double();
    restore_yaw_after_correction_ = get_parameter("restore_yaw_after_correction").as_bool();
    forward_step_distance_ = get_parameter("forward_step_distance").as_double();
    movement_timeout_ = get_parameter("movement_timeout").as_double();
    enable_final_push_ = get_parameter("enable_final_push").as_bool();
    service_straight_test_ = get_parameter("service_straight_test").as_bool();
    straight_sample_count_ = get_parameter("straight_sample_count").as_int();
    straight_sample_max_spread_ = get_parameter("straight_sample_max_spread").as_double();
    target_base_frame_ = get_parameter("target_base_frame").as_string();

    scan_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    service_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    if (straight_sample_count_ <= 0) {
      RCLCPP_WARN(get_logger(), "straight_sample_count=%d is invalid; using 1",
                  straight_sample_count_);
      straight_sample_count_ = 1;
    }

    rclcpp::SubscriptionOptions scan_options;
    scan_options.callback_group = scan_callback_group_;
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", rclcpp::SensorDataQoS(),
        std::bind(&ApproachServiceServerSimple::scan_callback, this, std::placeholders::_1),
        scan_options);

    cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    elevator_up_pub_ = create_publisher<std_msgs::msg::String>("/elevator_up", 10);
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    approach_service_ = create_service<attach_shelf::srv::GoToLoading>(
        "/approach_shelf",
        std::bind(&ApproachServiceServerSimple::handle_approach_request, this,
                  std::placeholders::_1, std::placeholders::_2),
        rmw_qos_profile_services_default, service_callback_group_);

    RCLCPP_INFO(get_logger(),
                "approach_service_server_simple ready on /approach_shelf; target_base_frame=%s",
                target_base_frame_.c_str());
  }

private:
  struct CartFrame
  {
    double x;
    double y;
    std::string frame_id;
    double laser_x;
    double laser_y;
    std::string laser_frame_id;
  };

  struct LegCandidate
  {
    double x;
    double y;
    double angle;
    double range;
    size_t index;
    size_t low_index;
    size_t high_index;
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
    RCLCPP_INFO(get_logger(),
                "Published one-shot cart_frame TF in frame '%s'; original laser[%s]=(%.3f, %.3f)",
                cart_frame->frame_id.c_str(), cart_frame->laser_frame_id.c_str(),
                cart_frame->laser_x, cart_frame->laser_y);

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
      auto candidate =
          make_leg_candidate(scan, index, cluster.front(), cluster.back(), cluster.size());

      if (candidate.x <= 0.0) {
        RCLCPP_INFO(get_logger(),
                    "Ignoring reflective cluster behind robot: x=%.3f, y=%.3f, rays=%zu",
                    candidate.x, candidate.y, cluster.size());
        continue;
      }

      candidates.push_back(candidate);
    }

    log_reflective_candidates(candidates, clusters.size(), high_intensity_ray_count, max_intensity);

    if (candidates.size() < 2) {
      RCLCPP_WARN(
          get_logger(),
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
        largest_rejected_midpoint_y = std::max(largest_rejected_midpoint_y, std::abs(midpoint_y));

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

    auto leg_1 = best_pair->first;
    auto leg_2 = best_pair->second;
    choose_inner_edge_pair(scan, leg_1, leg_2);
    const double laser_x = (leg_1.x + leg_2.x) / 2.0;
    const double laser_y = (leg_1.y + leg_2.y) / 2.0;
    const double leg_separation = std::abs(leg_1.y - leg_2.y);
    const double x_difference = std::abs(leg_1.x - leg_2.x);

    RCLCPP_INFO(get_logger(),
                "One-shot cart_frame in laser frame '%s': x=%.3f, y=%.3f, "
                "leg1=(%.3f, %.3f), leg2=(%.3f, %.3f), "
                "separation=%.3f, x_difference=%.3f, front_candidates=%zu",
                scan.header.frame_id.c_str(), laser_x, laser_y, leg_1.x, leg_1.y, leg_2.x, leg_2.y,
                leg_separation, x_difference, candidates.size());

    return make_cart_frame_in_target_base(laser_x, laser_y, scan.header.frame_id);
  }

  std::optional<CartFrame> make_cart_frame_in_target_base(double laser_x, double laser_y,
                                                          const std::string & laser_frame_id)
  {
    if (target_base_frame_.empty() || target_base_frame_ == laser_frame_id) {
      RCLCPP_WARN(get_logger(),
                  "Using laser-frame cart target directly because target_base_frame='%s' and "
                  "laser_frame='%s'",
                  target_base_frame_.c_str(), laser_frame_id.c_str());
      return CartFrame{laser_x, laser_y, laser_frame_id, laser_x, laser_y, laser_frame_id};
    }

    geometry_msgs::msg::PointStamped laser_point;
    laser_point.header.stamp = rclcpp::Time(0);
    laser_point.header.frame_id = laser_frame_id;
    laser_point.point.x = laser_x;
    laser_point.point.y = laser_y;
    laser_point.point.z = 0.0;

    try {
      const auto transform = tf_buffer_->lookupTransform(target_base_frame_, laser_frame_id,
                                                         tf2::TimePointZero, 200ms);
      geometry_msgs::msg::PointStamped base_point;
      tf2::doTransform(laser_point, base_point, transform);

      RCLCPP_INFO(get_logger(),
                  "Transformed cart_frame target: laser[%s]=(%.3f, %.3f) -> base[%s]=(%.3f, %.3f); "
                  "laser_origin_in_base=(%.3f, %.3f)",
                  laser_frame_id.c_str(), laser_x, laser_y, target_base_frame_.c_str(),
                  base_point.point.x, base_point.point.y, transform.transform.translation.x,
                  transform.transform.translation.y);

      return CartFrame{base_point.point.x, base_point.point.y, target_base_frame_, laser_x, laser_y,
                       laser_frame_id};
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN(
          get_logger(),
          "Cannot transform cart target from laser frame '%s' to target_base_frame '%s': %s",
          laser_frame_id.c_str(), target_base_frame_.c_str(), ex.what());
      return std::nullopt;
    }
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

  LegCandidate make_leg_candidate(const sensor_msgs::msg::LaserScan & scan, size_t index,
                                  size_t low_index, size_t high_index, size_t size)
  {
    const double angle = scan.angle_min + index * scan.angle_increment;
    const double range = scan.ranges[index];
    const double x = range * std::cos(angle);
    const double y = range * std::sin(angle);
    return LegCandidate{x, y, angle, range, index, low_index, high_index, size};
  }

  void choose_inner_edge_pair(const sensor_msgs::msg::LaserScan & scan, LegCandidate & leg_1,
                              LegCandidate & leg_2)
  {
    // The shelf opening is bounded by the two inner reflective edges, not the cluster centers.
    const bool leg_1_is_right = leg_1.y < leg_2.y;
    const size_t leg_1_inner_index = leg_1_is_right ? leg_1.high_index : leg_1.low_index;
    const size_t leg_2_inner_index = leg_1_is_right ? leg_2.low_index : leg_2.high_index;
    const auto leg_1_center = leg_1;
    const auto leg_2_center = leg_2;

    leg_1 =
        make_leg_candidate(scan, leg_1_inner_index, leg_1.low_index, leg_1.high_index, leg_1.size);
    leg_2 =
        make_leg_candidate(scan, leg_2_inner_index, leg_2.low_index, leg_2.high_index, leg_2.size);

    RCLCPP_INFO(get_logger(),
                "Selected inner shelf-leg edges: leg1 center_index=%zu inner_index=%zu "
                "center=(%.3f, %.3f) inner=(%.3f, %.3f), leg2 center_index=%zu "
                "inner_index=%zu center=(%.3f, %.3f) inner=(%.3f, %.3f)",
                leg_1_center.index, leg_1.index, leg_1_center.x, leg_1_center.y, leg_1.x, leg_1.y,
                leg_2_center.index, leg_2.index, leg_2_center.x, leg_2_center.y, leg_2.x, leg_2.y);
  }

  bool perform_stepwise_final_approach(const CartFrame & initial_cart_frame)
  {
    const auto start_time = now();
    auto averaged_cart_frame = sample_average_cart_frame(initial_cart_frame);
    int step = 0;
    bool center_approach_complete = false;

    RCLCPP_INFO(get_logger(),
                "Stepwise final approach started: center_tolerance=%.3f m, "
                "lateral_tolerance=%.3f m, lock_distance=%.3f m, lock_min_steps=%d, "
                "center_drive_scale=%.3f, center_extra_forward=%.3f m, "
                "yaw_correction_steps=%d, lateral_yaw_gain=%.3f, forward_step=%.3f m, "
                "movement_timeout=%.3f s, final_push=%.3f m, enable_final_push=%s",
                center_distance_tolerance_, center_lateral_tolerance_, center_lock_distance_,
                center_lock_min_steps_, center_drive_scale_, center_extra_forward_distance_,
                yaw_correction_steps_, lateral_yaw_gain_, forward_step_distance_, movement_timeout_,
                final_drive_distance_, enable_final_push_ ? "true" : "false");

    while (rclcpp::ok() && (now() - start_time).seconds() < movement_timeout_) {
      if (!averaged_cart_frame.has_value()) {
        RCLCPP_WARN(get_logger(),
                    "Stepwise final approach failed: cart_frame samples were not stable");
        return false;
      }

      publish_cart_frame(averaged_cart_frame.value());
      const bool yaw_correction_enabled =
          step < yaw_correction_steps_ && averaged_cart_frame->x > min_yaw_correction_distance_;
      const double raw_target_yaw = std::atan2(averaged_cart_frame->y, averaged_cart_frame->x);
      const double target_yaw = yaw_correction_enabled ? raw_target_yaw * lateral_yaw_gain_ : 0.0;
      const double distance_to_center = std::hypot(averaged_cart_frame->x, averaged_cart_frame->y);
      const double lateral_error = std::abs(averaged_cart_frame->y);
      const bool yaw_needs_rotation = std::abs(target_yaw) >= yaw_tolerance_;
      const bool smooth_forward =
          step > 0 && !yaw_needs_rotation && lateral_error <= center_lateral_tolerance_;
      RCLCPP_INFO(get_logger(),
                  "Stepwise center step %d: cart_frame=(%.3f, %.3f), distance=%.3f m, "
                  "lateral_error=%.3f m, raw_target_yaw=%.3f rad, lateral_yaw_gain=%.3f, "
                  "applied_target_yaw=%.3f rad, yaw_correction_enabled=%s, "
                  "min_yaw_correction_distance=%.3f m, restore_yaw_after_correction=%s, "
                  "smooth_forward=%s",
                  step, averaged_cart_frame->x, averaged_cart_frame->y, distance_to_center,
                  lateral_error, raw_target_yaw, lateral_yaw_gain_, target_yaw,
                  yaw_correction_enabled ? "true" : "false", min_yaw_correction_distance_,
                  restore_yaw_after_correction_ ? "true" : "false",
                  smooth_forward ? "true" : "false");

      if (averaged_cart_frame->x <= center_distance_tolerance_ &&
          lateral_error <= center_lateral_tolerance_) {
        RCLCPP_INFO(get_logger(),
                    "Reached cart center tolerance: x=%.3f <= %.3f and abs(y)=%.3f <= %.3f",
                    averaged_cart_frame->x, center_distance_tolerance_, lateral_error,
                    center_lateral_tolerance_);
        center_approach_complete = true;
        break;
      }

      if (step >= center_lock_min_steps_ && averaged_cart_frame->x <= center_lock_distance_) {
        const double raw_locked_drive_distance =
            std::max(averaged_cart_frame->x - conservative_offset_, 0.0);
        const double locked_drive_distance = raw_locked_drive_distance * center_drive_scale_;
        RCLCPP_WARN(get_logger(),
                    "Locking final center approach: step=%d, cart_frame=(%.3f, %.3f), "
                    "raw_locked_drive_distance=%.3f m, center_drive_scale=%.3f, "
                    "locked_drive_distance=%.3f m. Further cart_frame re-detection is skipped "
                    "because close-range reflective clusters can jump.",
                    step, averaged_cart_frame->x, averaged_cart_frame->y, raw_locked_drive_distance,
                    center_drive_scale_, locked_drive_distance);
        if (!drive_forward_open_loop(locked_drive_distance, "Locked drive to cart center")) {
          return false;
        }
        RCLCPP_INFO(get_logger(),
                    "Locked center approach complete; robot stopped at detected center");
        center_approach_complete = true;
        break;
      }

      if (yaw_needs_rotation) {
        if (!rotate_by_yaw_open_loop(target_yaw, "Stepwise yaw correction")) {
          return false;
        }
      } else {
        RCLCPP_INFO(get_logger(),
                    "Stepwise yaw correction skipped without stop: target_yaw %.3f is within "
                    "tolerance %.3f",
                    target_yaw, yaw_tolerance_);
      }

      const double drive_distance = std::min(
          forward_step_distance_, std::max(averaged_cart_frame->x - conservative_offset_, 0.0));
      if (!drive_forward_open_loop(drive_distance, "Stepwise drive toward cart center",
                                   !smooth_forward)) {
        return false;
      }

      if (restore_yaw_after_correction_) {
        if (!rotate_by_yaw_open_loop(-target_yaw, "Stepwise yaw recovery")) {
          return false;
        }
      } else if (yaw_correction_enabled) {
        RCLCPP_INFO(
            get_logger(),
            "Stepwise yaw recovery skipped: keeping corrected heading for next center sample");
      }

      log_cart_frame_diagnostic("after stepwise center drive");
      averaged_cart_frame = sample_average_cart_frame_after_motion(!smooth_forward);
      ++step;
    }

    if (!center_approach_complete && (now() - start_time).seconds() >= movement_timeout_) {
      RCLCPP_WARN(get_logger(), "Stepwise final approach timed out after %.3f seconds",
                  movement_timeout_);
      return false;
    }

    if (!center_approach_complete) {
      RCLCPP_WARN(get_logger(), "Stepwise final approach stopped before reaching cart center");
      return false;
    }

    log_final_center_verification("after center approach");

    if (center_extra_forward_distance_ > 0.0) {
      RCLCPP_WARN(get_logger(),
                  "Center calibration extra forward drive: distance=%.3f m. This is separate from "
                  "the final shelf push and does not publish /elevator_up.",
                  center_extra_forward_distance_);
      if (!drive_forward_open_loop(center_extra_forward_distance_,
                                   "Center calibration extra forward drive")) {
        return false;
      }
      log_final_center_verification("after center calibration extra forward");
    }

    if (enable_final_push_) {
      RCLCPP_WARN(get_logger(),
                  "Starting final shelf push: distance=%.3f m. Elevator will be raised after this "
                  "drive completes.",
                  final_drive_distance_);
      if (!drive_forward_open_loop(final_drive_distance_, "Final shelf push")) {
        return false;
      }
    } else {
      RCLCPP_WARN(
          get_logger(),
          "Final shelf push skipped because enable_final_push=false; stopping at cart center");
    }

    if (enable_final_push_) {
      publish_elevator_up();
      RCLCPP_INFO(get_logger(), "One-shot final approach complete; published /elevator_up");
    } else {
      RCLCPP_INFO(get_logger(), "Center-only final approach complete; /elevator_up not published");
    }
    return true;
  }

  void log_final_center_verification(const std::string & label)
  {
    publish_stop();
    rclcpp::sleep_for(300ms);

    auto verification_cart_frame = wait_for_cart_frame(1.0);
    if (!verification_cart_frame.has_value()) {
      RCLCPP_WARN(
          get_logger(),
          "Final center verification only (%s): cart_frame detection failed after stopping; "
          "no extra motion command was sent",
          label.c_str());
      return;
    }

    const double remaining_distance =
        std::hypot(verification_cart_frame->x, verification_cart_frame->y);
    const double remaining_yaw = std::atan2(verification_cart_frame->y, verification_cart_frame->x);
    RCLCPP_WARN(get_logger(),
                "Final center verification only (%s): detected cart_frame[%s]=(%.3f, %.3f), "
                "laser[%s]=(%.3f, %.3f), remaining_distance=%.3f m, lateral_error=%.3f m, "
                "remaining_yaw=%.3f rad. No extra motion command was sent.",
                label.c_str(), verification_cart_frame->frame_id.c_str(),
                verification_cart_frame->x, verification_cart_frame->y,
                verification_cart_frame->laser_frame_id.c_str(), verification_cart_frame->laser_x,
                verification_cart_frame->laser_y, remaining_distance,
                std::abs(verification_cart_frame->y), remaining_yaw);
  }

  bool perform_straight_test_final_approach(const CartFrame & cart_frame)
  {
    const auto start_time = now();
    auto averaged_cart_frame = sample_average_cart_frame(cart_frame);

    while (rclcpp::ok() && (now() - start_time).seconds() < movement_timeout_) {
      if (!averaged_cart_frame.has_value()) {
        RCLCPP_WARN(get_logger(), "Straight test failed: cart_frame samples were not stable");
        return false;
      }

      const double remaining_distance = std::hypot(averaged_cart_frame->x, averaged_cart_frame->y);
      const double straight_distance = std::min(
          forward_step_distance_, std::max(averaged_cart_frame->x - conservative_offset_, 0.0));
      RCLCPP_WARN(get_logger(),
                  "SERVICE STRAIGHT TEST MODE: skipping service yaw correction. "
                  "averaged_cart_frame=(%.3f, %.3f), remaining_distance=%.3f m, "
                  "straight_distance=%.3f m, center_tolerance=%.3f m",
                  averaged_cart_frame->x, averaged_cart_frame->y, remaining_distance,
                  straight_distance, center_distance_tolerance_);

      if (remaining_distance <= center_distance_tolerance_ ||
          averaged_cart_frame->x <= center_distance_tolerance_) {
        RCLCPP_INFO(get_logger(),
                    "Straight service reached detected cart center: remaining_distance=%.3f m, "
                    "remaining_x=%.3f m",
                    remaining_distance, averaged_cart_frame->x);
        const double center_trim_distance =
            std::max(averaged_cart_frame->x - conservative_offset_, 0.0);
        if (center_trim_distance > 0.02) {
          RCLCPP_INFO(
              get_logger(),
              "Straight service trimming remaining center distance before final push: %.3f m",
              center_trim_distance);
          if (!drive_forward_open_loop(center_trim_distance, "Straight test final center trim")) {
            return false;
          }
        }
        break;
      }

      if (!drive_forward_open_loop(straight_distance, "Straight test drive to detected cart x")) {
        return false;
      }

      log_cart_frame_diagnostic("after straight center drive");
      averaged_cart_frame = sample_average_cart_frame_after_motion();
    }

    if ((now() - start_time).seconds() >= movement_timeout_) {
      RCLCPP_WARN(get_logger(), "Straight test final approach timed out after %.3f seconds",
                  movement_timeout_);
      return false;
    }

    if (!drive_forward_open_loop(final_drive_distance_, "Straight test final shelf push")) {
      return false;
    }

    publish_elevator_up();
    RCLCPP_INFO(get_logger(), "Straight test final approach complete; published /elevator_up");
    return true;
  }

  std::optional<CartFrame> sample_average_cart_frame_after_motion(bool stop_before_sampling = true)
  {
    auto cart_frame = wait_for_cart_frame(1.0);
    if (!cart_frame.has_value()) {
      RCLCPP_WARN(get_logger(), "Straight service re-detection failed after center drive");
      return std::nullopt;
    }

    return sample_average_cart_frame(cart_frame.value(), stop_before_sampling);
  }

  std::optional<CartFrame> sample_average_cart_frame(const CartFrame & first_cart_frame,
                                                    bool stop_before_sampling = true)
  {
    std::vector<CartFrame> samples;
    samples.push_back(first_cart_frame);
    if (stop_before_sampling) {
      publish_stop();
    }

    RCLCPP_INFO(get_logger(),
                "Straight service sampling started: target_samples=%d, max_spread=%.3f m, "
                "stop_before_sampling=%s",
                straight_sample_count_, straight_sample_max_spread_,
                stop_before_sampling ? "true" : "false");

    for (int i = 1; i < straight_sample_count_; ++i) {
      rclcpp::sleep_for(200ms);
      auto cart_frame = wait_for_cart_frame(0.5);
      if (!cart_frame.has_value()) {
        RCLCPP_WARN(get_logger(), "Straight service sample %d/%d failed: no cart_frame", i + 1,
                    straight_sample_count_);
        return std::nullopt;
      }

      samples.push_back(cart_frame.value());
      RCLCPP_INFO(get_logger(),
                  "Straight service sample %d/%d: cart_frame[%s]=(%.3f, %.3f), "
                  "laser[%s]=(%.3f, %.3f)",
                  i + 1, straight_sample_count_, cart_frame->frame_id.c_str(), cart_frame->x,
                  cart_frame->y, cart_frame->laser_frame_id.c_str(), cart_frame->laser_x,
                  cart_frame->laser_y);
    }

    double min_x = samples.front().x;
    double max_x = samples.front().x;
    double min_y = samples.front().y;
    double max_y = samples.front().y;
    double sum_x = 0.0;
    double sum_y = 0.0;
    double sum_laser_x = 0.0;
    double sum_laser_y = 0.0;

    for (const auto & sample : samples) {
      min_x = std::min(min_x, sample.x);
      max_x = std::max(max_x, sample.x);
      min_y = std::min(min_y, sample.y);
      max_y = std::max(max_y, sample.y);
      sum_x += sample.x;
      sum_y += sample.y;
      sum_laser_x += sample.laser_x;
      sum_laser_y += sample.laser_y;
    }

    const double x_spread = max_x - min_x;
    const double y_spread = max_y - min_y;
    const double average_x = sum_x / static_cast<double>(samples.size());
    const double average_y = sum_y / static_cast<double>(samples.size());
    const double average_laser_x = sum_laser_x / static_cast<double>(samples.size());
    const double average_laser_y = sum_laser_y / static_cast<double>(samples.size());

    RCLCPP_INFO(get_logger(),
                "Straight service sample summary: count=%zu, average[%s]=(%.3f, %.3f), "
                "average_laser[%s]=(%.3f, %.3f), x_spread=%.3f, y_spread=%.3f",
                samples.size(), first_cart_frame.frame_id.c_str(), average_x, average_y,
                first_cart_frame.laser_frame_id.c_str(), average_laser_x, average_laser_y, x_spread,
                y_spread);

    if (x_spread > straight_sample_max_spread_ || y_spread > straight_sample_max_spread_) {
      RCLCPP_WARN(get_logger(),
                  "Straight service samples rejected: spread too large "
                  "(x_spread=%.3f, y_spread=%.3f, max=%.3f)",
                  x_spread, y_spread, straight_sample_max_spread_);
      return std::nullopt;
    }

    return CartFrame{average_x,       average_y,       first_cart_frame.frame_id,
                     average_laser_x, average_laser_y, first_cart_frame.laser_frame_id};
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
                    "cart_frame recovery retry %d/%d accepted: recovered=(%.3f, %.3f)", retry,
                    cart_frame_retry_count_, retry_cart_frame->x, retry_cart_frame->y);
        return retry_cart_frame;
      }

      RCLCPP_WARN(get_logger(),
                  "cart_frame recovery retry %d/%d still suspicious: recovered=(%.3f, %.3f)", retry,
                  cart_frame_retry_count_, retry_cart_frame->x, retry_cart_frame->y);
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
                "Cart-frame diagnostic %s: remaining cart_frame[%s]=(%.3f, %.3f), "
                "laser[%s]=(%.3f, %.3f), remaining_distance=%.3f m, remaining_yaw=%.3f rad",
                label.c_str(), cart_frame->frame_id.c_str(), cart_frame->x, cart_frame->y,
                cart_frame->laser_frame_id.c_str(), cart_frame->laser_x, cart_frame->laser_y,
                remaining_distance, remaining_yaw);
  }

  bool rotate_by_yaw_open_loop(double target_yaw, const std::string & label)
  {
    const double abs_target_yaw = std::abs(target_yaw);
    if (abs_target_yaw < yaw_tolerance_) {
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

    if (min_rotate_speed_ <= 0.0) {
      RCLCPP_WARN(get_logger(), "%s failed: min_rotate_speed %.3f is not positive", label.c_str(),
                  min_rotate_speed_);
      return false;
    }

    if (rotate_speed_gain_ <= 0.0) {
      RCLCPP_WARN(get_logger(), "%s failed: rotate_speed_gain %.3f is not positive", label.c_str(),
                  rotate_speed_gain_);
      return false;
    }

    const double adaptive_rotate_speed =
        std::clamp(abs_target_yaw * rotate_speed_gain_, min_rotate_speed_, rotate_speed_);
    const double rotate_time = abs_target_yaw / adaptive_rotate_speed;
    if (rotate_time > movement_timeout_) {
      RCLCPP_WARN(get_logger(), "%s failed: rotate_time %.3f exceeds movement_timeout %.3f",
                  label.c_str(), rotate_time, movement_timeout_);
      return false;
    }

    geometry_msgs::msg::Twist cmd;
    cmd.angular.z = target_yaw > 0.0 ? adaptive_rotate_speed : -adaptive_rotate_speed;
    const auto start_time = now();
    rclcpp::Rate rate(20.0);

    RCLCPP_INFO(get_logger(),
                "%s: target_yaw=%.4f rad, angular_z=%.4f rad/s, duration=%.3f s "
                "(adaptive speed, min=%.3f, max=%.3f, gain=%.3f)",
                label.c_str(), target_yaw, cmd.angular.z, rotate_time, min_rotate_speed_,
                rotate_speed_, rotate_speed_gain_);

    while (rclcpp::ok() && (now() - start_time).seconds() < rotate_time) {
      cmd_vel_pub_->publish(cmd);
      rate.sleep();
    }

    publish_stop();
    rclcpp::sleep_for(200ms);
    return true;
  }

  bool drive_forward_open_loop(double distance, const std::string & label, bool stop_after = true)
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

    RCLCPP_INFO(get_logger(),
                "%s: distance=%.3f m, speed=%.3f m/s, duration=%.3f s, stop_after=%s",
                label.c_str(), distance, forward_speed_, drive_time,
                stop_after ? "true" : "false");

    while (rclcpp::ok() && (now() - start_time).seconds() < drive_time) {
      cmd_vel_pub_->publish(cmd);
      rate.sleep();
    }

    if (stop_after) {
      publish_stop();
      rclcpp::sleep_for(200ms);
    }
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

  void publish_elevator_up()
  {
    std_msgs::msg::String elevator_msg;
    elevator_msg.data = "up";
    rclcpp::Rate rate(10.0);
    for (int i = 0; rclcpp::ok() && i < 5; ++i) {
      elevator_up_pub_->publish(elevator_msg);
      rate.sleep();
    }
  }

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr elevator_up_pub_;
  rclcpp::Service<attach_shelf::srv::GoToLoading>::SharedPtr approach_service_;
  rclcpp::CallbackGroup::SharedPtr scan_callback_group_;
  rclcpp::CallbackGroup::SharedPtr service_callback_group_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  std::optional<sensor_msgs::msg::LaserScan> latest_scan_;

  double intensity_threshold_;
  int min_cluster_size_;
  double max_x_difference_;
  double min_leg_separation_;
  double rotate_speed_;
  double min_rotate_speed_;
  double rotate_speed_gain_;
  double forward_speed_;
  double yaw_tolerance_;
  double center_lateral_tolerance_;
  double center_distance_tolerance_;
  double center_lock_distance_;
  int center_lock_min_steps_;
  double center_drive_scale_;
  double center_extra_forward_distance_;
  int yaw_correction_steps_;
  double lateral_yaw_gain_;
  double min_yaw_correction_distance_;
  bool restore_yaw_after_correction_;
  double forward_step_distance_;
  int cart_frame_retry_count_;
  double movement_timeout_;
  double conservative_offset_;
  double final_drive_distance_;
  bool enable_final_push_;
  bool service_straight_test_;
  int straight_sample_count_;
  double straight_sample_max_spread_;
  std::string target_base_frame_;

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
