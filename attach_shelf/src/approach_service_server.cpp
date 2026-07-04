#include <algorithm>
#include <cmath>
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

class ApproachServiceServer : public rclcpp::Node
{
public:
  ApproachServiceServer()
      : Node("approach_service_server"),
        intensity_threshold_(8000.0),
        min_cluster_size_(2),
        max_x_difference_(0.35),
        min_leg_separation_(0.25),
        rotate_speed_(0.3),
        forward_speed_(0.2),
        yaw_tolerance_(0.05),
        movement_timeout_(10.0),
        conservative_offset_(0.15),
        max_target_yaw_(0.8),
        target_x_before_push_(0.35),
        forward_step_distance_(0.15),
        final_drive_distance_(0.30)
  {
    scan_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    service_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    rclcpp::SubscriptionOptions scan_options;
    scan_options.callback_group = scan_callback_group_;
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", rclcpp::SensorDataQoS(),
        std::bind(&ApproachServiceServer::scan_callback, this, std::placeholders::_1), scan_options);

    cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    elevator_up_pub_ = create_publisher<std_msgs::msg::String>("/elevator_up", 10);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    approach_service_ = create_service<attach_shelf::srv::GoToLoading>(
        "/approach_shelf", std::bind(&ApproachServiceServer::handle_approach_request, this,
                                     std::placeholders::_1, std::placeholders::_2),
        rclcpp::ServicesQoS(), service_callback_group_);

    RCLCPP_INFO(get_logger(), "approach_service_server ready on /approach_shelf");
  }

private:
  struct CartFrame
  {
    double x;
    double y;
    std::string frame_id;
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

    auto cart_frame = detect_cart_frame();
    if (!cart_frame.has_value()) {
      publish_stop();
      response->complete = false;
      RCLCPP_WARN(get_logger(),
                  "/approach_shelf response complete=false: cart_frame detection failed");
      return;
    }

    publish_cart_frame(cart_frame.value());
    RCLCPP_INFO(get_logger(), "Published cart_frame TF in frame '%s'",
                cart_frame->frame_id.c_str());

    if (!request->attach_to_shelf) {
      response->complete = true;
      RCLCPP_INFO(get_logger(), "/approach_shelf response complete=true: detection-only request");
      return;
    }

    response->complete = perform_final_approach(cart_frame.value());
    if (!response->complete) {
      publish_stop();
      RCLCPP_WARN(get_logger(), "/approach_shelf response complete=false: final approach failed");
      return;
    }

    RCLCPP_INFO(get_logger(), "/approach_shelf response complete=true: final approach finished");
  }

  std::optional<CartFrame> detect_cart_frame()
  {
    if (!latest_scan_.has_value()) {
      RCLCPP_WARN(get_logger(), "Cannot detect cart_frame: no /scan received yet");
      return std::nullopt;
    }

    const auto & scan = latest_scan_.value();
    if (scan.ranges.empty() || scan.intensities.empty()) {
      RCLCPP_WARN(get_logger(), "Cannot detect cart_frame: scan ranges or intensities are empty");
      return std::nullopt;
    }
    // Adjacent high-intensity rays are treated as one reflective shelf leg candidate.
    std::vector<std::vector<size_t>> clusters;
    std::vector<size_t> current_cluster;
    const size_t n = std::min(scan.ranges.size(), scan.intensities.size());
    size_t high_intensity_ray_count = 0;
    float max_intensity = 0.0F;

    for (size_t i = 0; i < n; i++) {
      const float intensity = scan.intensities[i];
      const float range = scan.ranges[i];

      if (!std::isfinite(intensity)) {
        continue;
      }
      max_intensity = std::max(max_intensity, intensity);

      if (!std::isfinite(range) || range < scan.range_min || range > scan.range_max) {
        if (current_cluster.size() >= static_cast<size_t>(min_cluster_size_)) {
          clusters.push_back(current_cluster);
        }
        current_cluster.clear();
        continue;
      }

      if (scan.intensities[i] >= intensity_threshold_) {
        ++high_intensity_ray_count;
        current_cluster.push_back(static_cast<size_t>(i));
      } else {
        if (current_cluster.size() >= static_cast<size_t>(min_cluster_size_)) {
          clusters.push_back(current_cluster);
        }
        current_cluster.clear();
      }
    }
    // Handle a high-intensity cluster that reaches the end of the scan.
    if (current_cluster.size() >= static_cast<size_t>(min_cluster_size_)) {
      clusters.push_back(current_cluster);
    }

    if (clusters.size() < 2) {
      RCLCPP_WARN(get_logger(),
                  "Cannot detect cart_frame: found %zu reflective clusters, need at least 2 "
                  "(high_intensity_rays=%zu, max_intensity=%.1f, threshold=%.1f)",
                  clusters.size(), high_intensity_ray_count, max_intensity, intensity_threshold_);
      return std::nullopt;
    }

    std::stable_sort(clusters.begin(), clusters.end(),
                     [](const auto & a, const auto & b) { return a.size() > b.size(); });

    const auto & cluster_1 = clusters[0];
    const auto & cluster_2 = clusters[1];

    // Use the middle ray of each cluster as a stable representative leg point.
    const size_t index_1 = cluster_1[cluster_1.size() / 2];
    const size_t index_2 = cluster_2[cluster_2.size() / 2];
    double angle_1 = scan.angle_min + index_1 * scan.angle_increment;
    double range_1 = scan.ranges[index_1];
    const double x_1 = range_1 * std::cos(angle_1);
    const double y_1 = range_1 * std::sin(angle_1);
    double angle_2 = scan.angle_min + index_2 * scan.angle_increment;
    double range_2 = scan.ranges[index_2];
    const double x_2 = range_2 * std::cos(angle_2);
    const double y_2 = range_2 * std::sin(angle_2);

    // Reject detections that do not look like two shelf legs in front of the robot.
    if (x_1 <= 0.0 || x_2 <= 0.0) {
      RCLCPP_WARN(get_logger(),
                  "Invalid shelf leg geometry: leg points must be in front of the robot, got "
                  "x1=%.3f, x2=%.3f",
                  x_1, x_2);
      return std::nullopt;
    }

    double leg_separation = std::abs(y_1 - y_2);
    if (leg_separation < min_leg_separation_) {
      RCLCPP_WARN(
          get_logger(),
          "Invalid shelf leg geometry: lateral separation %.3f is smaller than minimum %.3f",
          leg_separation, min_leg_separation_);
      return std::nullopt;
    }

    double x_difference = std::abs(x_1 - x_2);
    if (x_difference > max_x_difference_) {
      RCLCPP_WARN(get_logger(),
                  "Invalid shelf leg geometry: x difference %.3f is larger than maximum %.3f",
                  x_difference, max_x_difference_);
      return std::nullopt;
    }

    const double x = (x_1 + x_2) / 2;
    const double y = (y_1 + y_2) / 2;

    if (x <= 0.0) {
      RCLCPP_WARN(get_logger(), "Invalid cart_frame: midpoint x %.3f must be positive", x);
      return std::nullopt;
    }

    RCLCPP_INFO(get_logger(),
                "Detected cart_frame: x=%.3f, y=%.3f, leg1=(%.3f, %.3f), leg2=(%.3f, %.3f), "
                "separation=%.3f",
                x, y, x_1, y_1, x_2, y_2, leg_separation);
    return CartFrame{x, y, scan.header.frame_id};
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
    transform.transform.rotation.x = 0.0;
    transform.transform.rotation.y = 0.0;
    transform.transform.rotation.z = 0.0;
    transform.transform.rotation.w = 1.0;

    tf_broadcaster_->sendTransform(transform);
  }

  bool rotate_by_yaw_open_loop(double target_yaw)
  {
    if (std::abs(target_yaw) < yaw_tolerance_) {
      RCLCPP_INFO(get_logger(), "Target yaw is within tolerance, skipping rotation");
      return true;
    }

    const double rotate_time = std::abs(target_yaw) / rotate_speed_;
    if (rotate_time > movement_timeout_) {
      RCLCPP_WARN(get_logger(), "Rotate time %.3f exceeds movement timeout %.3f", rotate_time,
                  movement_timeout_);
      publish_stop();
      return false;
    }

    geometry_msgs::msg::Twist cmd;
    cmd.angular.z = target_yaw > 0.0 ? rotate_speed_ : -rotate_speed_;
    const auto start_time = now();
    rclcpp::Rate rate(20.0);
    RCLCPP_INFO(get_logger(),
                "Open-loop rotate correction: target_yaw=%.3f rad, angular_z=%.3f rad/s, "
                "duration=%.3f s",
                target_yaw, cmd.angular.z, rotate_time);

    while (rclcpp::ok() && (now() - start_time).seconds() < rotate_time) {
      cmd_vel_pub_->publish(cmd);
      rate.sleep();
    }

    publish_stop();
    return true;
  }

  bool drive_forward_open_loop(double distance, const std::string & label)
  {
    if (distance <= 0.0) {
      RCLCPP_WARN(get_logger(), "%s distance %.3f is not positive", label.c_str(), distance);
      publish_stop();
      return false;
    }

    const double drive_time = distance / forward_speed_;
    if (drive_time > movement_timeout_) {
      RCLCPP_WARN(get_logger(), "%s drive time %.3f exceeds movement timeout %.3f", label.c_str(),
                  drive_time, movement_timeout_);
      publish_stop();
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
    return true;
  }

  bool perform_final_approach(const CartFrame & initial_cart_frame)
  {
    RCLCPP_INFO(get_logger(),
                "Starting scan-closed-loop final approach from cart_frame=(%.3f, %.3f)",
                initial_cart_frame.x, initial_cart_frame.y);

    CartFrame cart_frame = initial_cart_frame;
    const auto start_time = now();
    int step_count = 0;
    while (rclcpp::ok() && (now() - start_time).seconds() < movement_timeout_) {
      publish_cart_frame(cart_frame);
      const double target_yaw = std::atan2(cart_frame.y, cart_frame.x);
      RCLCPP_INFO(get_logger(),
                  "Final approach step %d: cart_frame=(%.3f, %.3f), target_yaw=%.3f",
                  step_count, cart_frame.x, cart_frame.y, target_yaw);

      if (std::abs(target_yaw) > max_target_yaw_) {
        RCLCPP_WARN(get_logger(), "Invalid target_yaw: target yaw %.3f is larger than maximum %.3f",
                    target_yaw, max_target_yaw_);
        publish_stop();
        return false;
      }

      if (!rotate_by_yaw_open_loop(target_yaw)) {
        return false;
      }

      if (cart_frame.x <= target_x_before_push_) {
        RCLCPP_INFO(get_logger(),
                    "cart_frame x %.3f is within final approach target %.3f; starting final push",
                    cart_frame.x, target_x_before_push_);
        break;
      }

      const double step_distance =
          std::min(forward_step_distance_, cart_frame.x - target_x_before_push_);
      if (!drive_forward_open_loop(step_distance, "Scan-closed-loop forward step")) {
        return false;
      }

      rclcpp::sleep_for(std::chrono::milliseconds(200));
      auto updated_cart_frame = detect_cart_frame();
      if (!updated_cart_frame.has_value()) {
        RCLCPP_WARN(get_logger(),
                    "Cannot refresh cart_frame after forward step; stopping final approach");
        publish_stop();
        return false;
      }

      cart_frame = updated_cart_frame.value();
      ++step_count;
    }

    if ((now() - start_time).seconds() >= movement_timeout_) {
      publish_stop();
      RCLCPP_WARN(get_logger(), "Final approach timed out before reaching target_x %.3f",
                  target_x_before_push_);
      return false;
    }

    if (!drive_forward_open_loop(final_drive_distance_, "Final shelf push")) {
      return false;
    }

    std_msgs::msg::String elevator_msg;
    elevator_up_pub_->publish(elevator_msg);

    RCLCPP_INFO(get_logger(), "Final approach complete; published /elevator_up");
    return true;
  }

  void publish_stop()
  {
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = 0.0;
    cmd.angular.z = 0.0;
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
  double movement_timeout_;
  double conservative_offset_;
  double max_target_yaw_;
  double target_x_before_push_;
  double forward_step_distance_;
  double final_drive_distance_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ApproachServiceServer>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
