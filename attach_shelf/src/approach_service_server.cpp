#include <cmath>
#include <memory>
#include <optional>
#include <string>

#include "attach_shelf/srv/go_to_loading.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
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
    max_target_yaw_(0.8)
  {
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan",
      rclcpp::SensorDataQoS(),
      std::bind(&ApproachServiceServer::scan_callback, this, std::placeholders::_1));

    cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    elevator_up_pub_ = create_publisher<std_msgs::msg::String>("/elevator_up", 10);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    approach_service_ = create_service<attach_shelf::srv::GoToLoading>(
      "/approach_shelf",
      std::bind(
        &ApproachServiceServer::handle_approach_request,
        this,
        std::placeholders::_1,
        std::placeholders::_2));

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
    RCLCPP_INFO(
      get_logger(),
      "Received /approach_shelf request: attach_to_shelf=%s",
      request->attach_to_shelf ? "true" : "false");

    auto cart_frame = detect_cart_frame();
    if (!cart_frame.has_value()) {
      publish_stop();
      response->complete = false;
      return;
    }

    publish_cart_frame(cart_frame.value());

    if (!request->attach_to_shelf) {
      response->complete = true;
      return;
    }

    response->complete = perform_final_approach(cart_frame.value());
    if (!response->complete) {
      publish_stop();
    }
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
    // TODO: group adjacent high-intensity rays into clusters.
    std::vector<std::vector<size_t>> clusters;
    std::vector<size_t> current_cluster;
    const size_t n = std::min(scan.ranges.size(), scan.intensities.size());

    for (size_t i = 0; i < n; i++) {
      const float intensity = scan.intensities[i];
      const float range = scan.ranges[i];

      if (!std::isfinite(intensity)) {
        continue;
      }

      if (!std::isfinite(range) ||
          range < scan.range_min ||
          range > scan.range_max) {
        if (current_cluster.size() >= static_cast<size_t>(min_cluster_size_)) {
          clusters.push_back(current_cluster);
        }
        current_cluster.clear();
        continue;
      }

      if (scan.intensities[i] > intensity_threshold_) {
        current_cluster.push_back(static_cast<size_t>(i));
      }
      else {
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

    // TODO: choose the two largest valid clusters.
    if (clusters.size() < 2) {
      return std::nullopt;
    }

    std::stable_sort(
      clusters.begin(),
      clusters.end(),
      [](const auto & a, const auto & b) {
        return a.size() > b.size();
    });

    const auto & cluster_1 = clusters[0];
    const auto & cluster_2 = clusters[1];

    // TODO: convert representative rays into leg points.
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

    // TODO: run geometry sanity checks and return the midpoint.
    if (x_1 <= 0.0 || x_2 <= 0.0) {
      RCLCPP_WARN(
        get_logger(),
        "Invalid shelf leg geometry: leg points must be in front of the robot, got x1=%.3f, x2=%.3f",
        x_1,
        x_2);
      return std::nullopt;
    }

    double leg_separation = std::abs(y_1 - y_2);
    if (leg_separation < min_leg_separation_) {
      RCLCPP_WARN(
        get_logger(),
        "Invalid shelf leg geometry: lateral separation %.3f is smaller than minimum %.3f",
        leg_separation,
        min_leg_separation_);
      return std::nullopt;
    }

    double x_difference = std::abs(x_1 - x_2);
    if (x_difference > max_x_difference_) {
      RCLCPP_WARN(
        get_logger(),
        "Invalid shelf leg geometry: x difference %.3f is larger than maximum %.3f",
        x_difference,
        max_x_difference_);
      return std::nullopt;
    }

    const double x = (x_1 + x_2) / 2;
    const double y = (y_1 + y_2) / 2;

    if (x <= 0.0) {
      RCLCPP_WARN(
        get_logger(),
        "Invalid cart_frame: midpoint x %.3f must be positive",
        x);
      return std::nullopt;
    }

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

  bool perform_final_approach(const CartFrame & cart_frame)
  {
    // TODO: rotate toward cart_frame with timeout.
    const double target_yaw = std::atan2(cart_frame.y, cart_frame.x);

    if (std::abs(target_yaw) > max_target_yaw_) {
      RCLCPP_WARN(
        get_logger(),
        "Invalid target_yaw: target yaw  %.3f is larger than maximum %.3f",
        target_yaw,
        max_target_yaw_
      );
      publish_stop();
      return false;
    }

    if (std::abs(target_yaw) < yaw_tolerance_) {
      RCLCPP_INFO(get_logger(), "Target yaw is within tolerance, skipping rotation");
    }

    const double rotate_time = std::abs(target_yaw) / rotate_speed_;
    (void)rotate_time;

    // TODO: drive toward cart_frame with conservative_offset_.
    // TODO: drive forward 0.30 m more.
    // TODO: publish /elevator_up once after reaching the shelf underside.
    RCLCPP_WARN(get_logger(), "final approach movement is not implemented yet");
    return false;
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
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ApproachServiceServer>());
  rclcpp::shutdown();
  return 0;
}
