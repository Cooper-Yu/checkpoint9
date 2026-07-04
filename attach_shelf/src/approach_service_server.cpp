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
    conservative_offset_(0.15)
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
    // TODO: choose the two largest valid clusters.
    // TODO: convert representative rays into leg points.
    // TODO: run geometry sanity checks and return the midpoint.
    RCLCPP_WARN(get_logger(), "cart_frame detection is not implemented yet");
    return std::nullopt;
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
    const double target_yaw = std::atan2(cart_frame.y, cart_frame.x);
    (void)target_yaw;

    // TODO: rotate toward cart_frame with timeout.
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
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ApproachServiceServer>());
  rclcpp::shutdown();
  return 0;
}
