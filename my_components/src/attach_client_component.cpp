#include <chrono>
#include <memory>

#include "attach_shelf/srv/go_to_loading.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

namespace my_components
{

// Runtime-loaded client component. Loading this node is the explicit Task 2
// trigger: it calls /approach_shelf once and then lets the container exit after
// the response is received.
class AttachClient : public rclcpp::Node
{
public:
  explicit AttachClient(const rclcpp::NodeOptions & options)
      : Node("attach_client", options), request_sent_(false)
  {
    client_ = create_client<attach_shelf::srv::GoToLoading>("/approach_shelf");
    timer_ = create_wall_timer(500ms, std::bind(&AttachClient::on_timer, this));
    RCLCPP_INFO(get_logger(), "attach_client component loaded; waiting for /approach_shelf");
  }

private:
  using ServiceResponseFuture =
      rclcpp::Client<attach_shelf::srv::GoToLoading>::SharedFuture;

  void on_timer()
  {
    if (request_sent_) {
      return;
    }

    if (!client_->wait_for_service(100ms)) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
                           "Waiting for /approach_shelf service...");
      return;
    }

    auto request = std::make_shared<attach_shelf::srv::GoToLoading::Request>();
    request->attach_to_shelf = true;

    request_sent_ = true;
    timer_->cancel();
    RCLCPP_INFO(get_logger(), "Calling /approach_shelf with attach_to_shelf=true");
    client_->async_send_request(
        request, std::bind(&AttachClient::on_response, this, std::placeholders::_1));
  }

  void on_response(ServiceResponseFuture future)
  {
    const auto response = future.get();
    if (response->complete) {
      RCLCPP_INFO(get_logger(), "/approach_shelf completed successfully");
    } else {
      RCLCPP_ERROR(get_logger(), "/approach_shelf failed");
    }
    rclcpp::shutdown();
  }

  rclcpp::Client<attach_shelf::srv::GoToLoading>::SharedPtr client_;
  rclcpp::TimerBase::SharedPtr timer_;
  bool request_sent_;
};

}  // namespace my_components

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(my_components::AttachClient)
