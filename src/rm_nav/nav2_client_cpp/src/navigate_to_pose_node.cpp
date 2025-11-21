#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include <chrono>
#include <memory>
#include <string>

using namespace std::chrono_literals;

class NavigateToPoseClient : public rclcpp::Node {
public:
  explicit NavigateToPoseClient(const rclcpp::NodeOptions& options = rclcpp::NodeOptions()) :
      Node("navigate_to_pose_client", options) {
    action_client_ =
        rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(this, "navigate_to_pose");

    declare_parameter<std::string>("goal_frame_id", "map");
    declare_parameter<double>("goal_x", 0.0);
    declare_parameter<double>("goal_y", 0.0);
    declare_parameter<double>("goal_yaw", 0.0);
  }

  void send_goal() {
    if (!action_client_->wait_for_action_server(5s)) {
      RCLCPP_ERROR(get_logger(), "Nav2 action server not available");
      return;
    }

    auto goal_msg = nav2_msgs::action::NavigateToPose::Goal();

    const auto   frame_id = get_parameter("goal_frame_id").as_string();
    const double x        = get_parameter("goal_x").as_double();
    const double y        = get_parameter("goal_y").as_double();
    const double yaw      = get_parameter("goal_yaw").as_double();

    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp    = now();
    pose.header.frame_id = frame_id;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, yaw);
    pose.pose.position.x  = x;
    pose.pose.position.y  = y;
    pose.pose.position.z  = 0.0;
    pose.pose.orientation = tf2::toMsg(q);

    goal_msg.pose = pose;

    auto send_goal_options =
        rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
    send_goal_options.goal_response_callback = [this](auto future) {
      auto goal_handle = future.get();
      if (!goal_handle) {
        RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
      } else {
        RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
      }
    };
    send_goal_options.feedback_callback = [this](auto, auto feedback) {
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Distance remaining: %.2f",
                           feedback->distance_remaining);
    };
    send_goal_options.result_callback = [this](const auto& result) {
      switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        RCLCPP_INFO(this->get_logger(), "Navigation succeeded");
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(this->get_logger(), "Navigation aborted");
        break;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(this->get_logger(), "Navigation canceled");
        break;
      default:
        RCLCPP_ERROR(this->get_logger(), "Unknown result code");
      }
      rclcpp::shutdown();
    };

    action_client_->async_send_goal(goal_msg, send_goal_options);
  }

private:
  rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr action_client_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto                                      node = std::make_shared<NavigateToPoseClient>();
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);
  node->send_goal();
  exec.spin();
  return 0;
}