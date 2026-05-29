#include <memory>
#include <thread>
#include <cmath>
#include <mutex>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "interfaces/action/navigate.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "std_msgs/msg/empty.hpp"

using namespace std::placeholders;

namespace nav_action_server_lib
{

class NavActionServer : public rclcpp::Node
{
public:
  using Navigate = interfaces::action::Navigate;
  using GoalHandleNavigate = rclcpp_action::ServerGoalHandle<Navigate>;

  explicit NavActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
    : Node("nav_action_server", options)
  {
    // Initialize TF2 buffer and listener for transform lookups
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Odometry subscription used as fallback when TF2 is unavailable
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10, std::bind(&NavActionServer::odom_callback, this, _1));

    // Create the action server on the "navigate" action topic
    action_server_ = rclcpp_action::create_server<Navigate>(
      this, "navigate",
      std::bind(&NavActionServer::handle_goal, this, _1, _2),
      std::bind(&NavActionServer::handle_cancel, this, _1),
      std::bind(&NavActionServer::handle_accepted, this, _1));

    // Velocity command publisher
    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    // Listen for shutdown requests from the client
    shutdown_sub_ = this->create_subscription<std_msgs::msg::Empty>(
      "/shutdown", 10, [this](const std_msgs::msg::Empty::SharedPtr) {
        RCLCPP_INFO(this->get_logger(), "Shutdown request received, terminating node.");
        rclcpp::shutdown();
      });
  }

private:
  rclcpp_action::Server<Navigate>::SharedPtr action_server_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr shutdown_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // Current robot pose, updated by odom_callback
  double current_x_     = 0.0;
  double current_y_     = 0.0;
  double current_yaw_   = 0.0;
  bool   odom_received_ = false;
  std::mutex odom_mutex_;

  // Updates the stored pose each time an odometry message arrives
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(odom_mutex_);
    current_x_   = msg->pose.pose.position.x;
    current_y_   = msg->pose.pose.position.y;
    current_yaw_ = tf2::getYaw(msg->pose.pose.orientation);
    odom_received_ = true;
  }

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const Navigate::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(), "Goal received — target: x=%.2f, y=%.2f", goal->x, goal->y);
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleNavigate>)
  {
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleNavigate> goal_handle)
  {
    // Execute the goal in a separate thread to avoid blocking the executor
    std::thread{std::bind(&NavActionServer::execute, this, _1), goal_handle}.detach();
  }

  void execute(const std::shared_ptr<GoalHandleNavigate> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Execution started.");
    const auto goal = goal_handle->get_goal();
    auto feedback   = std::make_shared<Navigate::Feedback>();
    auto result     = std::make_shared<Navigate::Result>();
    rclcpp::Rate loop_rate(10);

    while (rclcpp::ok()) {

      // Check if a cancel request has been received
      if (goal_handle->is_canceling()) {
        result->success = false;
        goal_handle->canceled(result);
        geometry_msgs::msg::Twist stop;
        stop.linear.x  = 0.0;
        stop.angular.z = 0.0;
        publisher_->publish(stop);
        RCLCPP_INFO(this->get_logger(), "Goal canceled — robot stopped.");
        return;
      }

      double cx = 0.0, cy = 0.0, cyaw = 0.0;
      bool pose_available = false;

      // Primary: attempt to get pose from TF2
      try {
        geometry_msgs::msg::TransformStamped tf = tf_buffer_->lookupTransform(
          "odom", "base_footprint", tf2::TimePointZero);
        cx   = tf.transform.translation.x;
        cy   = tf.transform.translation.y;
        cyaw = tf2::getYaw(tf.transform.rotation);
        pose_available = true;
      } catch (const tf2::TransformException & ex) {
        RCLCPP_DEBUG_THROTTLE(
          this->get_logger(), *this->get_clock(), 2000,
          "TF2 unavailable: %s — switching to odometry.", ex.what());
      }

      // Fallback: use odometry data directly
      if (!pose_available) {
        std::lock_guard<std::mutex> lock(odom_mutex_);
        if (!odom_received_) {
          RCLCPP_INFO_THROTTLE(
            this->get_logger(), *this->get_clock(), 2000,
            "Waiting for pose data...");
          loop_rate.sleep();
          continue;
        }
        cx   = current_x_;
        cy   = current_y_;
        cyaw = current_yaw_;
      }

      // Compute distance and heading errors
      double dist_err  = std::sqrt(std::pow(goal->x - cx, 2) + std::pow(goal->y - cy, 2));
      double heading   = std::atan2(goal->y - cy, goal->x - cx);
      double angle_err = std::atan2(std::sin(heading - cyaw), std::cos(heading - cyaw));

      geometry_msgs::msg::Twist cmd;
      if (dist_err > 0.20) {
        if (std::abs(angle_err) > 0.15) {
          // Rotate in place to align with the target
          cmd.angular.z = 1.0 * angle_err;
          cmd.linear.x  = 0.0;
        } else {
          // Move forward proportionally to remaining distance
          cmd.linear.x  = 0.4 * dist_err;
          cmd.angular.z = 0.0;
        }
      } else {
        // Goal reached — stop the robot
        cmd.linear.x  = 0.0;
        cmd.angular.z = 0.0;
        publisher_->publish(cmd);
        break;
      }

      publisher_->publish(cmd);
      feedback->distance_remaining = dist_err;
      RCLCPP_INFO(this->get_logger(), "Distance to goal: %.3f m", dist_err);
      goal_handle->publish_feedback(feedback);
      loop_rate.sleep();
    }

    result->success = true;
    goal_handle->succeed(result);
    RCLCPP_INFO(this->get_logger(), "Goal reached successfully.");
  }
};

} // namespace nav_action_server_lib

RCLCPP_COMPONENTS_REGISTER_NODE(nav_action_server_lib::NavActionServer)
