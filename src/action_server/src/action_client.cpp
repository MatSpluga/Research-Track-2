#include <memory>
#include <thread>
#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "interfaces/action/navigate.hpp"
#include "std_msgs/msg/empty.hpp"

using namespace std::placeholders;

namespace nav_action_client_lib
{

// NavActionClient: sends navigation goals to the action server
// and handles user commands from the terminal via a dedicated thread.
class NavActionClient : public rclcpp::Node
{
public:
  using Navigate = interfaces::action::Navigate;
  using GoalHandleNavigate = rclcpp_action::ClientGoalHandle<Navigate>;

  explicit NavActionClient(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
    : Node("nav_action_client", options)
  {
    // Create the action client connected to the "navigate" server
    client_ptr_ = rclcpp_action::create_client<Navigate>(this, "navigate");

    // Publisher used to signal the server to shut down
    shutdown_pub_ = this->create_publisher<std_msgs::msg::Empty>("/shutdown", 10);

    // Start the CLI loop in a separate thread
    cli_thread_ = std::thread(&NavActionClient::cli_loop, this);
  }

  ~NavActionClient()
  {
    if (cli_thread_.joinable()) {
      cli_thread_.detach();
    }
  }

private:
  rclcpp_action::Client<Navigate>::SharedPtr client_ptr_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr shutdown_pub_;
  std::thread cli_thread_;
  std::shared_ptr<GoalHandleNavigate> current_goal_handle_;

  // Runs in a separate thread, reads user input and dispatches commands
  void cli_loop()
  {
    while (rclcpp::ok()) {
      std::cout << "\n=== Navigation Control ===\n";
      std::cout << " 'g' — send a new goal\n";
      std::cout << " 'c' — cancel current goal\n";
      std::cout << " 'q' — quit\n";
      std::cout << "Input: ";

      std::string input;
      if (!std::getline(std::cin, input)) {
        continue;
      }

      if (input == "c" || input == "C") {
        if (current_goal_handle_) {
          RCLCPP_INFO(this->get_logger(), "Canceling active goal...");
          client_ptr_->async_cancel_goal(current_goal_handle_);
          current_goal_handle_ = nullptr;
        } else {
          RCLCPP_WARN(this->get_logger(), "No goal is currently active.");
        }
        continue;
      }

      if (input == "q" || input == "Q") {
        if (current_goal_handle_) {
          RCLCPP_INFO(this->get_logger(), "Canceling goal before shutdown...");
          client_ptr_->async_cancel_goal(current_goal_handle_);
          std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
        RCLCPP_INFO(this->get_logger(), "Sending shutdown signal to server...");
        shutdown_pub_->publish(std_msgs::msg::Empty());
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        RCLCPP_INFO(this->get_logger(), "Shutting down client...");
        rclcpp::shutdown();
        break;
      }

      if (input == "g" || input == "G") {
        std::cout << "Enter target coordinates as <x y theta> (e.g. 3.0 1.5 0.0):\n";
        std::string coords;
        if (std::getline(std::cin, coords)) {
          double x, y, theta;
          if (sscanf(coords.c_str(), "%lf %lf %lf", &x, &y, &theta) == 3) {
            send_goal(x, y, theta);
          } else {
            std::cout << "Invalid format. Please enter three numeric values.\n";
          }
        }
      }
    }
  }

  // Builds and sends a Navigate goal to the action server
  void send_goal(double x, double y, double theta)
  {
    if (!client_ptr_->wait_for_action_server(std::chrono::seconds(5))) {
      RCLCPP_ERROR(this->get_logger(), "Action server not reachable.");
      return;
    }

    auto goal_msg  = Navigate::Goal();
    goal_msg.x     = x;
    goal_msg.y     = y;
    goal_msg.theta = theta;

    RCLCPP_INFO(this->get_logger(), "Sending goal: x=%.2f, y=%.2f, theta=%.2f", x, y, theta);

    auto options = rclcpp_action::Client<Navigate>::SendGoalOptions();
    options.goal_response_callback =
      std::bind(&NavActionClient::goal_response_callback, this, _1);
    options.result_callback =
      std::bind(&NavActionClient::result_callback, this, _1);

    client_ptr_->async_send_goal(goal_msg, options);
  }

  // Called when the server accepts or rejects the goal
  void goal_response_callback(GoalHandleNavigate::SharedPtr goal_handle)
  {
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Goal rejected by the server.");
      current_goal_handle_ = nullptr;
    } else {
      RCLCPP_INFO(this->get_logger(), "Goal accepted, navigation in progress...");
      current_goal_handle_ = goal_handle;
    }
  }

  // Called when the goal terminates (success, abort, or cancel)
  void result_callback(const GoalHandleNavigate::WrappedResult & result)
  {
    current_goal_handle_ = nullptr;
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        RCLCPP_INFO(this->get_logger(), "Navigation completed successfully.");
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(this->get_logger(), "Navigation was aborted.");
        return;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_WARN(this->get_logger(), "Navigation was canceled.");
        return;
      default:
        RCLCPP_ERROR(this->get_logger(), "Unexpected result code.");
        return;
    }

    if (result.result->success) {
      RCLCPP_INFO(this->get_logger(), "Robot reached the target pose.");
    } else {
      RCLCPP_WARN(this->get_logger(), "Robot did not reach the target pose.");
    }
  }
};

} // namespace nav_action_client_lib

RCLCPP_COMPONENTS_REGISTER_NODE(nav_action_client_lib::NavActionClient)
