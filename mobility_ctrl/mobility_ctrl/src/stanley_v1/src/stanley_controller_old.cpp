#include "nav2_core/controller.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "tf2_ros/buffer.h"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"

namespace stanley_controller
{

class StanleyController : public nav2_core::Controller
{
public:
  StanleyController() = default;
  ~StanleyController() override = default;

  // Updated to match the interface exactly.
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & weak_node,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf_buffer,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override
  {
    auto node = weak_node.lock();
    if (node) {
      RCLCPP_INFO(node->get_logger(), "Stanley Controller configured!!!!!");
    } else {
      throw std::runtime_error("Lifecycle node expired in Stanley Controller configure");
    }
  }

  void cleanup() override
  {
    // No special cleanup needed for this demo.
  }

  void activate() override
  {
    // No activation routines for this demo.
  }

  void deactivate() override
  {
    // No deactivation routines for this demo.
  }

  // NEW: Implement setPlan() to satisfy the interface.
  void setPlan(const nav_msgs::msg::Path & path) override
  {
    // For demo purposes, we won't store the plan.
    (void)path;  // suppress unused parameter warning
  }

  // Updated computeVelocityCommands: notice it no longer takes the plan as an argument.
  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) override
  {

    RCLCPP_INFO(rclcpp::get_logger("StanleyController"), "computeVelocityCommands called");
    // For this demo, we build a basic cmd_vel message.
    geometry_msgs::msg::TwistStamped cmd_vel;
  
    // Populate the header from the current pose
    cmd_vel.header = pose.header;

    // Set a constant forward velocity.
    double linear_vel = 0.1;  // m/s

    // For demonstration, assign a fixed angular velocity.
    // In a full Stanley controller, angular velocity would be computed based on errors.
    double angular_vel = 0.2; // rad/s

    // Populate the command message with the computed velocities.
    cmd_vel.twist.linear.x = linear_vel;
    cmd_vel.twist.angular.z = angular_vel;

    // Optionally, update internal state variables (like last_cmd_vel_) if needed.

    return cmd_vel;
  }

  void setSpeedLimit(const double & speed_limit, const bool & percentage) override
  {
    RCLCPP_INFO(rclcpp::get_logger("StanleyController"), "setPlan called");
    
    // For demonstration, we simply ignore these parameters.
    (void)speed_limit;
    (void)percentage;
  }

};

}  // namespace stanley_controller

PLUGINLIB_EXPORT_CLASS(stanley_controller::StanleyController, nav2_core::Controller)
