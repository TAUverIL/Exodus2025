// Stanley Algorithm CPP Code

#include <algorithm>
#include <string>
#include <limits>
#include <memory>
#include <vector>
#include <utility>
#include <cmath>

#include "stanley_v1/stanley_controller.hpp"
#include "angles/angles.h"
#include "nav2_core/exceptions.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_costmap_2d/costmap_filters/filter_values.hpp"
#include "visualization_msgs/msg/marker.hpp"

using nav2_util::declare_parameter_if_not_declared;
using nav2_util::geometry_utils::euclidean_distance;
using rcl_interfaces::msg::ParameterType;

namespace stanley_controller
{
void StanleyController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  auto node = parent.lock();
  node_ = parent;
  if (!node) {
    throw nav2_core::PlannerException("Unable to lock node!");
  }

  costmap_ros_ = costmap_ros;
  costmap_ = costmap_ros_->getCostmap();
  tf_buffer_ = tf;
  plugin_name_ = name;
  logger_ = node->get_logger();
  clock_ = node->get_clock();

  double transform_tolerance = 0.1;
  // double control_frequency = 20.0;

  // FIXME - check what this does
  goal_dist_tol_ = 0.25;  // reasonable default before first update

  declare_parameter_if_not_declared(
    node, plugin_name_ + ".k_steer", rclcpp::ParameterValue(0.5));

  declare_parameter_if_not_declared(
    node, plugin_name_ + ".k_vel", rclcpp::ParameterValue(1.0));

  declare_parameter_if_not_declared(
    node, plugin_name_ + ".epsilon", rclcpp::ParameterValue(1e-9));

  declare_parameter_if_not_declared(
    node, plugin_name_ + ".wheel_base", rclcpp::ParameterValue(1.6));

  // number of waypoints to use when calculating global path yaw
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".max_wpts", rclcpp::ParameterValue(20));

  declare_parameter_if_not_declared(
    node, plugin_name_ + ".min_wpts", rclcpp::ParameterValue(5));

  declare_parameter_if_not_declared(
    node, plugin_name_ + ".desired_linear_vel", rclcpp::ParameterValue(1.0));
  
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".use_collision_detection", rclcpp::ParameterValue(true));

  declare_parameter_if_not_declared(
    node, plugin_name_ + ".allow_reversing", rclcpp::ParameterValue(false));

  declare_parameter_if_not_declared(
    node, plugin_name_ + ".max_steer_angle", rclcpp::ParameterValue(85.0));

  // FIXME - see how we can change this for Stanley
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".max_robot_pose_search_dist",
    rclcpp::ParameterValue(getCostmapMaxExtent()));
  
  // FIXME - see how we can change this for Stanley
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".transform_tolerance", rclcpp::ParameterValue(0.1));

  declare_parameter_if_not_declared(
    node, plugin_name_ + ".slow_down_dist", rclcpp::ParameterValue(1.0));

  declare_parameter_if_not_declared(
    node, plugin_name_ + ".stop_dist", rclcpp::ParameterValue(0.1));

  declare_parameter_if_not_declared(
    node, plugin_name_ + ".min_speed", rclcpp::ParameterValue(0.1));
    
  node->get_parameter(plugin_name_ + ".k_steer", k_steer_);
  node->get_parameter(plugin_name_ + ".k_vel", k_vel_);
  node->get_parameter(plugin_name_ + ".epsilon", epsilon_);
  node->get_parameter(plugin_name_ + ".wheel_base", wheel_base_);
  node->get_parameter(plugin_name_ + ".max_wpts", max_wpts_);
  node->get_parameter(plugin_name_ + ".min_wpts", min_wpts_);
  node->get_parameter(plugin_name_ + ".desired_linear_vel", desired_linear_vel_);
  node->get_parameter(plugin_name_ + ".use_collision_detection", use_collision_detection_);
  node->get_parameter(plugin_name_ + ".allow_reversing", allow_reversing_);
  node->get_parameter(plugin_name_ + ".max_steer_angle", max_steer_angle_);
  node->get_parameter(plugin_name_ + ".max_robot_pose_search_dist", max_robot_pose_search_dist_);
  node->get_parameter(plugin_name_ + ".transform_tolerance", transform_tolerance);
  node->get_parameter(plugin_name_ + ".slow_down_dist", slow_down_dist_);
  node->get_parameter(plugin_name_ + ".stop_dist", stop_dist_);
  node->get_parameter(plugin_name_ + ".min_speed", min_speed_);

  // FIXME - check how this affects Stanley
  base_desired_linear_vel_ = desired_linear_vel_;

  // publishers
  global_path_pub_ = node->create_publisher<nav_msgs::msg::Path>("received_global_plan", 1);
  marker_pub_ = node->create_publisher<visualization_msgs::msg::Marker>("/visualization_marker", 1);
  // end_goal_speed_limit_pub_ = 
  //     node->create_publisher<nav2_msgs::msg::SpeedLimit>("/end_goal_speed_limit", rclcpp::QoS(1),
  //         [this](const nav2_msgs::msg::SpeedLimit::SharedPtr msg){
  //             end_goal_speed_limit_ = msg->speed_limit;
  //             end_goal_percentage_ = msg->percentage;
  //         }
  //     );

  // initialize collision checker and set costmap
  collision_checker_ = std::make_unique<nav2_costmap_2d::
      FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *>>(costmap_);
  collision_checker_->setCostmap(costmap_);
}

void StanleyController::cleanup()
{
  RCLCPP_INFO(
    logger_,
    "Cleaning up controller: %s of type"
    "stanley_controller::StanleyController",
    plugin_name_.c_str());
    global_path_pub_.reset();
    marker_pub_.reset();
    // end_goal_speed_limit_pub_.reset();
}

void StanleyController::activate()
{
  RCLCPP_INFO(
    logger_,
    "Activating controller: %s of type "
    "stanley_controller::StanleyController",
    plugin_name_.c_str());
    global_path_pub_->on_activate();
    // end_goal_speed_limit_pub_->on_activate();
    // Add callback for dynamic parameters
    auto node = node_.lock();
    dyn_params_handler_ = node->add_on_set_parameters_callback(
    std::bind(
      &StanleyController::dynamicParametersCallback,
      this, std::placeholders::_1)
    );
}

void StanleyController::deactivate()
{
  RCLCPP_INFO(
    logger_,
    "Deactivating controller: %s of type "
    "stanley_controller::StanleyController",
    plugin_name_.c_str());
    global_path_pub_->on_deactivate();
    // end_goal_speed_limit_pub_->on_deactivate();
    dyn_params_handler_.reset();
}

double StanleyController::computeDistance(double x1, double y1, double x2, double y2) {
  return std::sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
}

double StanleyController::radToDeg(double angle) {
  return (angle * 180) / M_PI;
}

double StanleyController::computeRoverYaw() {
  geometry_msgs::msg::TransformStamped tf_map_bl =
    tf_buffer_->lookupTransform("map",           // target frame
                                "base_link",     // source frame
                                tf2::TimePointZero);  // latest available

  // CONVERT QUATERNION TO YAW EXPLICITLY
  tf2::Quaternion q(
      tf_map_bl.transform.rotation.x,
      tf_map_bl.transform.rotation.y,
      tf_map_bl.transform.rotation.z,
      tf_map_bl.transform.rotation.w);

  double roll, pitch, yaw;                         // radians
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

  return yaw;
}

double StanleyController::getNormalizedAngle(double angle) {
  if (angle > M_PI) angle -= 2 * M_PI;
  if (angle < -M_PI) angle += 2 * M_PI;
  return angle;
}

void StanleyController::setPlan(const nav_msgs::msg::Path & path)
{
  global_plan_ = path;
}

std::pair<size_t, double> StanleyController::findNearestIndex(const nav_msgs::msg::Path & path,
  double x_ref, double y_ref) const {
  
  double min_dist = std::numeric_limits<double>::max();
  size_t best_idx = 0;

  for (size_t i = 0; i < path.poses.size(); ++i) {
    double px = path.poses[i].pose.position.x;
    double py = path.poses[i].pose.position.y;

    double d = StanleyController::computeDistance(px, py, x_ref, y_ref);
    if (d < min_dist) {
      min_dist = d;
      best_idx = i;
    }
  }

  return {best_idx, min_dist};

}

void StanleyController::findNearestWpt(const geometry_msgs::msg::PoseStamped & robot_pose) {
  
  nav_msgs::msg::Path transformed_plan = transformGlobalPlan(robot_pose);

  // Find the waypoint closest to the robot origin (0,0) in that frame
  auto [idx, min_dist] = findNearestIndex(transformed_plan, 0.0, 0.0);

  double Closest_Point_X = transformed_plan.poses[idx].pose.position.x;
  double Closest_Point_Y = transformed_plan.poses[idx].pose.position.y;

  RCLCPP_INFO(logger_,
    "Closest Point (X Y) in Robot TF: (%.2f %.2f)", Closest_Point_X, Closest_Point_Y);
  RCLCPP_INFO(logger_,
    "Distance to nearest point: %.2f, with Index: %zu", min_dist, idx);

}

double StanleyController::computeCrossTrackError(const geometry_msgs::msg::PoseStamped & robot_pose) {
  
  // Transform the global plan into the robot frame
  nav_msgs::msg::Path transformed_plan = StanleyController::transformGlobalPlan(robot_pose);

  // Compute the front‐axle location with respect to the center of the rover
  double curr_yaw = StanleyController::computeRoverYaw();
  // double front_x  = (wheel_base_ / 2.0) * std::cos(curr_yaw);
  // double front_y  = (wheel_base_ / 2.0) * std::sin(curr_yaw);
  double front_x  = wheel_base_ / 2.0;
  double front_y  = 0.0;

  // Find the closest waypoint to the front axle
  auto [target_idx, min_dist] = StanleyController::findNearestIndex(transformed_plan, front_x, front_y);

  // Build the unit normal vector (90° to heading)
  // double nx = -std::cos(curr_yaw + M_PI / 2.0);
  // double ny = -std::sin(curr_yaw + M_PI / 2.0);
  double nx = -std::sin(curr_yaw);
  double ny = std::cos(curr_yaw);

  // Get the coordinates of that closest point
  double Closest_Point_X = transformed_plan.poses[target_idx].pose.position.x;
  double Closest_Point_Y = transformed_plan.poses[target_idx].pose.position.y;

  // Compute cross‐track error using dot product to project onto the unit vector
  double cte = (Closest_Point_X - front_x) * nx + (Closest_Point_Y - front_y) * ny;

  return cte;

}

double StanleyController::computeSpeed(double k, double target_vel, geometry_msgs::msg::Twist curr_vel) {
  return k * (target_vel - curr_vel.linear.x);
}

double StanleyController::computeSteeringAngle(const geometry_msgs::msg::PoseStamped & robot_pose,
    double vel) {

  double rover_yaw = StanleyController::computeRoverYaw();
  double curr_yaw_deg = StanleyController::radToDeg(rover_yaw);
  // RCLCPP_INFO(logger_, "Yaw using Quarternions: %.3f, in degrees: %.3f", rover_yaw, StanleyController::radToDeg(rover_yaw));

  int last_wpt = global_plan_.poses.size() - 1;
  int begin_wpt = (max_wpts_ <= last_wpt) ? min_wpts_ : 0;
  int end_wpt = std::min(max_wpts_, last_wpt);

  double first_x = global_plan_.poses[begin_wpt].pose.position.x;
  double last_x = global_plan_.poses[end_wpt].pose.position.x;

  double first_y = global_plan_.poses[begin_wpt].pose.position.y;
  double last_y = global_plan_.poses[end_wpt].pose.position.y;
  RCLCPP_INFO(logger_, "Begin Wpt X: %.3f, Last Wpt X: %.3f, Final Wpt X: %.3f", 
    first_x, last_x, global_plan_.poses[last_wpt].pose.position.x);

  double yaw_path = std::atan2(last_y - first_y, last_x - first_x);
  double yaw_path_deg = StanleyController::radToDeg(yaw_path);

  // angle_err is the angle error between the rover and global path angles
  double angle_err = StanleyController::getNormalizedAngle(yaw_path - rover_yaw);
  // double angle_err_deg = StanleyController::radToDeg(angle_err);
  
  // track_err_corr is the adjustment to rover angle to correct the cross track error
  double cte = StanleyController::computeCrossTrackError(robot_pose);
  double xtrack_err_corr = std::atan2(k_steer_ * cte, vel);
  double xtrack_err_corr_deg = StanleyController::radToDeg(xtrack_err_corr);

  // delta is the final adjustment to the rover angle, adds the angle_err adjustment
  double pre_delta = angle_err + xtrack_err_corr;
  double max_steer_rad = (M_PI / 180) * max_steer_angle_;
  double delta = (pre_delta > max_steer_rad) ? max_steer_rad :
                 (pre_delta < -max_steer_rad) ? -max_steer_rad :
                  pre_delta;

  RCLCPP_INFO(logger_, "Yaw Path: %.3f, Curr Yaw: %.3f, Angle Err: %.3f, X-track Err: %.3f, Delta: %.3f", 
    yaw_path_deg, curr_yaw_deg, StanleyController::radToDeg(angle_err), xtrack_err_corr_deg, StanleyController::radToDeg(delta));
  return delta;

}

geometry_msgs::msg::TwistStamped StanleyController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & speed,
  nav2_core::GoalChecker * goal_checker)
{
  std::lock_guard<std::mutex> lock_reinit(mutex_);
  nav2_costmap_2d::Costmap2D * costmap = costmap_ros_->getCostmap();
  std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*(costmap->getMutex()));

  // Update goal tolerances
  geometry_msgs::msg::Pose pose_tolerance;
  geometry_msgs::msg::Twist vel_tolerance;
  if (!goal_checker->getTolerances(pose_tolerance, vel_tolerance)) {
    RCLCPP_WARN(logger_, "Unable to retrieve goal checker's tolerances!");
  } else {
    goal_dist_tol_ = pose_tolerance.position.x;
  }

  // Transform path to robot base frame
  auto transformed_plan = transformGlobalPlan(pose);

  double target_angle;

  double desired_lin_vel = std::min(desired_linear_vel_, StanleyController::adjustSpeedLimit(pose));

  // FIXME - check if we need to use speed or desired_lin_vel
  target_angle = computeSteeringAngle(pose, desired_lin_vel);

  double speed_update;
  speed_update = computeSpeed(k_vel_, desired_lin_vel, speed) + speed.linear.x;

  double angle_update;
  angle_update = speed_update * std::tan(target_angle) / wheel_base_;

  // Collision Checking
  if (use_collision_detection_) {
    // const double check_time = 1.5;                           // seconds
    // const double max_horizon = max_robot_pose_search_dist_;
    // const double horizon = std::max( min_horizon_,           // e.g. 0.5 m minimum
    // std::min(max_horizon, speed.linear.x * check_time ));             // up to max_horizon_

    if (isCollisionImminent(pose, speed_update, StanleyController::computeRoverYaw()))
    {
      speed_update = 0.0;
      angle_update = 0.0;
      throw nav2_core::PlannerException("StanleyController: Collision detected ahead!");
    }
  }

  // RCLCPP_INFO(logger_, "Collision Imminent: %d", isCollisionImminent(pose, speed_update, speed.angular.z));
  
  geometry_msgs::msg::TwistStamped cmd_vel;
  cmd_vel.header = pose.header;
  cmd_vel.twist.linear.x  = speed_update;
  cmd_vel.twist.angular.z = angle_update;               // yaw rate = v * curvature

  RCLCPP_INFO(logger_, "Twist Ang. Cmd: %.3f, Curr Vel: %.3f", StanleyController::radToDeg(cmd_vel.twist.angular.z), speed.linear.x);
  // RCLCPP_INFO(logger_, "Cmd Vel Rover Pose X: %.3f, Cmd Vel Rover Pose X: %.3f", pose.pose.position.x, pose.pose.position.y);
  

  // Markers

  visualization_msgs::msg::Marker marker;
  marker.header = global_plan_.header;
  marker.ns = "stanley_controller";
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
  marker.action = visualization_msgs::msg::Marker::ADD;

  // Size of each sphere
  marker.scale.x = 0.1;
  marker.scale.y = 0.1;
  marker.scale.z = 0.1;

  // Green color
  marker.points.clear();
  marker.colors.clear();

  double front_x  = wheel_base_ / 2.0;
  double front_y  = 0.0;

  for (size_t i = 0; i < global_plan_.poses.size(); ++i)
  {
      const auto & pose_stamped = global_plan_.poses[i];

      geometry_msgs::msg::Point pt;
      pt.x = pose_stamped.pose.position.x;
      pt.y = pose_stamped.pose.position.y;
      pt.z = 0.1;
      
      auto idx = StanleyController::findNearestIndex(transformed_plan, front_x, front_y).first;
      std_msgs::msg::ColorRGBA color;
      if (static_cast<size_t>(i) == idx || static_cast<size_t>(i) == static_cast<size_t>(max_wpts_) || static_cast<size_t>(i) == static_cast<size_t>(min_wpts_))
      {
          color.r = 1.0f;
          color.g = 0.0f;
          color.b = 0.0f;
          color.a = 1.0f; // Red for target
          RCLCPP_INFO(logger_, "Closest Idx: %ld", idx);
      }
      else
      {
          color.r = 0.0f;
          color.g = 1.0f;
          color.b = 0.0f;
          color.a = 1.0f; // Green for regular points
      }
      marker.points.push_back(pt);
      marker.colors.push_back(color);
  }

  // double projection_time = 1.0;   // FIXME - change if necessary

  // visualization_msgs::msg::Marker traj_pt;
  // traj_pt.header.frame_id = "base_link";          // <-- key line
  // traj_pt.ns   = "stanley_controller";
  // traj_pt.id   = 0;
  // traj_pt.type = visualization_msgs::msg::Marker::SPHERE;
  // traj_pt.action = visualization_msgs::msg::Marker::ADD;

  // traj_pt.pose.position.x = projection_time * (speed.linear.x * cos(StanleyController::computeRoverYaw()));  
  // traj_pt.pose.position.y = projection_time * (speed.linear.y * sin(StanleyController::computeRoverYaw()));
  // traj_pt.pose.position.z = 0.5;           // or a small positive value
  // // traj_pt.pose.orientation.w = projection_time * speed.angular.z;

  // traj_pt.scale.x = traj_pt.scale.y = traj_pt.scale.z = 0.5;  // 8-cm sphere
  // traj_pt.color.r = 0.8f;
  // traj_pt.color.g = 0.4f;
  // traj_pt.color.b = 0.0f;
  // traj_pt.color.a = 1.0f;

  // marker_pub_->publish(traj_pt);
  marker_pub_->publish(marker);

  // End Markers
  return cmd_vel;

}

// double StanleyController::approachVelocityScalingFactor(
//   const nav_msgs::msg::Path & transformed_path
// ) const
// {
//   // Waiting to apply the threshold based on integrated distance ensures we don't
//   // erroneously apply approach scaling on curvy paths that are contained in a large local costmap.
//   double remaining_distance = nav2_util::geometry_utils::calculate_path_length(transformed_path);
//   if (remaining_distance < approach_velocity_scaling_dist_) {
//     auto & last = transformed_path.poses.back();
//     double distance_to_last_pose = std::hypot(last.pose.position.x, last.pose.position.y);
//     return distance_to_last_pose / approach_velocity_scaling_dist_;
//   } else {
//     return 1.0;
//   }
// }

// void StanleyController::applyApproachVelocityScaling(
//   const nav_msgs::msg::Path & path,
//   double & linear_vel
// ) const
// {
//   double approach_vel = desired_linear_vel_;
//   double velocity_scaling = approachVelocityScalingFactor(path);
//   double unbounded_vel = approach_vel * velocity_scaling;
//   if (unbounded_vel < min_approach_linear_velocity_) {
//     approach_vel = min_approach_linear_velocity_;
//   } else {
//     approach_vel = unbounded_vel;
//   }

//   // Use the lowest velocity between approach and other constraints, if all overlapping
//   linear_vel = std::min(linear_vel, approach_vel);
// }

// void StanleyController::applyConstraints(
//   const double & curvature, const geometry_msgs::msg::Twist & curr_speed,
//   const double & pose_cost, double & linear_vel, const nav_msgs::msg::Path & path, double & sign)
// {
//   double cost_vel = linear_vel;

//   // limit the linear velocity by proximity to obstacles
//   if (use_cost_regulated_linear_velocity_scaling_ &&
//     pose_cost != static_cast<double>(nav2_costmap_2d::NO_INFORMATION) &&
//     pose_cost != static_cast<double>(nav2_costmap_2d::FREE_SPACE))
//   {
//     const double inscribed_radius = costmap_ros_->getLayeredCostmap()->getInscribedRadius();
//     const double min_distance_to_obstacle = (-1.0 / inflation_cost_scaling_factor_) *
//       std::log(pose_cost / (nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE - 1)) + inscribed_radius;

//     if (min_distance_to_obstacle < cost_scaling_dist_) {
//       cost_vel *= cost_scaling_gain_ * min_distance_to_obstacle / cost_scaling_dist_;
//     }
//   }

  // limit the linear velocity by curvature
//   double max_vel_for_curve = std::sqrt(max_lateral_accel_ / std::abs(curvature));

//   // Apply constraints
//   linear_vel = std::min(
//     {linear_vel, max_vel_for_curve, cost_vel,
//       std::abs(curr_speed.linear.x) + max_linear_accel_ * control_duration_});

//   applyApproachVelocityScaling(path, linear_vel);

//   // Ensure the linear velocity is not below the minimum allowed linear velocity
//   linear_vel = std::max(linear_vel, min_linear_velocity_);
//   linear_vel = sign * linear_vel;
// }

// void StanleyController::rotateToHeading(
//   double & linear_vel, double & angular_vel,
//   const double & angle_to_path, const geometry_msgs::msg::Twist & curr_speed)
// {
//   // Rotate in place using max angular velocity / acceleration possible
//   linear_vel = 0.0;
//   const double sign = angle_to_path > 0.0 ? 1.0 : -1.0;
//   angular_vel = sign * rotate_to_heading_angular_vel_;

//   const double & dt = control_duration_;
//   const double min_feasible_angular_speed = curr_speed.angular.z - max_angular_accel_ * dt;
//   const double max_feasible_angular_speed = curr_speed.angular.z + max_angular_accel_ * dt;
//   angular_vel = std::clamp(angular_vel, min_feasible_angular_speed, max_feasible_angular_speed);
// }

bool StanleyController::isCollisionImminent(
  const geometry_msgs::msg::PoseStamped & robot_pose,
  const double & linear_vel, const double & angular_vel)
{
  double rover_yaw = StanleyController::computeRoverYaw();

  // check current point is OK
  if (inCollision(robot_pose.pose.position.x, robot_pose.pose.position.y, rover_yaw))
  {
    RCLCPP_WARN(logger_, "Robot is in collision at current pose");
    return true;
  }

  // const geometry_msgs::msg::Point & robot_xy = robot_pose.pose.position;
  geometry_msgs::msg::Pose2D curr_pose;
  curr_pose.x = robot_pose.pose.position.x;
  curr_pose.y = robot_pose.pose.position.y;
  curr_pose.theta = rover_yaw;

  double projection_time = 1.0;   // FIXME - change if necessary
  curr_pose.x += projection_time * (linear_vel * cos(curr_pose.theta));
  curr_pose.y += projection_time * (linear_vel * sin(curr_pose.theta));
  curr_pose.theta += projection_time * angular_vel;

  RCLCPP_INFO(logger_, "Proj Pose X: %.3f, Proj Pose Y: %.3f, Curr Pose X: %.3f, Curr Pose Y: %.3f, isCollision: %d", 
    curr_pose.x, curr_pose.y, robot_pose.pose.position.x, robot_pose.pose.position.y,
    inCollision(curr_pose.x, curr_pose.y, curr_pose.theta));
  
  // check for collision at the projected pose
    if (inCollision(curr_pose.x, curr_pose.y, curr_pose.theta)) {
      return true;
    }

  return false;
}

bool StanleyController::inCollision(
  const double & x,
  const double & y,
  const double & theta)
{
  unsigned int mx, my;

  if (!costmap_->worldToMap(x, y, mx, my)) {
    RCLCPP_WARN_THROTTLE(
      logger_, *(clock_), 30000,
      "The dimensions of the costmap is too small to successfully check for "
      "collisions as far ahead as requested. Proceed at your own risk, slow the robot, or "
      "increase your costmap size.");
    return false;
  }

  double footprint_cost = collision_checker_->footprintCostAtPose(
    x, y, theta, costmap_ros_->getRobotFootprint());

  // RCLCPP_INFO(logger_, "Footprint X: %.3f, Footprint Y: %.3f, Footprint Theta: %.3f", x, y, theta);

  // RCLCPP_INFO(logger_, "Footprint Cost: %.3f, Lethal Obstacle: %.3f", 
  //   footprint_cost, static_cast<double>(nav2_costmap_2d::LETHAL_OBSTACLE));
  
  if (footprint_cost == static_cast<double>(nav2_costmap_2d::NO_INFORMATION) &&
    costmap_ros_->getLayeredCostmap()->isTrackingUnknown())
  {
    RCLCPP_WARN(logger_, "Footprint cost is unknown, collision check failed");
    return false;
  }

  // if occupied or unknown and not to traverse unknown space
  return footprint_cost >= static_cast<double>(nav2_costmap_2d::LETHAL_OBSTACLE);
}

void StanleyController::setSpeedLimit(
  const double & speed_limit,
  const bool & percentage)
{
  if (speed_limit == nav2_costmap_2d::NO_SPEED_LIMIT) {
    // Restore default value
    desired_linear_vel_ = base_desired_linear_vel_;
  } else {
    if (percentage) {
      // Speed limit is expressed in % from maximum speed of robot
      desired_linear_vel_ = base_desired_linear_vel_ * speed_limit / 100.0;
    } else {
      // Speed limit is expressed in absolute value
      desired_linear_vel_ = speed_limit;
    }
  }
}

double StanleyController::adjustSpeedLimit(const geometry_msgs::msg::PoseStamped & robot_pose) 
{
  double end_goal_desired_linear_vel;
  auto transformed_plan = transformGlobalPlan(robot_pose);

  int goal_idx = transformed_plan.poses.size() - 1;
  double dx = transformed_plan.poses[goal_idx].pose.position.x;
  double dy = transformed_plan.poses[goal_idx].pose.position.y;
  double dist_to_goal = std::hypot(dx, dy);

  if (dist_to_goal < stop_dist_) end_goal_desired_linear_vel = 0.0;   // If really close to final wpt
  else if (dist_to_goal < slow_down_dist_) {                          // Close to final wpt
    double gradient = dist_to_goal / (slow_down_dist_ - stop_dist_);
    end_goal_desired_linear_vel = min_speed_ + (base_desired_linear_vel_ - min_speed_) * gradient;
  }
  else end_goal_desired_linear_vel = base_desired_linear_vel_; // Still far from final wpt

  // RCLCPP_INFO(logger_, "goal idx: %d, dist to goal: %.3f, slow dist: %.3f, stop dist: %.3f", 
  //     goal_idx, dist_to_goal, slow_down_dist_, stop_dist_);

  // RCLCPP_INFO(logger_, "within slow dist: %d, within stop dist: %d", 
  //     (dist_to_goal < slow_down_dist_), (dist_to_goal < stop_dist_));

  return end_goal_desired_linear_vel;
}

nav_msgs::msg::Path StanleyController::transformGlobalPlan(
  const geometry_msgs::msg::PoseStamped & pose)
{
  if (global_plan_.poses.empty()) {
    throw nav2_core::PlannerException("Received plan with zero length");
  }

  // let's get the pose of the robot in the frame of the plan
  geometry_msgs::msg::PoseStamped robot_pose;
  if (!transformPose(global_plan_.header.frame_id, pose, robot_pose)) {
    throw nav2_core::PlannerException("Unable to transform robot pose into global plan's frame");
  }

  // discard points on the plan that are outside the local costmap - FIXME
  double max_costmap_extent = getCostmapMaxExtent();

  auto closest_pose_upper_bound =
    nav2_util::geometry_utils::first_after_integrated_distance(
    global_plan_.poses.begin(), global_plan_.poses.end(), max_robot_pose_search_dist_);

  // First find the closest pose on the path to the robot
  // bounded by when the path turns around (if it does) so we don't get a pose from a later
  // portion of the path
  auto transformation_begin =
    nav2_util::geometry_utils::min_by(
    global_plan_.poses.begin(), closest_pose_upper_bound,
    [&robot_pose](const geometry_msgs::msg::PoseStamped & ps) {
      return euclidean_distance(robot_pose, ps);
    });

  // Find points up to max_transform_dist so we only transform them.
  auto transformation_end = std::find_if(
    transformation_begin, global_plan_.poses.end(),
    [&](const auto & pose) {
      return euclidean_distance(pose, robot_pose) > max_costmap_extent;
    });

  // Lambda to transform a PoseStamped from global frame to local
  auto transformGlobalPoseToLocal = [&](const auto & global_plan_pose) {
      geometry_msgs::msg::PoseStamped stamped_pose, transformed_pose;
      stamped_pose.header.frame_id = global_plan_.header.frame_id;
      stamped_pose.header.stamp = robot_pose.header.stamp;
      stamped_pose.pose = global_plan_pose.pose;
      transformPose(costmap_ros_->getBaseFrameID(), stamped_pose, transformed_pose);
      transformed_pose.pose.position.z = 0.0;
      return transformed_pose;
    };

  // Transform the near part of the global plan into the robot's frame of reference.
  nav_msgs::msg::Path transformed_plan;
  std::transform(
    transformation_begin, transformation_end,
    std::back_inserter(transformed_plan.poses),
    transformGlobalPoseToLocal);
  transformed_plan.header.frame_id = costmap_ros_->getBaseFrameID();
  transformed_plan.header.stamp = robot_pose.header.stamp;

  // Remove the portion of the global plan that we've already passed so we don't
  // process it on the next iteration (this is called path pruning)
  global_plan_.poses.erase(begin(global_plan_.poses), transformation_begin);
  global_path_pub_->publish(transformed_plan);

  if (transformed_plan.poses.empty()) {
    throw nav2_core::PlannerException("Resulting plan has 0 poses in it.");
  }

  return transformed_plan;
}

bool StanleyController::transformPose(
  const std::string frame,
  const geometry_msgs::msg::PoseStamped & in_pose,
  geometry_msgs::msg::PoseStamped & out_pose) const
{
  if (in_pose.header.frame_id == frame) {
    out_pose = in_pose;
    return true;
  }

  try {
    tf_buffer_->transform(in_pose, out_pose, frame, transform_tolerance_);
    out_pose.header.frame_id = frame;
    return true;
  } catch (tf2::TransformException & ex) {
    RCLCPP_ERROR(logger_, "Exception in transformPose: %s", ex.what());
  }
  return false;
}

// returns the size of the costmap (meters from center point to each side)
double StanleyController::getCostmapMaxExtent() const
{
  const double max_costmap_dim_meters = std::max(
    costmap_->getSizeInMetersX(), costmap_->getSizeInMetersY());
  return max_costmap_dim_meters / 2.0;
}

// allows us to update parameters by name during runtime
rcl_interfaces::msg::SetParametersResult StanleyController::dynamicParametersCallback(
  std::vector<rclcpp::Parameter> parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  std::lock_guard<std::mutex> lock_reinit(mutex_);

  for (auto parameter : parameters) {
    const auto & type = parameter.get_type();
    const auto & name = parameter.get_name();

    // stanley parameters (can be updated using "ros2 param set /node some_param param_value")
    
    if (type == ParameterType::PARAMETER_DOUBLE) {
      if (name == plugin_name_ + ".k_steer") k_steer_ = parameter.as_double();
      else if (name == plugin_name_ + ".k_vel") k_vel_ = parameter.as_double();
      else if (name == plugin_name_ + ".epsilon") epsilon_ = parameter.as_double();
      else if (name == plugin_name_ + ".wheel_base") wheel_base_ = parameter.as_double();
      else if (name == plugin_name_ + ".desired_linear_vel") desired_linear_vel_ = parameter.as_double();
      else if (name == plugin_name_ + ".transform_tolerance") {
          double transform_tolerance = parameter.as_double();
          transform_tolerance_ = tf2::durationFromSec(transform_tolerance);
      }
    } else if (type == ParameterType::PARAMETER_INTEGER) {
      if (name == plugin_name_ + ".max_wpts") max_wpts_ = parameter.as_int();
      else if (name == plugin_name_ + ".min_wpts") min_wpts_ = parameter.as_int();
    }
  }

  result.successful = true;
  return result;
}

}  // namespace stanley_controller

// Register this controller as a stanley_controller
PLUGINLIB_EXPORT_CLASS(
  stanley_controller::StanleyController,
  nav2_core::Controller)