// Stanley Algorithm HPP Code

#ifndef STANLEY__STANLEY_CONTROLLER_HPP_
#define STANLEY__STANLEY_CONTROLLER_HPP_

#include <string>
#include <vector>
#include <memory>
#include <algorithm>
#include <mutex>

#include "nav2_costmap_2d/footprint_collision_checker.hpp"
#include "nav2_core/controller.hpp"
#include "rclcpp/rclcpp.hpp"
#include "pluginlib/class_loader.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "nav2_util/odometry_utils.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "nav2_msgs/msg/speed_limit.hpp"

namespace stanley_controller
{

class StanleyController : public nav2_core::Controller
{
public:
  /**
   * @brief Constructor for stanley_controller::StanleyController
   */
  StanleyController() = default;

  /**
   * @brief Destructor for stanley_controller::StanleyController
   */
  ~StanleyController() override = default;

  /**
   * @brief Configure controller state machine
   * @param parent WeakPtr to node
   * @param name Name of plugin
   * @param tf TF buffer
   * @param costmap_ros Costmap2DROS object of environment taken from Nav2
   */
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  /**
   * @brief Cleanup controller state machine
   */
  void cleanup() override;

  /**
   * @brief Activate controller state machine
   */
  void activate() override;

  /**
   * @brief Deactivate controller state machine
   */
  void deactivate() override;

   // calculates the distance between two points (any), return double
   static double computeDistance(double x1, double y1, double x2, double y2); 

   static double radToDeg(double angle);

   double computeRoverYaw();
   
   // finds the nearest index on a path object to a certain reference point (x_ref, y_ref)
   std::pair<size_t, double> findNearestIndex(const nav_msgs::msg::Path & path,
     double x_ref = 0.0, double y_ref = 0.0) const;

   // makes a short vector of nearest waypoints, find closest
   void findNearestWpt(const geometry_msgs::msg::PoseStamped & robot_pose); 

   // Normalize angle
   static double getNormalizedAngle(double angle);
   
   // use pose and yaw (AckermannDrive msg), L (?) and wpt position
   double computeCrossTrackError(const geometry_msgs::msg::PoseStamped & robot_pose);


   double computeSpeed(double k, double target_vel, geometry_msgs::msg::Twist curr_vel);

   double computeSteeringAngle(const geometry_msgs::msg::PoseStamped & robot_pose, double vel);

  //  void StanleyController::publishAckermannDrive(double speed, double steering_angle);

  /**
   * @brief Compute the best command given the current pose and velocity, with possible debug information
   *
   * Same as above computeVelocityCommands, but with debug results.
   * If the results pointer is not null, additional information about the twists
   * evaluated will be in results after the call.
   *
   * @param pose            Current robot pose
   * @param velocity        Current robot velocity
   * @param goal_checker    Ptr to the goal checker for this task in case useful in computing commands
   * @return                Best command
   */
  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * /*goal_checker*/) override;

  /**
   * @brief nav2_core setPlan - Sets the global plan
   * @param path The global plan
   */
  void setPlan(const nav_msgs::msg::Path & path) override;

  /**
   * @brief Limits the maximum linear speed of the robot.
   * @param speed_limit expressed in absolute value (in m/s)
   * or in percentage from maximum robot speed.
   * @param percentage Setting speed limit in percentage if true
   * or in absolute values in false case.
   */
  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;

  double adjustSpeedLimit(const geometry_msgs::msg::PoseStamped & pose);

protected:
  /**
   * @brief Transforms global plan into same frame as pose and clips poses ineligible for lookaheadPoint
   * Points ineligible to be selected as a lookahead point if they are any of the following:
   * - Outside the local_costmap (collision avoidance cannot be assured)
   * @param pose pose to transform
   * @return Path in new frame
   */
  nav_msgs::msg::Path transformGlobalPlan(
    const geometry_msgs::msg::PoseStamped & pose);

  /**
   * @brief Transform a pose to another frame.
   * @param frame Frame ID to transform to
   * @param in_pose Pose input to transform
   * @param out_pose transformed output
   * @return bool if successful
   */
  bool transformPose(
    const std::string frame,
    const geometry_msgs::msg::PoseStamped & in_pose,
    geometry_msgs::msg::PoseStamped & out_pose) const;

  /**
   * @brief checks for collision at projected pose
   * @param x Pose of pose x
   * @param y Pose of pose y
   * @param theta orientation of Yaw
   * @return Whether in collision
   */
  bool inCollision(
    const double & x,
    const double & y,
    const double & theta);

  /**
   * @brief Whether collision is imminent
   * @param robot_pose Pose of robot
   * @param linear_vel linear velocity to forward project
   * @param angular_vel angular velocity to forward project
   * @return Whether collision is imminent
   */
  bool isCollisionImminent(
    const geometry_msgs::msg::PoseStamped & robot_pose,
    const double & linear_vel, const double & angular_vel);

  // /**
  //  * @brief Cost at a point
  //  * @param x Pose of pose x
  //  * @param y Pose of pose y
  //  * @return Cost of pose in costmap
  //  */
  // double costAtPose(const double & x, const double & y);

  // double approachVelocityScalingFactor(
  //   const nav_msgs::msg::Path & path
  // ) const;

  // /**
  //  * @brief Apply approach velocity scaling to the system
  //  * @param path Transformed global path
  //  * @param linear_vel robot command linear velocity input
  //  */
  // void applyApproachVelocityScaling(
  //   const nav_msgs::msg::Path & path,
  //   double & linear_vel
  // ) const;

  // /**
  //  * @brief Apply cost and curvature constraints to the system
  //  * @param curvature curvature of path
  //  * @param curr_speed Speed of robot
  //  * @param pose_cost cost at this pose
  //  * @param linear_vel robot command linear velocity input
  //  * @param path Transformed global path
  //  */
  // void applyConstraints(
  //   const double & curvature, const geometry_msgs::msg::Twist & curr_speed,
  //   const double & pose_cost, double & linear_vel, const nav_msgs::msg::Path & path, double & sign);

  /**
   * Get the greatest extent of the costmap in meters from the center.
   * @return max of distance from center in meters to edge of costmap
   */
  double getCostmapMaxExtent() const;

  /**
   * @brief Callback executed when a parameter change is detected
   * @param event ParameterEvent message
   */
  rcl_interfaces::msg::SetParametersResult
  dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters);

  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::string plugin_name_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  nav2_costmap_2d::Costmap2D * costmap_;
  rclcpp::Logger logger_ {rclcpp::get_logger("StanleyController")};
  rclcpp::Clock::SharedPtr clock_;

  // Stanley parameters
  double k_vel_, k_steer_, epsilon_;
  double wheel_base_; 
  int max_wpts_, min_wpts_;
  double desired_linear_vel_, base_desired_linear_vel_;
  double max_robot_pose_search_dist_;
  double min_horizon_;
  bool use_collision_detection_;
  bool allow_reversing_;
  tf2::Duration transform_tolerance_;
  double goal_dist_tol_;
  double slow_down_dist_, stop_dist_, min_speed_;
  double max_steer_angle_;

  nav_msgs::msg::Path global_plan_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>> global_path_pub_;
  std::unique_ptr<nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *>> collision_checker_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
  // rclcpp_lifecycle::LifecyclePublisher<nav2_msgs::msg::SpeedLimit>::SharedPtr 
  //     end_goal_speed_limit_pub_;

  // Dynamic parameters handler
  std::mutex mutex_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;

};

}  // namespace stanley_controller

#endif  // STANLEY__STANLEY_CONTROLLER_HPP_