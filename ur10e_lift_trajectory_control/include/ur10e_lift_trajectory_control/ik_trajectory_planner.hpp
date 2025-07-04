#ifndef UR10E_LIFT_TRAJECTORY_CONTROL__IK_TRAJECTORY_PLANNER_HPP_
#define UR10E_LIFT_TRAJECTORY_CONTROL__IK_TRAJECTORY_PLANNER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <vector>
#include <string>
#include <memory>

namespace ur10e_lift_trajectory_control
{

struct CartesianWaypoint
{
  double x, y, z;
  double time_from_start;
  std::string description;
};

class IKTrajectoryPlanner : public rclcpp::Node
{
public:
  IKTrajectoryPlanner();

private:
  // Publishers
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr waypoint_markers_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr path_marker_pub_;
  
  // Subscriber
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr waypoint_file_sub_;
  rclcpp::TimerBase::SharedPtr control_timer_;
  
  // Data
  std::vector<std::string> joint_names_;
  std::vector<CartesianWaypoint> waypoints_;
  std::vector<double> current_joint_positions_;
  std::vector<geometry_msgs::msg::Point> executed_path_;
  
  // Trajectory state
  bool trajectory_active_;
  size_t current_waypoint_index_;
  rclcpp::Time trajectory_start_time_;
  
  // Functions
  void waypoint_file_callback(const std_msgs::msg::String::SharedPtr msg);
  void control_timer_callback();
  bool load_waypoints(const std::string& filename);
  std::vector<double> solve_ik_simple(double x, double y, double z);
  geometry_msgs::msg::Point forward_kinematics(const std::vector<double>& joints);
  void publish_waypoint_markers();
  void publish_path_marker();
};

} // namespace ur10e_lift_trajectory_control

#endif
