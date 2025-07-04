#ifndef UR10E_LIFT_TRAJECTORY_CONTROL__SPLINE_TRAJECTORY_PLANNER_HPP_
#define UR10E_LIFT_TRAJECTORY_CONTROL__SPLINE_TRAJECTORY_PLANNER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <vector>
#include <string>
#include <memory>
#include <cmath>

namespace ur10e_lift_trajectory_control
{

struct CartesianWaypoint
{
  double x, y, z;
  double time_from_start;
  std::string description;
};

struct SplineSegment
{
  double a, b, c, d;
  double t_start, t_end;
};

class SplineTrajectoryPlanner : public rclcpp::Node
{
public:
  SplineTrajectoryPlanner();

private:
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr waypoint_markers_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr path_marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr spline_marker_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr waypoint_file_sub_;
  rclcpp::TimerBase::SharedPtr control_timer_;
  
  std::vector<std::string> joint_names_;
  std::vector<CartesianWaypoint> waypoints_;
  std::vector<double> current_joint_positions_;
  std::vector<geometry_msgs::msg::Point> executed_path_;
  std::vector<SplineSegment> spline_x_, spline_y_, spline_z_;
  std::vector<geometry_msgs::msg::Point> spline_preview_points_;
  
  bool trajectory_active_;
  double trajectory_time_;
  rclcpp::Time trajectory_start_time_;
  double control_frequency_;
  
  void waypoint_file_callback(const std_msgs::msg::String::SharedPtr msg);
  void control_timer_callback();
  bool load_waypoints(const std::string& filename);
  void generate_cubic_splines();
  std::vector<SplineSegment> create_cubic_spline(const std::vector<double>& times, const std::vector<double>& values);
  geometry_msgs::msg::Point evaluate_spline_at_time(double t);
  geometry_msgs::msg::Point evaluate_spline_velocity_at_time(double t);
  void generate_spline_preview();
  std::vector<double> solve_ik_advanced(const geometry_msgs::msg::Point& target_pos, const geometry_msgs::msg::Point& target_vel);
  geometry_msgs::msg::Point forward_kinematics(const std::vector<double>& joints);
  void publish_waypoint_markers();
  void publish_path_marker();
  void publish_spline_preview();
  double clamp(double value, double min_val, double max_val);
  void log_trajectory_info();
};

} // namespace ur10e_lift_trajectory_control

#endif
