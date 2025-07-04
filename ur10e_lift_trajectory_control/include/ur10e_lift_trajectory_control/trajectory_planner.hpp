#ifndef UR10E_LIFT_TRAJECTORY_CONTROL__TRAJECTORY_PLANNER_HPP_
#define UR10E_LIFT_TRAJECTORY_CONTROL__TRAJECTORY_PLANNER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <vector>
#include <string>
#include <memory>
#include <fstream>

namespace ur10e_lift_trajectory_control
{

struct Waypoint
{
  std::vector<double> joint_positions;
  double time_from_start;
  std::string description;
};

class TrajectoryPlanner : public rclcpp::Node
{
public:
  TrajectoryPlanner();
  ~TrajectoryPlanner() = default;

private:
  // ROS Publishers and Subscribers
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr waypoint_file_sub_;
  
  // Timer for trajectory execution
  rclcpp::TimerBase::SharedPtr trajectory_timer_;
  
  // Joint names
  std::vector<std::string> joint_names_;
  
  // Current trajectory state
  std::vector<Waypoint> waypoints_;
  size_t current_waypoint_index_;
  bool trajectory_active_;
  rclcpp::Time trajectory_start_time_;
  
  // Current joint positions
  std::vector<double> current_joint_positions_;
  
  // Methods
  void waypoint_file_callback(const std_msgs::msg::String::SharedPtr msg);
  void trajectory_timer_callback();
  void publish_joint_states();
  
  bool load_waypoints_from_file(const std::string& filename);
  void start_trajectory_execution();
  void interpolate_to_waypoint(const Waypoint& target_waypoint, double progress);
  std::vector<double> cubic_interpolation(
    const std::vector<double>& start_pos,
    const std::vector<double>& end_pos,
    double t
  );
  
  void log_current_state();
};

} // namespace ur10e_lift_trajectory_control

#endif // UR10E_LIFT_TRAJECTORY_CONTROL__TRAJECTORY_PLANNER_HPP_