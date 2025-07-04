#include "ur10e_lift_trajectory_control/trajectory_planner.hpp"
#include <iostream>
#include <sstream>
#include <cmath>

namespace ur10e_lift_trajectory_control
{

TrajectoryPlanner::TrajectoryPlanner()
: Node("trajectory_planner"),
  current_waypoint_index_(0),
  trajectory_active_(false)
{
  // Initialize joint names
  joint_names_ = {
    "lift_joint",
    "ur10e_shoulder_pan_joint",
    "ur10e_shoulder_lift_joint",
    "ur10e_elbow_joint",
    "ur10e_wrist_1_joint",
    "ur10e_wrist_2_joint",
    "ur10e_wrist_3_joint"
  };
  
  // Initialize current joint positions to zero
  current_joint_positions_.resize(joint_names_.size(), 0.0);
  
  // Create publishers
  joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
    "/joint_states", 10);
    
  trajectory_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
    "/planned_trajectory", 10);
  
  waypoint_file_sub_ = this->create_subscription<std_msgs::msg::String>(
    "/load_waypoint_file", 10,
    std::bind(&TrajectoryPlanner::waypoint_file_callback, this, std::placeholders::_1));
  
  trajectory_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(10), // 100Hz
    std::bind(&TrajectoryPlanner::trajectory_timer_callback, this));
  
  RCLCPP_INFO(this->get_logger(), "UR10e Lift Trajectory Planner initialized");
  RCLCPP_INFO(this->get_logger(), "Send waypoint file path to /load_waypoint_file topic to start");
  RCLCPP_INFO(this->get_logger(), "Publishing joint states at 100Hz to /joint_states");
}

void TrajectoryPlanner::waypoint_file_callback(const std_msgs::msg::String::SharedPtr msg)
{
  std::string filename = msg->data;
  RCLCPP_INFO(this->get_logger(), "Loading waypoints from: %s", filename.c_str());
  
  if (load_waypoints_from_file(filename)) {
    RCLCPP_INFO(this->get_logger(), "Successfully loaded %zu waypoints", waypoints_.size());
    start_trajectory_execution();
  } else {
    RCLCPP_ERROR(this->get_logger(), "Failed to load waypoints from: %s", filename.c_str());
  }
}

bool TrajectoryPlanner::load_waypoints_from_file(const std::string& filename)
{
  std::ifstream file(filename);
  if (!file.is_open()) {
    RCLCPP_ERROR(this->get_logger(), "Cannot open waypoint file: %s", filename.c_str());
    return false;
  }
  
  waypoints_.clear();
  std::string line;
  int line_number = 0;
  
  while (std::getline(file, line)) {
    line_number++;
    
    if (line.empty() || line[0] == '#') {
      continue;
    }
    
    std::istringstream iss(line);
    Waypoint waypoint;
    waypoint.joint_positions.resize(joint_names_.size());
    
    // Read joint positions
    for (size_t i = 0; i < joint_names_.size(); ++i) {
      if (!(iss >> waypoint.joint_positions[i])) {
        RCLCPP_ERROR(this->get_logger(), 
          "Error reading joint position %zu at line %d", i, line_number);
        return false;
      }
    }
    
    // Read time from start
    if (!(iss >> waypoint.time_from_start)) {
      RCLCPP_ERROR(this->get_logger(), 
        "Error reading time_from_start at line %d", line_number);
      return false;
    }
    
    // Read optional description
    std::string remaining;
    std::getline(iss, remaining);
    waypoint.description = remaining.empty() ? ("Waypoint " + std::to_string(line_number)) : remaining;
    
    waypoints_.push_back(waypoint);
    
    RCLCPP_INFO(this->get_logger(), 
      "Loaded waypoint %d: [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f] at t=%.2f - %s",
      line_number,
      waypoint.joint_positions[0], waypoint.joint_positions[1],
      waypoint.joint_positions[2], waypoint.joint_positions[3],
      waypoint.joint_positions[4], waypoint.joint_positions[5],
      waypoint.joint_positions[6],
      waypoint.time_from_start,
      waypoint.description.c_str());
  }
  
  file.close();
  return !waypoints_.empty();
}

void TrajectoryPlanner::start_trajectory_execution()
{
  if (waypoints_.empty()) {
    RCLCPP_WARN(this->get_logger(), "No waypoints to execute");
    return;
  }
  
  current_waypoint_index_ = 0;
  trajectory_active_ = true;
  trajectory_start_time_ = this->now();
  
  RCLCPP_INFO(this->get_logger(), "Starting trajectory execution with %zu waypoints", waypoints_.size());
}

void TrajectoryPlanner::trajectory_timer_callback()
{
  if (trajectory_active_ && !waypoints_.empty()) {
    double elapsed_time = (this->now() - trajectory_start_time_).seconds();
    
    // Find current target waypoint
    while (current_waypoint_index_ < waypoints_.size() && 
           elapsed_time > waypoints_[current_waypoint_index_].time_from_start) {
      RCLCPP_INFO(this->get_logger(), "Reached waypoint %zu: %s", 
        current_waypoint_index_, waypoints_[current_waypoint_index_].description.c_str());
      current_waypoint_index_++;
    }
    
    if (current_waypoint_index_ >= waypoints_.size()) {
      // Trajectory completed
      RCLCPP_INFO(this->get_logger(), "Trajectory execution completed!");
      trajectory_active_ = false;
      current_joint_positions_ = waypoints_.back().joint_positions;
    } else {
      // Interpolate to current waypoint
      const auto& target_waypoint = waypoints_[current_waypoint_index_];
      double progress = 0.0;
      
      if (current_waypoint_index_ > 0) {
        double segment_start = waypoints_[current_waypoint_index_ - 1].time_from_start;
        double segment_duration = target_waypoint.time_from_start - segment_start;
        progress = segment_duration > 0 ? (elapsed_time - segment_start) / segment_duration : 1.0;
      } else {
        // First waypoint
        double segment_duration = target_waypoint.time_from_start;
        progress = segment_duration > 0 ? elapsed_time / segment_duration : 1.0;
      }
      
      progress = std::clamp(progress, 0.0, 1.0);
      interpolate_to_waypoint(target_waypoint, progress);
    }
  }
  
  // Always publish current joint states
  publish_joint_states();
}

void TrajectoryPlanner::interpolate_to_waypoint(const Waypoint& target_waypoint, double progress)
{
  std::vector<double> start_positions;
  
  if (current_waypoint_index_ == 0) {
    start_positions = std::vector<double>(joint_names_.size(), 0.0);
  } else {
    start_positions = waypoints_[current_waypoint_index_ - 1].joint_positions;
  }
  
  current_joint_positions_ = cubic_interpolation(start_positions, target_waypoint.joint_positions, progress);
}

std::vector<double> TrajectoryPlanner::cubic_interpolation(
  const std::vector<double>& start_pos,
  const std::vector<double>& end_pos,
  double t)
{
  std::vector<double> result(start_pos.size());
  
  double t2 = t * t;
  double t3 = t2 * t;
  double cubic_t = 3 * t2 - 2 * t3; // Smooth S-curve
  
  for (size_t i = 0; i < start_pos.size(); ++i) {
    result[i] = start_pos[i] + (end_pos[i] - start_pos[i]) * cubic_t;
  }
  
  return result;
}

void TrajectoryPlanner::publish_joint_states()
{
  auto joint_state_msg = sensor_msgs::msg::JointState();
  joint_state_msg.header.stamp = this->now();
  joint_state_msg.header.frame_id = "";
  joint_state_msg.name = joint_names_;
  joint_state_msg.position = current_joint_positions_;
  joint_state_msg.velocity.resize(joint_names_.size(), 0.0);
  joint_state_msg.effort.resize(joint_names_.size(), 0.0);
  
  joint_state_pub_->publish(joint_state_msg);
}

void TrajectoryPlanner::log_current_state()
{
  RCLCPP_INFO(this->get_logger(), 
    "Current positions: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f]",
    current_joint_positions_[0], current_joint_positions_[1],
    current_joint_positions_[2], current_joint_positions_[3],
    current_joint_positions_[4], current_joint_positions_[5],
    current_joint_positions_[6]);
}

} 

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ur10e_lift_trajectory_control::TrajectoryPlanner>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}