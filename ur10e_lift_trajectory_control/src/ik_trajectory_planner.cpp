#include "ur10e_lift_trajectory_control/ik_trajectory_planner.hpp"
#include <fstream>
#include <sstream>
#include <cmath>

namespace ur10e_lift_trajectory_control
{

IKTrajectoryPlanner::IKTrajectoryPlanner() : Node("ik_trajectory_planner")
{
  joint_names_ = {
    "lift_joint",
    "ur10e_shoulder_pan_joint", 
    "ur10e_shoulder_lift_joint",
    "ur10e_elbow_joint",
    "ur10e_wrist_1_joint",
    "ur10e_wrist_2_joint",
    "ur10e_wrist_3_joint"
  };
  
  current_joint_positions_.resize(joint_names_.size(), 0.0);
  trajectory_active_ = false;
  current_waypoint_index_ = 0;
  
  // Create publishers
  joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
  waypoint_markers_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/cartesian_waypoints", 10);
  path_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/end_effector_path", 10);
  
  // Create subscriber
  waypoint_file_sub_ = this->create_subscription<std_msgs::msg::String>(
    "/load_cartesian_waypoints", 10,
    std::bind(&IKTrajectoryPlanner::waypoint_file_callback, this, std::placeholders::_1));
  
  // Create timer
  control_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(100), // 10 Hz for smoother trajectory
    std::bind(&IKTrajectoryPlanner::control_timer_callback, this));
  
  RCLCPP_INFO(this->get_logger(), "IK Trajectory Planner with Sequential Waypoints started");
  RCLCPP_INFO(this->get_logger(), "Publishing waypoints to: /cartesian_waypoints");
  RCLCPP_INFO(this->get_logger(), "Publishing path to: /end_effector_path");
}

void IKTrajectoryPlanner::waypoint_file_callback(const std_msgs::msg::String::SharedPtr msg)
{
  RCLCPP_INFO(this->get_logger(), "Loading waypoints from: %s", msg->data.c_str());
  if (load_waypoints(msg->data)) {
    RCLCPP_INFO(this->get_logger(), "Loaded %zu waypoints", waypoints_.size());
    publish_waypoint_markers();
    executed_path_.clear(); // Clear previous path
    
    // Start trajectory execution
    trajectory_active_ = true;
    current_waypoint_index_ = 0;
    trajectory_start_time_ = this->now();
    
    RCLCPP_INFO(this->get_logger(), "Starting trajectory execution!");
  }
}

bool IKTrajectoryPlanner::load_waypoints(const std::string& filename)
{
  std::ifstream file(filename);
  if (!file.is_open()) {
    RCLCPP_ERROR(this->get_logger(), "Cannot open file: %s", filename.c_str());
    return false;
  }
  
  waypoints_.clear();
  std::string line;
  
  while (std::getline(file, line)) {
    if (line.empty() || line[0] == '#') continue;
    
    std::istringstream iss(line);
    CartesianWaypoint wp;
    
    if (iss >> wp.x >> wp.y >> wp.z >> wp.time_from_start) {
      std::getline(iss, wp.description);
      waypoints_.push_back(wp);
      RCLCPP_INFO(this->get_logger(), "Waypoint %zu: (%.2f, %.2f, %.2f) at t=%.2f %s", 
                  waypoints_.size(), wp.x, wp.y, wp.z, wp.time_from_start, wp.description.c_str());
    }
  }
  
  return !waypoints_.empty();
}

std::vector<double> IKTrajectoryPlanner::solve_ik_simple(double x, double y, double z)
{
  std::vector<double> joints(7, 0.0);
  
  // Improved geometric IK approximation
  joints[0] = std::max(0.0, std::min(1.0, z - 0.9)); // lift_joint (better offset)
  joints[1] = atan2(y, x); // base rotation
  
  double r = sqrt(x*x + y*y);
  double target_height = z - joints[0] - 0.18 - 0.163; // Remove base heights
  
  // Better arm positioning
  joints[2] = -atan2(target_height - 0.3, r - 0.2); // shoulder_lift
  joints[3] = 1.2; // elbow
  joints[4] = 0.0; // wrist joints
  joints[5] = 0.0;
  joints[6] = 0.0;
  
  return joints;
}

geometry_msgs::msg::Point IKTrajectoryPlanner::forward_kinematics(const std::vector<double>& joints)
{
  geometry_msgs::msg::Point point;
  
  if (joints.size() < 7) {
    point.x = point.y = point.z = 0.0;
    return point;
  }
  
  // Simplified forward kinematics
  double lift_height = joints[0];
  double base_rotation = joints[1];
  double shoulder_lift = joints[2];
  double elbow = joints[3];
  
  // Approximate arm reach calculation
  double upper_arm = 0.612;
  double forearm = 0.572;
  double arm_extension = upper_arm * cos(shoulder_lift) + forearm * cos(shoulder_lift + elbow);
  double arm_height = upper_arm * sin(shoulder_lift) + forearm * sin(shoulder_lift + elbow);
  
  point.x = arm_extension * cos(base_rotation);
  point.y = arm_extension * sin(base_rotation);
  point.z = lift_height + 0.18 + 0.163 + arm_height + 0.174; // Base heights + arm height
  
  return point;
}

void IKTrajectoryPlanner::control_timer_callback()
{
  if (trajectory_active_ && !waypoints_.empty()) {
    double elapsed_time = (this->now() - trajectory_start_time_).seconds();
    
    // Find which waypoint we should be targeting now
    size_t target_waypoint = 0;
    for (size_t i = 0; i < waypoints_.size(); ++i) {
      if (elapsed_time <= waypoints_[i].time_from_start) {
        target_waypoint = i;
        break;
      }
    }
    
    // If we've passed all waypoints, use the last one
    if (target_waypoint >= waypoints_.size()) {
      target_waypoint = waypoints_.size() - 1;
    }
    
    // Check if we've moved to a new waypoint
    if (target_waypoint != current_waypoint_index_) {
      current_waypoint_index_ = target_waypoint;
      RCLCPP_INFO(this->get_logger(), "Moving to waypoint %zu: (%.2f, %.2f, %.2f) - %s",
                  current_waypoint_index_,
                  waypoints_[current_waypoint_index_].x,
                  waypoints_[current_waypoint_index_].y,
                  waypoints_[current_waypoint_index_].z,
                  waypoints_[current_waypoint_index_].description.c_str());
    }
    
    // Solve IK for current target waypoint
    const auto& wp = waypoints_[current_waypoint_index_];
    current_joint_positions_ = solve_ik_simple(wp.x, wp.y, wp.z);
    
    // Calculate current end-effector position
    geometry_msgs::msg::Point current_ee = forward_kinematics(current_joint_positions_);
    
    // Add to path
    if (executed_path_.empty() || 
        fabs(executed_path_.back().x - current_ee.x) > 0.01 ||
        fabs(executed_path_.back().y - current_ee.y) > 0.01 ||
        fabs(executed_path_.back().z - current_ee.z) > 0.01) {
      executed_path_.push_back(current_ee);
    }
    
    // Check if trajectory is complete
    if (elapsed_time > waypoints_.back().time_from_start + 1.0) {
      RCLCPP_INFO(this->get_logger(), "Trajectory execution completed!");
      trajectory_active_ = false;
    }
  }
  
  // Always publish joint states
  auto msg = sensor_msgs::msg::JointState();
  msg.header.stamp = this->now();
  msg.name = joint_names_;
  msg.position = current_joint_positions_;
  msg.velocity.resize(joint_names_.size(), 0.0);
  msg.effort.resize(joint_names_.size(), 0.0);
  
  joint_state_pub_->publish(msg);
  
  // Publish visualizations
  publish_path_marker();
}

void IKTrajectoryPlanner::publish_waypoint_markers()
{
  auto marker_array = visualization_msgs::msg::MarkerArray();
  
  for (size_t i = 0; i < waypoints_.size(); ++i) {
    auto marker = visualization_msgs::msg::Marker();
    marker.header.frame_id = "world";
    marker.header.stamp = this->now();
    marker.ns = "cartesian_waypoints";
    marker.id = static_cast<int>(i);
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    
    marker.pose.position.x = waypoints_[i].x;
    marker.pose.position.y = waypoints_[i].y;
    marker.pose.position.z = waypoints_[i].z;
    marker.pose.orientation.w = 1.0;
    
    marker.scale.x = marker.scale.y = marker.scale.z = 0.08; // Larger spheres
    
    // Color based on sequence (blue -> green -> red)
    double ratio = static_cast<double>(i) / std::max(1.0, static_cast<double>(waypoints_.size() - 1));
    marker.color.r = ratio;
    marker.color.g = 1.0 - ratio;
    marker.color.b = 0.2;
    marker.color.a = 0.9;
    
    marker_array.markers.push_back(marker);
  }
  
  waypoint_markers_pub_->publish(marker_array);
  RCLCPP_INFO(this->get_logger(), "Published %zu waypoint markers", waypoints_.size());
}

void IKTrajectoryPlanner::publish_path_marker()
{
  if (executed_path_.empty()) return;
  
  auto path_marker = visualization_msgs::msg::Marker();
  path_marker.header.frame_id = "world";
  path_marker.header.stamp = this->now();
  path_marker.ns = "end_effector_path";
  path_marker.id = 0;
  path_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
  path_marker.action = visualization_msgs::msg::Marker::ADD;
  
  path_marker.scale.x = 0.02; // Thicker line
  path_marker.color.r = 1.0;  // Red path
  path_marker.color.g = 0.0;
  path_marker.color.b = 0.0;
  path_marker.color.a = 1.0;
  
  path_marker.points = executed_path_;
  
  path_marker_pub_->publish(path_marker);
}

} // namespace ur10e_lift_trajectory_control

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ur10e_lift_trajectory_control::IKTrajectoryPlanner>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
