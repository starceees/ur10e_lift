#include "ur10e_lift_trajectory_control/spline_trajectory_planner.hpp"
#include <fstream>
#include <sstream>
#include <algorithm>

namespace ur10e_lift_trajectory_control
{

SplineTrajectoryPlanner::SplineTrajectoryPlanner() : Node("spline_trajectory_planner")
{
  joint_names_ = {
    "lift_joint", "ur10e_shoulder_pan_joint", "ur10e_shoulder_lift_joint",
    "ur10e_elbow_joint", "ur10e_wrist_1_joint", "ur10e_wrist_2_joint", "ur10e_wrist_3_joint"
  };
  
  current_joint_positions_.resize(joint_names_.size(), 0.0);
  trajectory_active_ = false;
  trajectory_time_ = 0.0;
  control_frequency_ = 50.0;
  
  joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
  waypoint_markers_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/cartesian_waypoints", 10);
  path_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/end_effector_path", 10);
  spline_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/spline_path", 10);
  
  waypoint_file_sub_ = this->create_subscription<std_msgs::msg::String>(
    "/load_cartesian_waypoints", 10,
    std::bind(&SplineTrajectoryPlanner::waypoint_file_callback, this, std::placeholders::_1));
  
  control_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(static_cast<int>(1000.0 / control_frequency_)),
    std::bind(&SplineTrajectoryPlanner::control_timer_callback, this));
  
  RCLCPP_INFO(this->get_logger(), "Spline Trajectory Planner started");
}

void SplineTrajectoryPlanner::waypoint_file_callback(const std_msgs::msg::String::SharedPtr msg)
{
  RCLCPP_INFO(this->get_logger(), "Loading waypoints: %s", msg->data.c_str());
  if (load_waypoints(msg->data)) {
    generate_cubic_splines();
    generate_spline_preview();
    publish_waypoint_markers();
    publish_spline_preview();
    trajectory_active_ = true;
    trajectory_time_ = 0.0;
    trajectory_start_time_ = this->now();
    executed_path_.clear();
    RCLCPP_INFO(this->get_logger(), "Started spline trajectory with %zu waypoints", waypoints_.size());
  }
}

bool SplineTrajectoryPlanner::load_waypoints(const std::string& filename)
{
  std::ifstream file(filename);
  if (!file.is_open()) return false;
  
  waypoints_.clear();
  std::string line;
  while (std::getline(file, line)) {
    if (line.empty() || line[0] == '#') continue;
    std::istringstream iss(line);
    CartesianWaypoint wp;
    if (iss >> wp.x >> wp.y >> wp.z >> wp.time_from_start) {
      std::getline(iss, wp.description);
      waypoints_.push_back(wp);
    }
  }
  return !waypoints_.empty();
}

void SplineTrajectoryPlanner::generate_cubic_splines()
{
  if (waypoints_.size() < 2) return;
  
  std::vector<double> times, x_vals, y_vals, z_vals;
  for (const auto& wp : waypoints_) {
    times.push_back(wp.time_from_start);
    x_vals.push_back(wp.x);
    y_vals.push_back(wp.y);
    z_vals.push_back(wp.z);
  }
  
  spline_x_ = create_cubic_spline(times, x_vals);
  spline_y_ = create_cubic_spline(times, y_vals);
  spline_z_ = create_cubic_spline(times, z_vals);
}

std::vector<SplineSegment> SplineTrajectoryPlanner::create_cubic_spline(const std::vector<double>& times, const std::vector<double>& values)
{
  std::vector<SplineSegment> segments;
  size_t n = times.size() - 1;
  if (n < 1) return segments;
  

  for (size_t i = 0; i < n; ++i) {
    SplineSegment seg;
    double dt = times[i+1] - times[i];
    seg.d = values[i];
    seg.c = (values[i+1] - values[i]) / dt;
    seg.b = 0.0;
    seg.a = 0.0;
    seg.t_start = times[i];
    seg.t_end = times[i+1];
    segments.push_back(seg);
  }
  return segments;
}

geometry_msgs::msg::Point SplineTrajectoryPlanner::evaluate_spline_at_time(double t)
{
  geometry_msgs::msg::Point point;
  if (spline_x_.empty()) { point.x = point.y = point.z = 0.0; return point; }
  
  auto evaluate = [&](const std::vector<SplineSegment>& spline) -> double {
    for (const auto& seg : spline) {
      if (t >= seg.t_start && t <= seg.t_end) {
        double dt = t - seg.t_start;
        return seg.a*dt*dt*dt + seg.b*dt*dt + seg.c*dt + seg.d;
      }
    }
    return spline.back().d;
  };
  
  point.x = evaluate(spline_x_);
  point.y = evaluate(spline_y_);
  point.z = evaluate(spline_z_);
  return point;
}

geometry_msgs::msg::Point SplineTrajectoryPlanner::evaluate_spline_velocity_at_time(double t)
{
  geometry_msgs::msg::Point vel;
  vel.x = vel.y = vel.z = 0.0;
  return vel;
}

void SplineTrajectoryPlanner::generate_spline_preview()
{
  spline_preview_points_.clear();
  if (waypoints_.empty()) return;
  
  double total_time = waypoints_.back().time_from_start;
  for (double t = 0.0; t <= total_time; t += 0.1) {
    spline_preview_points_.push_back(evaluate_spline_at_time(t));
  }
}

void SplineTrajectoryPlanner::control_timer_callback()
{
  if (trajectory_active_ && !waypoints_.empty()) {
    trajectory_time_ = (this->now() - trajectory_start_time_).seconds();
    
    if (trajectory_time_ > waypoints_.back().time_from_start) {
      RCLCPP_INFO(this->get_logger(), "Spline trajectory completed!");
      trajectory_active_ = false;
      return;
    }
    
    geometry_msgs::msg::Point target_pos = evaluate_spline_at_time(trajectory_time_);
    geometry_msgs::msg::Point target_vel = evaluate_spline_velocity_at_time(trajectory_time_);
    current_joint_positions_ = solve_ik_advanced(target_pos, target_vel);
    
    geometry_msgs::msg::Point actual_ee = forward_kinematics(current_joint_positions_);
    if (executed_path_.empty() || 
        fabs(executed_path_.back().x - actual_ee.x) > 0.005 ||
        fabs(executed_path_.back().y - actual_ee.y) > 0.005 ||
        fabs(executed_path_.back().z - actual_ee.z) > 0.005) {
      executed_path_.push_back(actual_ee);
    }
  }
  
  auto msg = sensor_msgs::msg::JointState();
  msg.header.stamp = this->now();
  msg.name = joint_names_;
  msg.position = current_joint_positions_;
  msg.velocity.resize(joint_names_.size(), 0.0);
  msg.effort.resize(joint_names_.size(), 0.0);
  
  joint_state_pub_->publish(msg);
  publish_path_marker();
}

std::vector<double> SplineTrajectoryPlanner::solve_ik_advanced(const geometry_msgs::msg::Point& target_pos, const geometry_msgs::msg::Point& target_vel)
{
  std::vector<double> joints(7, 0.0);
  
  double x = target_pos.x, y = target_pos.y, z = target_pos.z;
  joints[0] = clamp(z - 0.9, 0.0, 1.0);
  joints[1] = atan2(y, x);
  
  double r = sqrt(x*x + y*y);
  joints[2] = -atan2(r - 0.3, z - 1.2);
  joints[3] = 1.2;
  
  return joints;
}

geometry_msgs::msg::Point SplineTrajectoryPlanner::forward_kinematics(const std::vector<double>& joints)
{
  geometry_msgs::msg::Point point;
  if (joints.size() < 7) { point.x = point.y = point.z = 0.0; return point; }
  
  double lift_height = joints[0];
  double base_rotation = joints[1];
  double shoulder_lift = joints[2];
  double elbow = joints[3];
  
  double arm_extension = 0.612 * cos(shoulder_lift) + 0.572 * cos(shoulder_lift + elbow);
  double arm_height = 0.612 * sin(shoulder_lift) + 0.572 * sin(shoulder_lift + elbow);
  
  point.x = arm_extension * cos(base_rotation);
  point.y = arm_extension * sin(base_rotation);
  point.z = lift_height + 0.18 + 0.163 + arm_height + 0.174;
  
  return point;
}

void SplineTrajectoryPlanner::publish_waypoint_markers()
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
    
    marker.scale.x = marker.scale.y = marker.scale.z = 0.06;
    
    double ratio = static_cast<double>(i) / std::max(1.0, static_cast<double>(waypoints_.size() - 1));
    marker.color.r = ratio;
    marker.color.g = 1.0 - ratio;
    marker.color.b = 0.3;
    marker.color.a = 0.9;
    
    marker_array.markers.push_back(marker);
  }
  waypoint_markers_pub_->publish(marker_array);
}

void SplineTrajectoryPlanner::publish_path_marker()
{
  if (executed_path_.empty()) return;
  
  auto path_marker = visualization_msgs::msg::Marker();
  path_marker.header.frame_id = "world";
  path_marker.header.stamp = this->now();
  path_marker.ns = "end_effector_path";
  path_marker.id = 0;
  path_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
  path_marker.action = visualization_msgs::msg::Marker::ADD;
  
  path_marker.scale.x = 0.015;
  path_marker.color.r = 1.0;
  path_marker.color.g = 0.0;
  path_marker.color.b = 0.0;
  path_marker.color.a = 1.0;
  
  path_marker.points = executed_path_;
  path_marker_pub_->publish(path_marker);
}

void SplineTrajectoryPlanner::publish_spline_preview()
{
  if (spline_preview_points_.empty()) return;
  
  auto spline_marker = visualization_msgs::msg::Marker();
  spline_marker.header.frame_id = "world";
  spline_marker.header.stamp = this->now();
  spline_marker.ns = "spline_path";
  spline_marker.id = 0;
  spline_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
  spline_marker.action = visualization_msgs::msg::Marker::ADD;
  
  spline_marker.scale.x = 0.01;
  spline_marker.color.r = 0.0;
  spline_marker.color.g = 0.0;
  spline_marker.color.b = 1.0;
  spline_marker.color.a = 0.7;
  
  spline_marker.points = spline_preview_points_;
  spline_marker_pub_->publish(spline_marker);
}

double SplineTrajectoryPlanner::clamp(double value, double min_val, double max_val)
{
  return std::max(min_val, std::min(value, max_val));
}

void SplineTrajectoryPlanner::log_trajectory_info()
{
  RCLCPP_INFO(this->get_logger(), "Spline trajectory info logged");
}

} // namespace ur10e_lift_trajectory_control

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ur10e_lift_trajectory_control::SplineTrajectoryPlanner>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
