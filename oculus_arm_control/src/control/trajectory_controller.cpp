#include "oculus_arm_control/control/trajectory_controller.h"
#include <fstream>
#include <yaml-cpp/yaml.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace oculus_arm_control {
namespace control {

TrajectoryController::TrajectoryController(ros::NodeHandle& nh)
    : nh_(nh),
      trajectory_type_(TrajectoryType::LINEAR),
      execution_state_(TrajectoryExecutionState::IDLE),
      max_velocity_(params::DEFAULT_MAX_VELOCITY),
      max_acceleration_(params::DEFAULT_MAX_ACCELERATION),
      max_jerk_(params::DEFAULT_MAX_JERK),
      time_between_waypoints_(1.0),
      current_waypoint_index_(0),
      trajectory_time_(0.0),
      trajectory_duration_(0.0),
      is_recording_(false),
      recording_interval_(0.1) {
    
    // Initialize publishers
    pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(topics::ARM_TARGET_POSE, 10);
    path_pub_ = nh_.advertise<nav_msgs::Path>(topics::VIZ_TRAJECTORY, 10);
    state_pub_ = nh_.advertise<std_msgs::String>("/trajectory_controller/state", 10);
    
    ROS_INFO("Trajectory controller initialized");
}

void TrajectoryController::addWaypoint(const geometry_msgs::Pose& waypoint, bool is_stop_point) {
    std::lock_guard<std::mutex> lock(waypoints_mutex_);
    
    TrajectoryWaypoint trajectory_waypoint;
    trajectory_waypoint.pose = waypoint;
    trajectory_waypoint.is_stop_point = is_stop_point;
    trajectory_waypoint.time = -1.0;  // Will be computed later
    
    waypoints_.push_back(trajectory_waypoint);
    
    ROS_INFO("Added waypoint %ld to trajectory", waypoints_.size());
    
    // Update trajectory timing
    computeWaypointTiming();
    
    // Generate and publish the trajectory path for visualization
    generateTrajectoryPath();
    publishTrajectoryPath();
}

void TrajectoryController::addWaypoints(const std::vector<geometry_msgs::Pose>& waypoints) {
    std::lock_guard<std::mutex> lock(waypoints_mutex_);
    
    for (const auto& waypoint : waypoints) {
        TrajectoryWaypoint trajectory_waypoint;
        trajectory_waypoint.pose = waypoint;
        trajectory_waypoint.is_stop_point = false;
        trajectory_waypoint.time = -1.0;  // Will be computed later
        
        waypoints_.push_back(trajectory_waypoint);
    }
    
    ROS_INFO("Added %ld waypoints to trajectory", waypoints.size());
    
    // Update trajectory timing
    computeWaypointTiming();
    
    // Generate and publish the trajectory path for visualization
    generateTrajectoryPath();
    publishTrajectoryPath();
}

void TrajectoryController::clearWaypoints() {
    std::lock_guard<std::mutex> lock(waypoints_mutex_);
    waypoints_.clear();
    
    ROS_INFO("Cleared all waypoints from trajectory");
    
    // Publish empty path
    nav_msgs::Path empty_path;
    empty_path.header.frame_id = frames::WORLD_FRAME;
    empty_path.header.stamp = ros::Time::now();
    path_pub_.publish(empty_path);
}

bool TrajectoryController::startExecution() {
    std::lock_guard<std::mutex> lock(waypoints_mutex_);
    
    if (waypoints_.empty()) {
        ROS_WARN("Cannot start trajectory execution: no waypoints defined");
        return false;
    }
    
    if (execution_state_ == TrajectoryExecutionState::EXECUTING) {
        ROS_WARN("Trajectory is already being executed");
        return false;
    }
    
    // Reset execution state
    current_waypoint_index_ = 0;
    trajectory_time_ = 0.0;
    trajectory_duration_ = calculateTrajectoryDuration();
    last_update_time_ = ros::Time::now();
    execution_state_ = TrajectoryExecutionState::EXECUTING;
    
    ROS_INFO("Started trajectory execution with %ld waypoints, duration: %.2f seconds",
            waypoints_.size(), trajectory_duration_);
    
    publishExecutionState();
    return true;
}

void TrajectoryController::pauseExecution() {
    std::lock_guard<std::mutex> lock(waypoints_mutex_);
    
    if (execution_state_ == TrajectoryExecutionState::EXECUTING) {
        execution_state_ = TrajectoryExecutionState::PAUSED;
        ROS_INFO("Trajectory execution paused");
        publishExecutionState();
    }
}

void TrajectoryController::resumeExecution() {
    std::lock_guard<std::mutex> lock(waypoints_mutex_);
    
    if (execution_state_ == TrajectoryExecutionState::PAUSED) {
        execution_state_ = TrajectoryExecutionState::EXECUTING;
        last_update_time_ = ros::Time::now();
        ROS_INFO("Trajectory execution resumed");
        publishExecutionState();
    }
}

void TrajectoryController::stopExecution() {
    std::lock_guard<std::mutex> lock(waypoints_mutex_);
    
    if (execution_state_ == TrajectoryExecutionState::EXECUTING || 
        execution_state_ == TrajectoryExecutionState::PAUSED) {
        execution_state_ = TrajectoryExecutionState::IDLE;
        ROS_INFO("Trajectory execution stopped");
        publishExecutionState();
    }
}

bool TrajectoryController::isExecuting() const {
    return execution_state_ == TrajectoryExecutionState::EXECUTING ||
           execution_state_ == TrajectoryExecutionState::PAUSED;
}

void TrajectoryController::update(const ros::Time& time) {
    std::lock_guard<std::mutex> lock(waypoints_mutex_);
    
    // Update the trajectory recording if active
    if (is_recording_ && (time - last_record_time_).toSec() >= recording_interval_) {
        last_record_time_ = time;
        // recordCurrentPose() will be called from outside with the current pose
    }
    
    // If not executing, nothing to do
    if (execution_state_ != TrajectoryExecutionState::EXECUTING) {
        return;
    }
    
    // Check if we have any waypoints
    if (waypoints_.empty()) {
        execution_state_ = TrajectoryExecutionState::IDLE;
        publishExecutionState();
        return;
    }
    
    // Calculate time elapsed since last update
    double dt = (time - last_update_time_).toSec();
    last_update_time_ = time;
    
    // Update trajectory time
    trajectory_time_ += dt;
    
    // Check if we've completed the trajectory
    if (trajectory_time_ >= trajectory_duration_) {
        // Set the final pose to the last waypoint
        geometry_msgs::PoseStamped pose_msg;
        pose_msg.header.frame_id = frames::WORLD_FRAME;
        pose_msg.header.stamp = time;
        pose_msg.pose = waypoints_.back().pose;
        pose_pub_.publish(pose_msg);
        
        execution_state_ = TrajectoryExecutionState::COMPLETED;
        publishExecutionState();
        ROS_INFO("Trajectory execution completed");
        return;
    }
    
    // Find the current segment in the trajectory
    double accumulated_time = 0.0;
    size_t from_index = 0;
    size_t to_index = 0;
    double segment_time_fraction = 0.0;
    
    for (size_t i = 0; i < waypoints_.size(); ++i) {
        if (i > 0) {
            double segment_duration = waypoints_[i].time - waypoints_[i-1].time;
            
            if (accumulated_time + segment_duration >= trajectory_time_) {
                from_index = i - 1;
                to_index = i;
                segment_time_fraction = (trajectory_time_ - accumulated_time) / segment_duration;
                break;
            }
            
            accumulated_time += segment_duration;
        }
    }
    
    // Interpolate between waypoints
    geometry_msgs::Pose interpolated_pose = interpolateWaypoints(from_index, to_index, segment_time_fraction);
    
    // Publish the interpolated pose
    geometry_msgs::PoseStamped pose_msg;
    pose_msg.header.frame_id = frames::WORLD_FRAME;
    pose_msg.header.stamp = time;
    pose_msg.pose = interpolated_pose;
    pose_pub_.publish(pose_msg);
    
    // Update current waypoint index for progress reporting
    current_waypoint_index_ = from_index;
}

void TrajectoryController::setMaxVelocity(double max_velocity) {
    max_velocity_ = max_velocity;
    
    // Update trajectory timing
    computeWaypointTiming();
    
    ROS_INFO("Set maximum velocity to %.2f", max_velocity);
}

void TrajectoryController::setMaxAcceleration(double max_acceleration) {
    max_acceleration_ = max_acceleration;
    
    // Update trajectory timing
    computeWaypointTiming();
    
    ROS_INFO("Set maximum acceleration to %.2f", max_acceleration);
}

void TrajectoryController::setMaxJerk(double max_jerk) {
    max_jerk_ = max_jerk;
    ROS_INFO("Set maximum jerk to %.2f", max_jerk);
}

void TrajectoryController::setTrajectoryType(TrajectoryType type) {
    trajectory_type_ = type;
    
    // Generate and publish the updated trajectory path
    generateTrajectoryPath();
    publishTrajectoryPath();
    
    std::string type_name;
    switch (type) {
        case TrajectoryType::LINEAR: type_name = "LINEAR"; break;
        case TrajectoryType::SPLINE: type_name = "SPLINE"; break;
        case TrajectoryType::TRAPEZOIDAL: type_name = "TRAPEZOIDAL"; break;
        case TrajectoryType::MINIMUM_JERK: type_name = "MINIMUM_JERK"; break;
    }
    
    ROS_INFO("Set trajectory type to %s", type_name.c_str());
}

void TrajectoryController::setWaypointTiming(double time_between_waypoints) {
    time_between_waypoints_ = time_between_waypoints;
    
    // Update trajectory timing
    computeWaypointTiming();
    
    ROS_INFO("Set time between waypoints to %.2f seconds", time_between_waypoints);
}

bool TrajectoryController::saveTrajectory(const std::string& filename) {
    std::lock_guard<std::mutex> lock(waypoints_mutex_);
    
    try {
        YAML::Emitter out;
        out << YAML::BeginMap;
        
        // Save trajectory metadata
        out << YAML::Key << "type" << YAML::Value << static_cast<int>(trajectory_type_);
        out << YAML::Key << "max_velocity" << YAML::Value << max_velocity_;
        out << YAML::Key << "max_acceleration" << YAML::Value << max_acceleration_;
        out << YAML::Key << "max_jerk" << YAML::Value << max_jerk_;
        
        // Save waypoints
        out << YAML::Key << "waypoints" << YAML::Value << YAML::BeginSeq;
        
        for (const auto& waypoint : waypoints_) {
            out << YAML::BeginMap;
            
            // Position
            out << YAML::Key << "position" << YAML::Value << YAML::BeginMap;
            out << YAML::Key << "x" << YAML::Value << waypoint.pose.position.x;
            out << YAML::Key << "y" << YAML::Value << waypoint.pose.position.y;
            out << YAML::Key << "z" << YAML::Value << waypoint.pose.position.z;
            out << YAML::EndMap;
            
            // Orientation
            out << YAML::Key << "orientation" << YAML::Value << YAML::BeginMap;
            out << YAML::Key << "x" << YAML::Value << waypoint.pose.orientation.x;
            out << YAML::Key << "y" << YAML::Value << waypoint.pose.orientation.y;
            out << YAML::Key << "z" << YAML::Value << waypoint.pose.orientation.z;
            out << YAML::Key << "w" << YAML::Value << waypoint.pose.orientation.w;
            out << YAML::EndMap;
            
            // Other properties
            out << YAML::Key << "is_stop_point" << YAML::Value << waypoint.is_stop_point;
            out << YAML::Key << "time" << YAML::Value << waypoint.time;
            
            out << YAML::EndMap;
        }
        
        out << YAML::EndSeq;
        out << YAML::EndMap;
        
        // Write to file
        std::ofstream fout(filename);
        if (!fout.is_open()) {
            ROS_ERROR("Failed to open file for writing: %s", filename.c_str());
            return false;
        }
        
        fout << out.c_str();
        fout.close();
        
        ROS_INFO("Saved trajectory with %ld waypoints to %s", waypoints_.size(), filename.c_str());
        return true;
        
    } catch (const std::exception& e) {
        ROS_ERROR("Error saving trajectory: %s", e.what());
        return false;
    }
}

bool TrajectoryController::loadTrajectory(const std::string& filename) {
    try {
        YAML::Node config = YAML::LoadFile(filename);
        
        // Clear existing waypoints
        clearWaypoints();
        
        std::lock_guard<std::mutex> lock(waypoints_mutex_);
        
        // Load trajectory metadata
        if (config["type"]) {
            trajectory_type_ = static_cast<TrajectoryType>(config["type"].as<int>());
        }
        
        if (config["max_velocity"]) {
            max_velocity_ = config["max_velocity"].as<double>();
        }
        
        if (config["max_acceleration"]) {
            max_acceleration_ = config["max_acceleration"].as<double>();
        }
        
        if (config["max_jerk"]) {
            max_jerk_ = config["max_jerk"].as<double>();
        }
        
        // Load waypoints
        if (config["waypoints"]) {
            const YAML::Node& waypoints = config["waypoints"];
            
            for (size_t i = 0; i < waypoints.size(); ++i) {
                const YAML::Node& wp = waypoints[i];
                
                TrajectoryWaypoint waypoint;
                
                // Position
                if (wp["position"]) {
                    const YAML::Node& pos = wp["position"];
                    waypoint.pose.position.x = pos["x"].as<double>();
                    waypoint.pose.position.y = pos["y"].as<double>();
                    waypoint.pose.position.z = pos["z"].as<double>();
                }
                
                // Orientation
                if (wp["orientation"]) {
                    const YAML::Node& orient = wp["orientation"];
                    waypoint.pose.orientation.x = orient["x"].as<double>();
                    waypoint.pose.orientation.y = orient["y"].as<double>();
                    waypoint.pose.orientation.z = orient["z"].as<double>();
                    waypoint.pose.orientation.w = orient["w"].as<double>();
                }
                
                // Other properties
                if (wp["is_stop_point"]) {
                    waypoint.is_stop_point = wp["is_stop_point"].as<bool>();
                }
                
                if (wp["time"]) {
                    waypoint.time = wp["time"].as<double>();
                }
                
                waypoints_.push_back(waypoint);
            }
        }
        
        // Update trajectory timing if no timing was loaded
        bool need_timing_update = false;
        for (const auto& waypoint : waypoints_) {
            if (waypoint.time < 0) {
                need_timing_update = true;
                break;
            }
        }
        
        if (need_timing_update) {
            computeWaypointTiming();
        }
        
        // Generate and publish the trajectory path
        generateTrajectoryPath();
        publishTrajectoryPath();
        
        ROS_INFO("Loaded trajectory with %ld waypoints from %s", waypoints_.size(), filename.c_str());
        return true;
        
    } catch (const std::exception& e) {
        ROS_ERROR("Error loading trajectory: %s", e.what());
        return false;
    }
}

void TrajectoryController::recordCurrentPose(const geometry_msgs::Pose& current_pose) {
    if (!is_recording_) {
        return;
    }
    
    addWaypoint(current_pose, false);
}

void TrajectoryController::startRecording() {
    if (is_recording_) {
        ROS_WARN("Trajectory recording is already active");
        return;
    }
    
    // Clear existing waypoints if requested
    clearWaypoints();
    
    is_recording_ = true;
    last_record_time_ = ros::Time::now();
    
    ROS_INFO("Started trajectory recording");
}

void TrajectoryController::stopRecording() {
    if (!is_recording_) {
        return;
    }
    
    is_recording_ = false;
    
    // Update trajectory timing
    computeWaypointTiming();
    
    ROS_INFO("Stopped trajectory recording, captured %ld waypoints", waypoints_.size());
}

bool TrajectoryController::isRecording() const {
    return is_recording_;
}

std::vector<TrajectoryWaypoint> TrajectoryController::getWaypoints() const {
    std::lock_guard<std::mutex> lock(waypoints_mutex_);
    return waypoints_;
}

TrajectoryExecutionState TrajectoryController::getExecutionState() const {
    return execution_state_;
}

double TrajectoryController::getTrajectoryProgress() const {
    std::lock_guard<std::mutex> lock(waypoints_mutex_);
    
    if (waypoints_.empty() || trajectory_duration_ <= 0) {
        return 0.0;
    }
    
    return std::min(1.0, trajectory_time_ / trajectory_duration_);
}

geometry_msgs::Pose TrajectoryController::getCurrentCommandedPose() const {
    std::lock_guard<std::mutex> lock(waypoints_mutex_);
    
    if (waypoints_.empty()) {
        // Return identity pose if no waypoints
        geometry_msgs::Pose pose;
        pose.position.x = 0;
        pose.position.y = 0;
        pose.position.z = 0;
        pose.orientation.x = 0;
        pose.orientation.y = 0;
        pose.orientation.z = 0;
        pose.orientation.w = 1;
        return pose;
    }
    
    if (execution_state_ != TrajectoryExecutionState::EXECUTING || 
        trajectory_time_ >= trajectory_duration_) {
        return waypoints_.back().pose;
    }
    
    // Find the current segment
    double accumulated_time = 0.0;
    size_t from_index = 0;
    size_t to_index = 0;
    double segment_time_fraction = 0.0;
    
    for (size_t i = 0; i < waypoints_.size(); ++i) {
        if (i > 0) {
            double segment_duration = waypoints_[i].time - waypoints_[i-1].time;
            
            if (accumulated_time + segment_duration >= trajectory_time_) {
                from_index = i - 1;
                to_index = i;
                segment_time_fraction = (trajectory_time_ - accumulated_time) / segment_duration;
                break;
            }
            
            accumulated_time += segment_duration;
        }
    }
    
    // Interpolate between waypoints
    return interpolateWaypoints(from_index, to_index, segment_time_fraction);
}

nav_msgs::Path TrajectoryController::getTrajectoryPath() const {
    std::lock_guard<std::mutex> lock(waypoints_mutex_);
    
    nav_msgs::Path path;
    path.header.frame_id = frames::WORLD_FRAME;
    path.header.stamp = ros::Time::now();
    
    if (waypoints_.empty()) {
        return path;
    }
    
    // For a simple path, just add the waypoints
    for (const auto& waypoint : waypoints_) {
        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header = path.header;
        pose_stamped.pose = waypoint.pose;
        path.poses.push_back(pose_stamped);
    }
    
    return path;
}

void TrajectoryController::generateTrajectoryPath() {
    // This method will generate a more detailed path for visualization
    // For now, it's empty as the simple path is generated in getTrajectoryPath()
}

void TrajectoryController::publishTrajectoryPath() {
    nav_msgs::Path path = getTrajectoryPath();
    path_pub_.publish(path);
}

void TrajectoryController::publishExecutionState() {
    std_msgs::String state_msg;
    
    switch (execution_state_) {
        case TrajectoryExecutionState::IDLE:
            state_msg.data = "IDLE";
            break;
        case TrajectoryExecutionState::EXECUTING:
            state_msg.data = "EXECUTING";
            break;
        case TrajectoryExecutionState::PAUSED:
            state_msg.data = "PAUSED";
            break;
        case TrajectoryExecutionState::COMPLETED:
            state_msg.data = "COMPLETED";
            break;
        case TrajectoryExecutionState::ERROR:
            state_msg.data = "ERROR";
            break;
    }
    
    state_pub_.publish(state_msg);
}

geometry_msgs::Pose TrajectoryController::interpolateWaypoints(size_t from_index, size_t to_index, double t) {
    if (waypoints_.empty() || from_index >= waypoints_.size() || to_index >= waypoints_.size()) {
        // Return identity pose if indices are invalid
        geometry_msgs::Pose pose;
        pose.position.x = 0;
        pose.position.y = 0;
        pose.position.z = 0;
        pose.orientation.x = 0;
        pose.orientation.y = 0;
        pose.orientation.z = 0;
        pose.orientation.w = 1;
        return pose;
    }
    
    // Clamp t to [0,1]
    t = std::max(0.0, std::min(1.0, t));
    
    const geometry_msgs::Pose& start_pose = waypoints_[from_index].pose;
    const geometry_msgs::Pose& end_pose = waypoints_[to_index].pose;
    
    return math_utils::interpolatePoses(start_pose, end_pose, t);
}

double TrajectoryController::calculateTrajectoryDuration() {
    if (waypoints_.empty()) {
        return 0.0;
    }
    
    // The duration is the time of the last waypoint
    return waypoints_.back().time;
}

void TrajectoryController::computeWaypointTiming() {
    if (waypoints_.empty()) {
        return;
    }
    
    // Set time of first waypoint to 0
    waypoints_[0].time = 0.0;
    
    for (size_t i = 1; i < waypoints_.size(); ++i) {
        // Calculate distance to previous waypoint
        double distance = math_utils::positionDistance(waypoints_[i-1].pose, waypoints_[i].pose);
        
        // Calculate time required based on maximum velocity
        double time = distance / max_velocity_;
        
        // Ensure minimum time between waypoints
        time = std::max(time, time_between_waypoints_);
        
        // Set the time for this waypoint (cumulative)
        waypoints_[i].time = waypoints_[i-1].time + time;
    }
}

std::vector<geometry_msgs::Pose> TrajectoryController::generateLinearTrajectory(const geometry_msgs::Pose& start, const geometry_msgs::Pose& end, int steps) {
    std::vector<geometry_msgs::Pose> trajectory;
    
    for (int i = 0; i <= steps; ++i) {
        double t = static_cast<double>(i) / steps;
        trajectory.push_back(math_utils::interpolatePoses(start, end, t));
    }
    
    return trajectory;
}

std::vector<geometry_msgs::Pose> TrajectoryController::generateSplineTrajectory(const std::vector<geometry_msgs::Pose>& control_points, int steps_per_segment) {
    // For now, just return a linear trajectory between each control point
    std::vector<geometry_msgs::Pose> trajectory;
    
    if (control_points.size() < 2) {
        return trajectory;
    }
    
    for (size_t i = 0; i < control_points.size() - 1; ++i) {
        std::vector<geometry_msgs::Pose> segment = generateLinearTrajectory(
            control_points[i],
            control_points[i+1],
            steps_per_segment
        );
        
        // Add all points except the last one (to avoid duplicates)
        trajectory.insert(trajectory.end(), segment.begin(), 
                        i < control_points.size() - 2 ? segment.end() - 1 : segment.end());
    }
    
    return trajectory;
}

std::vector<geometry_msgs::Pose> TrajectoryController::generateMinimumJerkTrajectory(const geometry_msgs::Pose& start, const geometry_msgs::Pose& end, double duration, double time_step) {
    std::vector<geometry_msgs::Pose> trajectory;
    
    int steps = static_cast<int>(duration / time_step);
    
    for (int i = 0; i <= steps; ++i) {
        double t = static_cast<double>(i) / steps;
        
        // Apply minimum jerk formula for position (5th order polynomial)
        double s = 10 * std::pow(t, 3) - 15 * std::pow(t, 4) + 6 * std::pow(t, 5);
        
        geometry_msgs::Pose pose = math_utils::interpolatePoses(start, end, s);
        trajectory.push_back(pose);
    }
    
    return trajectory;
}

} // namespace control
} // namespace oculus_arm_control 