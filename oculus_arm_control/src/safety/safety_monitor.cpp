#include "oculus_arm_control/safety/safety_monitor.h"
#include "oculus_arm_control/utils/math_utils.h"

namespace oculus_arm_control {
namespace safety {

SafetyMonitor::SafetyMonitor(ros::NodeHandle& nh)
    : nh_(nh), 
      is_emergency_stop_active_(false),
      is_enabled_(true),
      max_velocity_(params::DEFAULT_MAX_VELOCITY),
      safety_level_(safety::MEDIUM) {
    
    // Initialize publishers
    safety_pub_ = nh_.advertise<std_msgs::String>(topics::SAFETY_COLLISION, 10);
    emergency_stop_pub_ = nh_.advertise<std_msgs::Bool>(topics::SAFETY_EMERGENCY_STOP, 10);
    
    // Initialize subscribers
    joint_state_sub_ = nh_.subscribe(topics::ROBOT_JOINT_STATES, 10, &SafetyMonitor::jointStateCallback, this);
    robot_pose_sub_ = nh_.subscribe(topics::ROBOT_CARTESIAN_POSE_COMMAND, 10, &SafetyMonitor::robotPoseCallback, this);
    robot_velocity_sub_ = nh_.subscribe("/robot/velocity", 10, &SafetyMonitor::robotVelocityCallback, this);
    collision_sub_ = nh_.subscribe(topics::SAFETY_COLLISION, 10, &SafetyMonitor::collisionCallback, this);
    
    // Set default workspace limits
    workspace_limits_ = params::DEFAULT_WORKSPACE_LIMITS;
    
    // Initialize current pose
    current_pose_.position.x = 0;
    current_pose_.position.y = 0;
    current_pose_.position.z = 0;
    current_pose_.orientation.x = 0;
    current_pose_.orientation.y = 0;
    current_pose_.orientation.z = 0;
    current_pose_.orientation.w = 1;
    
    ROS_INFO("Safety monitor initialized");
}

void SafetyMonitor::setWorkspaceLimits(const std::vector<double>& limits) {
    if (limits.size() >= 6) {
        workspace_limits_ = limits;
        ROS_INFO("Workspace limits set to [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f]",
                limits[0], limits[1], limits[2], limits[3], limits[4], limits[5]);
    } else {
        ROS_WARN("Invalid workspace limits provided. Expected at least 6 values.");
    }
}

void SafetyMonitor::setJointLimits(const std::vector<double>& min_limits, 
                                 const std::vector<double>& max_limits) {
    if (min_limits.size() == max_limits.size()) {
        joint_min_limits_ = min_limits;
        joint_max_limits_ = max_limits;
        ROS_INFO("Joint limits set for %ld joints", min_limits.size());
    } else {
        ROS_WARN("Joint limit vectors must have the same size");
    }
}

void SafetyMonitor::setVelocityLimit(double max_velocity) {
    max_velocity_ = max_velocity;
    ROS_INFO("Max velocity set to %.2f", max_velocity);
}

void SafetyMonitor::setVelocityLimits(const std::vector<double>& max_velocities) {
    max_velocities_ = max_velocities;
    ROS_INFO("Per-axis velocity limits set");
}

void SafetyMonitor::setSafetyLevel(safety::SafetyLevel level) {
    safety_level_ = level;
    
    // Update velocity limit based on safety level
    if (VELOCITY_LIMITS.find(level) != VELOCITY_LIMITS.end()) {
        max_velocity_ = VELOCITY_LIMITS.at(level);
        ROS_INFO("Safety level set to %d, max velocity updated to %.2f", 
                static_cast<int>(level), max_velocity_);
    }
}

void SafetyMonitor::setObstacles(const std::vector<Eigen::Vector3d>& obstacle_positions,
                              const std::vector<double>& obstacle_radii) {
    if (obstacle_positions.size() == obstacle_radii.size()) {
        obstacle_positions_ = obstacle_positions;
        obstacle_radii_ = obstacle_radii;
        ROS_INFO("Set %ld obstacles for collision checking", obstacle_positions.size());
    } else {
        ROS_WARN("Obstacle positions and radii must have the same size");
    }
}

SafetyCheckResult SafetyMonitor::checkWorkspaceLimits(const geometry_msgs::Pose& pose) {
    SafetyCheckResult result;
    result.is_safe = true;
    result.error_code = errors::NO_ERROR;
    result.message = errors::ERROR_MESSAGES.at(errors::NO_ERROR);
    result.error_pose = pose;
    
    if (!isEnabled()) {
        return result;
    }
    
    Eigen::Vector3d position = math_utils::positionFromPose(pose);
    
    if (!isPointInWorkspace(position)) {
        result.is_safe = false;
        result.error_code = errors::WORKSPACE_LIMIT_ERROR;
        result.message = errors::ERROR_MESSAGES.at(errors::WORKSPACE_LIMIT_ERROR);
        
        // Calculate closest point within workspace
        double x = math_utils::clamp(position.x(), workspace_limits_[0], workspace_limits_[1]);
        double y = math_utils::clamp(position.y(), workspace_limits_[2], workspace_limits_[3]);
        double z = math_utils::clamp(position.z(), workspace_limits_[4], workspace_limits_[5]);
        
        result.error_pose.position.x = position.x();
        result.error_pose.position.y = position.y();
        result.error_pose.position.z = position.z();
    }
    
    return result;
}

SafetyCheckResult SafetyMonitor::checkJointLimits(const std::vector<double>& joint_positions) {
    SafetyCheckResult result;
    result.is_safe = true;
    result.error_code = errors::NO_ERROR;
    result.message = errors::ERROR_MESSAGES.at(errors::NO_ERROR);
    
    if (!isEnabled() || joint_min_limits_.empty() || joint_max_limits_.empty()) {
        return result;
    }
    
    // Check if joint positions are within limits
    for (size_t i = 0; i < joint_positions.size() && i < joint_min_limits_.size(); ++i) {
        double pos = joint_positions[i];
        double min = joint_min_limits_[i];
        double max = joint_max_limits_[i];
        
        // Add safety margin
        min += params::JOINT_LIMIT_SAFETY_MARGIN;
        max -= params::JOINT_LIMIT_SAFETY_MARGIN;
        
        if (pos < min || pos > max) {
            result.is_safe = false;
            result.error_code = errors::JOINT_LIMIT_ERROR;
            result.message = errors::ERROR_MESSAGES.at(errors::JOINT_LIMIT_ERROR) + 
                           " (Joint " + std::to_string(i) + ")";
            break;
        }
    }
    
    return result;
}

SafetyCheckResult SafetyMonitor::checkVelocityLimits(const geometry_msgs::Twist& velocity) {
    SafetyCheckResult result;
    result.is_safe = true;
    result.error_code = errors::NO_ERROR;
    result.message = errors::ERROR_MESSAGES.at(errors::NO_ERROR);
    
    if (!isEnabled()) {
        return result;
    }
    
    // Check linear velocity magnitude
    double linear_vel_magnitude = std::sqrt(
        velocity.linear.x * velocity.linear.x +
        velocity.linear.y * velocity.linear.y +
        velocity.linear.z * velocity.linear.z
    );
    
    if (linear_vel_magnitude > max_velocity_) {
        result.is_safe = false;
        result.error_code = errors::VELOCITY_LIMIT_ERROR;
        result.message = errors::ERROR_MESSAGES.at(errors::VELOCITY_LIMIT_ERROR) + 
                      " (Linear: " + std::to_string(linear_vel_magnitude) + 
                      " > " + std::to_string(max_velocity_) + ")";
        return result;
    }
    
    // Check per-axis velocity limits if specified
    if (!max_velocities_.empty()) {
        if (max_velocities_.size() >= 3) {
            if (std::abs(velocity.linear.x) > max_velocities_[0] ||
                std::abs(velocity.linear.y) > max_velocities_[1] ||
                std::abs(velocity.linear.z) > max_velocities_[2]) {
                
                result.is_safe = false;
                result.error_code = errors::VELOCITY_LIMIT_ERROR;
                result.message = errors::ERROR_MESSAGES.at(errors::VELOCITY_LIMIT_ERROR) + 
                             " (Per-axis limit exceeded)";
                return result;
            }
        }
        
        // Check angular velocity limits if specified
        if (max_velocities_.size() >= 6) {
            double angular_vel_magnitude = std::sqrt(
                velocity.angular.x * velocity.angular.x +
                velocity.angular.y * velocity.angular.y +
                velocity.angular.z * velocity.angular.z
            );
            
            double max_angular_vel = *std::max_element(max_velocities_.begin() + 3, max_velocities_.begin() + 6);
            
            if (angular_vel_magnitude > max_angular_vel) {
                result.is_safe = false;
                result.error_code = errors::VELOCITY_LIMIT_ERROR;
                result.message = errors::ERROR_MESSAGES.at(errors::VELOCITY_LIMIT_ERROR) + 
                             " (Angular: " + std::to_string(angular_vel_magnitude) + 
                             " > " + std::to_string(max_angular_vel) + ")";
                return result;
            }
        }
    }
    
    return result;
}

SafetyCheckResult SafetyMonitor::checkCollisions(const geometry_msgs::Pose& pose) {
    SafetyCheckResult result;
    result.is_safe = true;
    result.error_code = errors::NO_ERROR;
    result.message = errors::ERROR_MESSAGES.at(errors::NO_ERROR);
    result.error_pose = pose;
    
    if (!isEnabled() || obstacle_positions_.empty()) {
        return result;
    }
    
    // Extract robot position
    Eigen::Vector3d robot_position(pose.position.x, pose.position.y, pose.position.z);
    
    // Check distance to each obstacle
    for (size_t i = 0; i < obstacle_positions_.size(); ++i) {
        double distance = (obstacle_positions_[i] - robot_position).norm();
        double min_safe_distance = obstacle_radii_[i] + MIN_OBSTACLE_DISTANCE;
        
        if (distance < min_safe_distance) {
            result.is_safe = false;
            result.error_code = errors::COLLISION_ERROR;
            result.message = errors::ERROR_MESSAGES.at(errors::COLLISION_ERROR) + 
                         " (Distance: " + std::to_string(distance) + 
                         " < " + std::to_string(min_safe_distance) + ")";
            result.error_pose = pose;
            break;
        }
    }
    
    return result;
}

SafetyCheckResult SafetyMonitor::checkMotionSafety(const geometry_msgs::Pose& current_pose,
                                                const geometry_msgs::Pose& target_pose) {
    SafetyCheckResult result;
    result.is_safe = true;
    result.error_code = errors::NO_ERROR;
    result.message = errors::ERROR_MESSAGES.at(errors::NO_ERROR);
    
    if (!isEnabled()) {
        return result;
    }
    
    // Check workspace limits for target pose
    result = checkWorkspaceLimits(target_pose);
    if (!result.is_safe) {
        return result;
    }
    
    // Check for collisions along the path
    geometry_msgs::Pose collision_pose;
    if (checkPathCollisions(current_pose, target_pose, collision_pose)) {
        result.is_safe = false;
        result.error_code = errors::COLLISION_ERROR;
        result.message = errors::ERROR_MESSAGES.at(errors::COLLISION_ERROR) + " (Path)";
        result.error_pose = collision_pose;
        return result;
    }
    
    // Calculate implied velocity for the motion
    double distance = math_utils::positionDistance(current_pose, target_pose);
    double motion_time = distance / max_velocity_;
    
    if (motion_time < 0.01) {
        // Motion time too small, could indicate a jump or teleport
        result.is_safe = false;
        result.error_code = errors::VELOCITY_LIMIT_ERROR;
        result.message = errors::ERROR_MESSAGES.at(errors::VELOCITY_LIMIT_ERROR) + 
                      " (Implied velocity too high)";
        result.error_pose = target_pose;
        return result;
    }
    
    return result;
}

SafetyCheckResult SafetyMonitor::checkCommandSafety(const geometry_msgs::Pose& current_pose,
                                                 const geometry_msgs::Pose& target_pose,
                                                 const geometry_msgs::Twist& velocity,
                                                 const std::vector<double>& joint_positions) {
    SafetyCheckResult result;
    result.is_safe = true;
    result.error_code = errors::NO_ERROR;
    result.message = errors::ERROR_MESSAGES.at(errors::NO_ERROR);
    
    if (!isEnabled()) {
        return result;
    }
    
    if (isEmergencyStopActive()) {
        result.is_safe = false;
        result.error_code = errors::HARDWARE_ERROR;
        result.message = "Emergency stop is active";
        return result;
    }
    
    // Check workspace limits
    result = checkWorkspaceLimits(target_pose);
    if (!result.is_safe) {
        return result;
    }
    
    // Check velocity limits
    result = checkVelocityLimits(velocity);
    if (!result.is_safe) {
        return result;
    }
    
    // Check joint limits
    result = checkJointLimits(joint_positions);
    if (!result.is_safe) {
        return result;
    }
    
    // Check collisions
    result = checkCollisions(target_pose);
    if (!result.is_safe) {
        return result;
    }
    
    // Check motion safety
    result = checkMotionSafety(current_pose, target_pose);
    if (!result.is_safe) {
        return result;
    }
    
    return result;
}

void SafetyMonitor::triggerEmergencyStop() {
    is_emergency_stop_active_ = true;
    
    std_msgs::Bool estop_msg;
    estop_msg.data = true;
    emergency_stop_pub_.publish(estop_msg);
    
    ROS_ERROR("EMERGENCY STOP TRIGGERED");
}

void SafetyMonitor::resetEmergencyStop() {
    is_emergency_stop_active_ = false;
    
    std_msgs::Bool estop_msg;
    estop_msg.data = false;
    emergency_stop_pub_.publish(estop_msg);
    
    ROS_INFO("Emergency stop reset");
}

bool SafetyMonitor::isEmergencyStopActive() const {
    return is_emergency_stop_active_;
}

void SafetyMonitor::jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg) {
    if (!isEnabled()) {
        return;
    }
    
    current_joint_positions_ = msg->position;
    
    // Check joint limits on new joint state
    SafetyCheckResult result = checkJointLimits(current_joint_positions_);
    if (!result.is_safe) {
        publishSafetyStatus(result);
        
        if (safety_level_ >= MEDIUM) {
            triggerEmergencyStop();
        }
    }
}

void SafetyMonitor::robotPoseCallback(const geometry_msgs::Pose::ConstPtr& msg) {
    if (!isEnabled()) {
        return;
    }
    
    geometry_msgs::Pose new_pose = *msg;
    
    // Check motion safety between old and new pose
    SafetyCheckResult motion_result = checkMotionSafety(current_pose_, new_pose);
    if (!motion_result.is_safe) {
        publishSafetyStatus(motion_result);
        
        if (safety_level_ >= MEDIUM) {
            triggerEmergencyStop();
        }
        
        return;
    }
    
    // Update current pose if safe
    current_pose_ = new_pose;
    
    // Check workspace limits and collisions on new pose
    SafetyCheckResult workspace_result = checkWorkspaceLimits(current_pose_);
    if (!workspace_result.is_safe) {
        publishSafetyStatus(workspace_result);
        
        if (safety_level_ >= MEDIUM) {
            triggerEmergencyStop();
        }
        
        return;
    }
    
    SafetyCheckResult collision_result = checkCollisions(current_pose_);
    if (!collision_result.is_safe) {
        publishSafetyStatus(collision_result);
        
        if (safety_level_ >= MEDIUM) {
            triggerEmergencyStop();
        }
    }
}

void SafetyMonitor::robotVelocityCallback(const geometry_msgs::Twist::ConstPtr& msg) {
    if (!isEnabled()) {
        return;
    }
    
    current_velocity_ = *msg;
    
    // Check velocity limits
    SafetyCheckResult result = checkVelocityLimits(current_velocity_);
    if (!result.is_safe) {
        publishSafetyStatus(result);
        
        if (safety_level_ >= MEDIUM) {
            triggerEmergencyStop();
        }
    }
}

void SafetyMonitor::collisionCallback(const std_msgs::Bool::ConstPtr& msg) {
    if (!isEnabled()) {
        return;
    }
    
    if (msg->data) {
        SafetyCheckResult result;
        result.is_safe = false;
        result.error_code = errors::COLLISION_ERROR;
        result.message = errors::ERROR_MESSAGES.at(errors::COLLISION_ERROR) + " (External detection)";
        result.error_pose = current_pose_;
        
        publishSafetyStatus(result);
        
        if (safety_level_ >= LOW) {
            triggerEmergencyStop();
        }
    }
}

void SafetyMonitor::enable() {
    is_enabled_ = true;
    ROS_INFO("Safety monitor enabled");
}

void SafetyMonitor::disable() {
    is_enabled_ = false;
    ROS_WARN("Safety monitor disabled");
}

bool SafetyMonitor::isEnabled() const {
    return is_enabled_;
}

bool SafetyMonitor::isPointInWorkspace(const Eigen::Vector3d& point) const {
    if (workspace_limits_.size() < 6) {
        return true; // No limits defined
    }
    
    return (point.x() >= workspace_limits_[0] && point.x() <= workspace_limits_[1] &&
            point.y() >= workspace_limits_[2] && point.y() <= workspace_limits_[3] &&
            point.z() >= workspace_limits_[4] && point.z() <= workspace_limits_[5]);
}

double SafetyMonitor::distanceToWorkspaceBoundary(const Eigen::Vector3d& point) const {
    if (workspace_limits_.size() < 6) {
        return std::numeric_limits<double>::max(); // No limits defined
    }
    
    // Calculate distance to each boundary plane and return the minimum
    double dx1 = std::abs(point.x() - workspace_limits_[0]); // distance to xmin
    double dx2 = std::abs(point.x() - workspace_limits_[1]); // distance to xmax
    double dy1 = std::abs(point.y() - workspace_limits_[2]); // distance to ymin
    double dy2 = std::abs(point.y() - workspace_limits_[3]); // distance to ymax
    double dz1 = std::abs(point.z() - workspace_limits_[4]); // distance to zmin
    double dz2 = std::abs(point.z() - workspace_limits_[5]); // distance to zmax
    
    return std::min({dx1, dx2, dy1, dy2, dz1, dz2});
}

bool SafetyMonitor::checkPathCollisions(const geometry_msgs::Pose& start_pose,
                                     const geometry_msgs::Pose& end_pose,
                                     geometry_msgs::Pose& collision_pose) const {
    if (obstacle_positions_.empty()) {
        return false; // No obstacles defined
    }
    
    Eigen::Vector3d start_pos(start_pose.position.x, start_pose.position.y, start_pose.position.z);
    Eigen::Vector3d end_pos(end_pose.position.x, end_pose.position.y, end_pose.position.z);
    
    // Check path discretized at regular intervals
    double path_length = (end_pos - start_pos).norm();
    double step_size = 0.01; // 1cm steps
    int steps = std::ceil(path_length / step_size);
    
    for (int i = 0; i <= steps; ++i) {
        double t = static_cast<double>(i) / steps;
        Eigen::Vector3d pos = start_pos + t * (end_pos - start_pos);
        
        // Check against each obstacle
        for (size_t j = 0; j < obstacle_positions_.size(); ++j) {
            double distance = (pos - obstacle_positions_[j]).norm();
            double min_safe_distance = obstacle_radii_[j] + MIN_OBSTACLE_DISTANCE;
            
            if (distance < min_safe_distance) {
                // Collision detected, populate collision pose
                collision_pose = math_utils::interpolatePoses(start_pose, end_pose, t);
                return true;
            }
        }
    }
    
    return false; // No collisions detected
}

void SafetyMonitor::publishSafetyStatus(const SafetyCheckResult& result) {
    if (!result.is_safe) {
        std_msgs::String msg;
        msg.data = result.message;
        safety_pub_.publish(msg);
        
        ROS_WARN("Safety violation: %s", result.message.c_str());
    }
}

} // namespace safety
} // namespace oculus_arm_control 