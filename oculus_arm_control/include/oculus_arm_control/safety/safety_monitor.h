#ifndef OCULUS_ARM_CONTROL_SAFETY_MONITOR_H
#define OCULUS_ARM_CONTROL_SAFETY_MONITOR_H

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <Eigen/Dense>
#include <vector>
#include <string>
#include <map>

#include "oculus_arm_control/config.h"
#include "oculus_arm_control/utils/math_utils.h"

namespace oculus_arm_control {
namespace safety {

// Structure to hold safety check results
struct SafetyCheckResult {
    bool is_safe;
    errors::ErrorCode error_code;
    std::string message;
    geometry_msgs::Pose error_pose;  // Position where the error occurred
};

// Class to monitor safety of robot operations
class SafetyMonitor {
public:
    SafetyMonitor(ros::NodeHandle& nh);
    
    // Set workspace limits
    void setWorkspaceLimits(const std::vector<double>& limits);
    
    // Set joint limits
    void setJointLimits(const std::vector<double>& min_limits, 
                       const std::vector<double>& max_limits);
    
    // Set velocity limits
    void setVelocityLimit(double max_velocity);
    void setVelocityLimits(const std::vector<double>& max_velocities);
    
    // Set safety level
    void setSafetyLevel(safety::SafetyLevel level);
    
    // Set obstacle information for collision checking
    void setObstacles(const std::vector<Eigen::Vector3d>& obstacle_positions,
                     const std::vector<double>& obstacle_radii);
    
    // Check if a pose is within workspace limits
    SafetyCheckResult checkWorkspaceLimits(const geometry_msgs::Pose& pose);
    
    // Check if joint positions are within limits
    SafetyCheckResult checkJointLimits(const std::vector<double>& joint_positions);
    
    // Check if a velocity command is within limits
    SafetyCheckResult checkVelocityLimits(const geometry_msgs::Twist& velocity);
    
    // Check for potential collisions
    SafetyCheckResult checkCollisions(const geometry_msgs::Pose& pose);
    
    // Check motion safety between current pose and target pose
    SafetyCheckResult checkMotionSafety(const geometry_msgs::Pose& current_pose,
                                       const geometry_msgs::Pose& target_pose);
    
    // Check overall safety of a command
    SafetyCheckResult checkCommandSafety(const geometry_msgs::Pose& current_pose,
                                        const geometry_msgs::Pose& target_pose,
                                        const geometry_msgs::Twist& velocity,
                                        const std::vector<double>& joint_positions);
    
    // Emergency stop
    void triggerEmergencyStop();
    
    // Reset after emergency stop
    void resetEmergencyStop();
    
    // Check if emergency stop is active
    bool isEmergencyStopActive() const;
    
    // Callbacks for monitoring
    void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg);
    void robotPoseCallback(const geometry_msgs::Pose::ConstPtr& msg);
    void robotVelocityCallback(const geometry_msgs::Twist::ConstPtr& msg);
    void collisionCallback(const std_msgs::Bool::ConstPtr& msg);
    
    // Enable/disable the safety monitor
    void enable();
    void disable();
    bool isEnabled() const;
    
private:
    ros::NodeHandle nh_;
    
    // Publishers
    ros::Publisher safety_pub_;
    ros::Publisher emergency_stop_pub_;
    
    // Subscribers
    ros::Subscriber joint_state_sub_;
    ros::Subscriber robot_pose_sub_;
    ros::Subscriber robot_velocity_sub_;
    ros::Subscriber collision_sub_;
    
    // Current state
    geometry_msgs::Pose current_pose_;
    geometry_msgs::Twist current_velocity_;
    std::vector<double> current_joint_positions_;
    bool is_emergency_stop_active_;
    bool is_enabled_;
    
    // Safety parameters
    std::vector<double> workspace_limits_;  // [x_min, x_max, y_min, y_max, z_min, z_max]
    std::vector<double> joint_min_limits_;
    std::vector<double> joint_max_limits_;
    double max_velocity_;
    std::vector<double> max_velocities_;  // Per-axis
    safety::SafetyLevel safety_level_;
    
    // Obstacle information
    std::vector<Eigen::Vector3d> obstacle_positions_;
    std::vector<double> obstacle_radii_;
    
    // Helper methods
    bool isPointInWorkspace(const Eigen::Vector3d& point) const;
    double distanceToWorkspaceBoundary(const Eigen::Vector3d& point) const;
    bool checkPathCollisions(const geometry_msgs::Pose& start_pose,
                            const geometry_msgs::Pose& end_pose,
                            geometry_msgs::Pose& collision_pose) const;
    
    // Publish safety status
    void publishSafetyStatus(const SafetyCheckResult& result);
};

} // namespace safety
} // namespace oculus_arm_control

#endif // OCULUS_ARM_CONTROL_SAFETY_MONITOR_H 