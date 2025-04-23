#ifndef OCULUS_ARM_CONTROL_JOINT_CONTROLLER_H
#define OCULUS_ARM_CONTROL_JOINT_CONTROLLER_H

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/Float64MultiArray.h>
#include <vector>
#include <string>
#include <Eigen/Dense>

#include "oculus_arm_control/config.h"
#include "oculus_arm_control/utils/math_utils.h"

namespace oculus_arm_control {
namespace control {

// Class to handle joint-level control of the robot
class JointController {
public:
    JointController(ros::NodeHandle& nh);
    
    // Set joint names
    void setJointNames(const std::vector<std::string>& joint_names);
    
    // Set joint limits
    void setJointLimits(const std::vector<double>& min_limits, const std::vector<double>& max_limits);
    
    // Set robot kinematics parameters
    void setKinematicsParams(const Eigen::VectorXd& link_lengths);
    
    // Forward kinematics: joint positions to end effector pose
    geometry_msgs::Pose forwardKinematics(const std::vector<double>& joint_positions);
    
    // Inverse kinematics: end effector pose to joint positions
    std::vector<double> inverseKinematics(const geometry_msgs::Pose& target_pose);
    
    // Get current joint positions
    std::vector<double> getCurrentJointPositions() const;
    
    // Get current joint velocities
    std::vector<double> getCurrentJointVelocities() const;
    
    // Get current joint efforts
    std::vector<double> getCurrentJointEfforts() const;
    
    // Get current end effector pose
    geometry_msgs::Pose getCurrentPose() const;
    
    // Send joint position command
    void sendJointPositionCommand(const std::vector<double>& joint_positions);
    
    // Send joint velocity command
    void sendJointVelocityCommand(const std::vector<double>& joint_velocities);
    
    // Send joint effort command
    void sendJointEffortCommand(const std::vector<double>& joint_efforts);
    
    // Calculate Jacobian at the current joint positions
    Eigen::MatrixXd calculateJacobian() const;
    
    // Check joint limits
    bool checkJointLimits(const std::vector<double>& joint_positions) const;
    
    // Enforce joint limits
    std::vector<double> enforceJointLimits(const std::vector<double>& joint_positions) const;
    
    // Reset controller
    void reset();
    
private:
    ros::NodeHandle nh_;
    
    // Subscribers and publishers
    ros::Subscriber joint_state_sub_;
    ros::Publisher joint_command_pub_;
    
    // Joint parameters
    std::vector<std::string> joint_names_;
    std::vector<double> joint_min_limits_;
    std::vector<double> joint_max_limits_;
    
    // Current joint state
    sensor_msgs::JointState current_joint_state_;
    
    // Robot kinematics parameters
    Eigen::VectorXd link_lengths_;
    
    // Callbacks
    void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg);
    
    // Utility methods
    std::vector<double> getJointValues(const sensor_msgs::JointState& state, 
                                     const std::vector<std::string>& names) const;
    
    // Differential IK 
    std::vector<double> differentialIK(const geometry_msgs::Pose& current_pose,
                                     const geometry_msgs::Pose& target_pose,
                                     const std::vector<double>& current_joints,
                                     double step_size = 0.1);
};

} // namespace control
} // namespace oculus_arm_control

#endif // OCULUS_ARM_CONTROL_JOINT_CONTROLLER_H 