#include "oculus_arm_control/control/joint_controller.h"

namespace oculus_arm_control {
namespace control {

JointController::JointController(ros::NodeHandle& nh)
    : nh_(nh) {
    
    // Initialize subscribers and publishers
    joint_state_sub_ = nh_.subscribe(topics::ROBOT_JOINT_STATES, 10, &JointController::jointStateCallback, this);
    joint_command_pub_ = nh_.advertise<std_msgs::Float64MultiArray>(topics::ROBOT_JOINT_COMMAND, 10);
    
    // Initialize empty joint state
    current_joint_state_.name.clear();
    current_joint_state_.position.clear();
    current_joint_state_.velocity.clear();
    current_joint_state_.effort.clear();
    
    ROS_INFO("Joint controller initialized");
}

void JointController::setJointNames(const std::vector<std::string>& joint_names) {
    joint_names_ = joint_names;
    ROS_INFO("Set %ld joint names", joint_names.size());
}

void JointController::setJointLimits(const std::vector<double>& min_limits, const std::vector<double>& max_limits) {
    if (min_limits.size() != max_limits.size()) {
        ROS_WARN("Joint limit vectors must have the same size");
        return;
    }
    
    joint_min_limits_ = min_limits;
    joint_max_limits_ = max_limits;
    
    ROS_INFO("Set joint limits for %ld joints", min_limits.size());
}

void JointController::setKinematicsParams(const Eigen::VectorXd& link_lengths) {
    link_lengths_ = link_lengths;
    ROS_INFO("Set kinematics parameters with %ld link lengths", link_lengths.size());
}

geometry_msgs::Pose JointController::forwardKinematics(const std::vector<double>& joint_positions) {
    // For a real robot, you would implement actual forward kinematics here
    // using the robot's specific kinematics model
    
    // This is a simplified placeholder implementation
    // Assuming a 6-DOF robot with standard DH parameters
    
    // Create identity pose as fallback
    geometry_msgs::Pose pose;
    pose.position.x = 0;
    pose.position.y = 0;
    pose.position.z = 0;
    pose.orientation.x = 0;
    pose.orientation.y = 0;
    pose.orientation.z = 0;
    pose.orientation.w = 1;
    
    if (joint_positions.empty() || link_lengths_.size() < joint_positions.size()) {
        ROS_WARN("Cannot perform forward kinematics: insufficient joint or link data");
        return pose;
    }
    
    // Create 4x4 transformation matrix, starting with identity
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    
    // Apply each joint transformation
    for (size_t i = 0; i < joint_positions.size(); ++i) {
        double theta = joint_positions[i];
        double d = 0.0;            // Assuming no prismatic joints
        double a = link_lengths_[i]; // Link length from DH parameters
        double alpha = 0.0;        // Assuming no joint twists for simplicity
        
        if (i % 2 == 1) {
            // Every other joint has a 90-degree twist
            alpha = M_PI / 2.0;
        }
        
        // DH transformation matrix
        Eigen::Matrix4d A;
        A << cos(theta), -sin(theta)*cos(alpha), sin(theta)*sin(alpha), a*cos(theta),
             sin(theta), cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta),
             0, sin(alpha), cos(alpha), d,
             0, 0, 0, 1;
        
        // Update the transformation
        T = T * A;
    }
    
    // Extract position from the transformation matrix
    pose.position.x = T(0, 3);
    pose.position.y = T(1, 3);
    pose.position.z = T(2, 3);
    
    // Extract orientation (converting the rotation matrix to a quaternion)
    Eigen::Matrix3d rot = T.block<3, 3>(0, 0);
    Eigen::Quaterniond q(rot);
    pose.orientation.x = q.x();
    pose.orientation.y = q.y();
    pose.orientation.z = q.z();
    pose.orientation.w = q.w();
    
    return pose;
}

std::vector<double> JointController::inverseKinematics(const geometry_msgs::Pose& target_pose) {
    // Get current joint positions as the starting point
    std::vector<double> current_joints = getCurrentJointPositions();
    
    if (current_joints.empty()) {
        ROS_WARN("Cannot perform inverse kinematics: no current joint state");
        return std::vector<double>();
    }
    
    // Get current end effector pose
    geometry_msgs::Pose current_pose = getCurrentPose();
    
    // Use differential IK to find the solution
    return differentialIK(current_pose, target_pose, current_joints);
}

std::vector<double> JointController::getCurrentJointPositions() const {
    return getJointValues(current_joint_state_, joint_names_);
}

std::vector<double> JointController::getCurrentJointVelocities() const {
    return getJointValues(current_joint_state_, joint_names_);
}

std::vector<double> JointController::getCurrentJointEfforts() const {
    return getJointValues(current_joint_state_, joint_names_);
}

geometry_msgs::Pose JointController::getCurrentPose() const {
    // Calculate forward kinematics using current joint positions
    return forwardKinematics(getCurrentJointPositions());
}

void JointController::sendJointPositionCommand(const std::vector<double>& joint_positions) {
    if (joint_positions.size() != joint_names_.size()) {
        ROS_WARN("Joint command size (%ld) does not match joint names size (%ld)",
                joint_positions.size(), joint_names_.size());
        return;
    }
    
    // Enforce joint limits
    std::vector<double> limited_positions = enforceJointLimits(joint_positions);
    
    // Create and publish the command message
    std_msgs::Float64MultiArray cmd_msg;
    cmd_msg.data = limited_positions;
    
    joint_command_pub_.publish(cmd_msg);
}

void JointController::sendJointVelocityCommand(const std::vector<double>& joint_velocities) {
    if (joint_velocities.size() != joint_names_.size()) {
        ROS_WARN("Joint velocity command size does not match joint names size");
        return;
    }
    
    // For now, this is a placeholder
    // A real implementation would send velocity commands to the robot
    
    ROS_WARN("Joint velocity control not implemented yet");
}

void JointController::sendJointEffortCommand(const std::vector<double>& joint_efforts) {
    if (joint_efforts.size() != joint_names_.size()) {
        ROS_WARN("Joint effort command size does not match joint names size");
        return;
    }
    
    // For now, this is a placeholder
    // A real implementation would send effort commands to the robot
    
    ROS_WARN("Joint effort control not implemented yet");
}

Eigen::MatrixXd JointController::calculateJacobian() const {
    // Calculate the Jacobian at the current joint positions
    std::vector<double> joint_positions = getCurrentJointPositions();
    
    if (joint_positions.empty() || link_lengths_.size() < joint_positions.size()) {
        ROS_WARN("Cannot calculate Jacobian: insufficient joint or link data");
        return Eigen::MatrixXd::Zero(6, 1);  // Return empty Jacobian
    }
    
    return math_utils::calculateJacobian(joint_positions, link_lengths_);
}

bool JointController::checkJointLimits(const std::vector<double>& joint_positions) const {
    if (joint_positions.size() != joint_min_limits_.size() || 
        joint_positions.size() != joint_max_limits_.size()) {
        ROS_WARN("Joint position size does not match joint limit size");
        return false;
    }
    
    for (size_t i = 0; i < joint_positions.size(); ++i) {
        if (joint_positions[i] < joint_min_limits_[i] || 
            joint_positions[i] > joint_max_limits_[i]) {
            return false;
        }
    }
    
    return true;
}

std::vector<double> JointController::enforceJointLimits(const std::vector<double>& joint_positions) const {
    std::vector<double> limited_positions = joint_positions;
    
    if (joint_min_limits_.size() != joint_positions.size() || 
        joint_max_limits_.size() != joint_positions.size()) {
        ROS_WARN("Joint limits not defined for all joints, not enforcing limits");
        return limited_positions;
    }
    
    for (size_t i = 0; i < limited_positions.size(); ++i) {
        limited_positions[i] = std::max(joint_min_limits_[i], 
                                      std::min(joint_max_limits_[i], limited_positions[i]));
    }
    
    return limited_positions;
}

void JointController::reset() {
    // Reset the current joint state
    current_joint_state_.name.clear();
    current_joint_state_.position.clear();
    current_joint_state_.velocity.clear();
    current_joint_state_.effort.clear();
    
    ROS_INFO("Joint controller reset");
}

void JointController::jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg) {
    // Update current joint state
    current_joint_state_ = *msg;
}

std::vector<double> JointController::getJointValues(const sensor_msgs::JointState& state, 
                                                 const std::vector<std::string>& names) const {
    std::vector<double> values(names.size(), 0.0);
    
    // Create a map from joint name to index
    std::map<std::string, size_t> name_to_index;
    for (size_t i = 0; i < state.name.size(); ++i) {
        name_to_index[state.name[i]] = i;
    }
    
    // Extract values for the requested joints
    for (size_t i = 0; i < names.size(); ++i) {
        auto it = name_to_index.find(names[i]);
        if (it != name_to_index.end() && it->second < state.position.size()) {
            values[i] = state.position[it->second];
        }
    }
    
    return values;
}

std::vector<double> JointController::differentialIK(const geometry_msgs::Pose& current_pose,
                                                 const geometry_msgs::Pose& target_pose,
                                                 const std::vector<double>& current_joints,
                                                 double step_size) {
    // Set up IK parameters
    const int max_iterations = 100;
    const double tolerance = 1e-4;
    
    // Start with current joint positions
    std::vector<double> joint_positions = current_joints;
    
    // Convert poses to Eigen
    Eigen::Vector3d current_position(current_pose.position.x, current_pose.position.y, current_pose.position.z);
    Eigen::Quaterniond current_orientation(current_pose.orientation.w, current_pose.orientation.x, 
                                         current_pose.orientation.y, current_pose.orientation.z);
    
    Eigen::Vector3d target_position(target_pose.position.x, target_pose.position.y, target_pose.position.z);
    Eigen::Quaterniond target_orientation(target_pose.orientation.w, target_pose.orientation.x, 
                                        target_pose.orientation.y, target_pose.orientation.z);
    
    // Iterative differential IK
    for (int iter = 0; iter < max_iterations; ++iter) {
        // Calculate current end effector pose via forward kinematics
        geometry_msgs::Pose current_ee_pose = forwardKinematics(joint_positions);
        
        Eigen::Vector3d ee_position(current_ee_pose.position.x, current_ee_pose.position.y, current_ee_pose.position.z);
        Eigen::Quaterniond ee_orientation(current_ee_pose.orientation.w, current_ee_pose.orientation.x, 
                                        current_ee_pose.orientation.y, current_ee_pose.orientation.z);
        
        // Calculate position error
        Eigen::Vector3d position_error = target_position - ee_position;
        
        // Calculate orientation error (simplified)
        Eigen::Quaterniond orientation_error = target_orientation * ee_orientation.inverse();
        Eigen::AngleAxisd angle_axis(orientation_error);
        Eigen::Vector3d orientation_error_vec = angle_axis.axis() * angle_axis.angle();
        
        // Check if we've reached the target
        double position_distance = position_error.norm();
        double orientation_distance = orientation_error_vec.norm();
        
        if (position_distance < tolerance && orientation_distance < tolerance) {
            break;
        }
        
        // Combine into a single 6D error vector
        Eigen::VectorXd error(6);
        error.head(3) = position_error;
        error.tail(3) = orientation_error_vec;
        
        // Scale the error
        error = error * step_size;
        
        // Calculate the Jacobian
        Eigen::MatrixXd jacobian = math_utils::calculateJacobian(joint_positions, link_lengths_);
        
        // Calculate the pseudoinverse of the Jacobian
        Eigen::MatrixXd J_pinv = jacobian.transpose() * 
                               (jacobian * jacobian.transpose()).inverse();
        
        // Calculate joint updates
        Eigen::VectorXd delta_theta = J_pinv * error;
        
        // Update joint positions
        for (size_t i = 0; i < joint_positions.size() && i < static_cast<size_t>(delta_theta.size()); ++i) {
            joint_positions[i] += delta_theta(i);
        }
        
        // Enforce joint limits
        joint_positions = enforceJointLimits(joint_positions);
    }
    
    return joint_positions;
}

} // namespace control
} // namespace oculus_arm_control 