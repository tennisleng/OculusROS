#include "oculus_arm_control/utils/math_utils.h"
#include <cmath>

namespace oculus_arm_control {
namespace math_utils {

// Pose transformation and manipulation
Eigen::Isometry3d poseToEigen(const geometry_msgs::Pose& pose) {
    Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
    transform.translation() = Eigen::Vector3d(pose.position.x, pose.position.y, pose.position.z);
    Eigen::Quaterniond q(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
    transform.rotate(q);
    return transform;
}

geometry_msgs::Pose eigenToPose(const Eigen::Isometry3d& transform) {
    geometry_msgs::Pose pose;
    pose.position.x = transform.translation().x();
    pose.position.y = transform.translation().y();
    pose.position.z = transform.translation().z();
    
    Eigen::Quaterniond q(transform.rotation());
    pose.orientation.w = q.w();
    pose.orientation.x = q.x();
    pose.orientation.y = q.y();
    pose.orientation.z = q.z();
    
    return pose;
}

tf2::Transform poseToTf(const geometry_msgs::Pose& pose) {
    tf2::Transform transform;
    tf2::Vector3 origin(pose.position.x, pose.position.y, pose.position.z);
    tf2::Quaternion rotation(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
    transform.setOrigin(origin);
    transform.setRotation(rotation);
    return transform;
}

geometry_msgs::Pose tfToPose(const tf2::Transform& transform) {
    geometry_msgs::Pose pose;
    pose.position.x = transform.getOrigin().x();
    pose.position.y = transform.getOrigin().y();
    pose.position.z = transform.getOrigin().z();
    
    pose.orientation.x = transform.getRotation().x();
    pose.orientation.y = transform.getRotation().y();
    pose.orientation.z = transform.getRotation().z();
    pose.orientation.w = transform.getRotation().w();
    
    return pose;
}

Eigen::Vector3d positionFromPose(const geometry_msgs::Pose& pose) {
    return Eigen::Vector3d(pose.position.x, pose.position.y, pose.position.z);
}

Eigen::Quaterniond orientationFromPose(const geometry_msgs::Pose& pose) {
    return Eigen::Quaterniond(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
}

geometry_msgs::Pose transformPose(const geometry_msgs::Pose& pose, 
                                const Eigen::Isometry3d& transform) {
    Eigen::Isometry3d pose_eigen = poseToEigen(pose);
    Eigen::Isometry3d result = transform * pose_eigen;
    return eigenToPose(result);
}

// Calculate velocity from poses
geometry_msgs::Twist calculateTwist(const geometry_msgs::Pose& pose1, 
                                 const geometry_msgs::Pose& pose2, 
                                 double dt) {
    geometry_msgs::Twist twist;
    
    // Linear velocity
    twist.linear.x = (pose2.position.x - pose1.position.x) / dt;
    twist.linear.y = (pose2.position.y - pose1.position.y) / dt;
    twist.linear.z = (pose2.position.z - pose1.position.z) / dt;
    
    // Angular velocity (simplified approach)
    Eigen::Quaterniond q1(pose1.orientation.w, pose1.orientation.x, pose1.orientation.y, pose1.orientation.z);
    Eigen::Quaterniond q2(pose2.orientation.w, pose2.orientation.x, pose2.orientation.y, pose2.orientation.z);
    
    // Compute the difference quaternion
    Eigen::Quaterniond q_diff = q2 * q1.inverse();
    
    // Convert to axis-angle representation
    Eigen::AngleAxisd angle_axis(q_diff);
    
    // Get the angular velocity
    Eigen::Vector3d angular_velocity = angle_axis.axis() * angle_axis.angle() / dt;
    
    twist.angular.x = angular_velocity.x();
    twist.angular.y = angular_velocity.y();
    twist.angular.z = angular_velocity.z();
    
    return twist;
}

// Calculate distance between poses
double positionDistance(const geometry_msgs::Pose& pose1, 
                      const geometry_msgs::Pose& pose2) {
    double dx = pose2.position.x - pose1.position.x;
    double dy = pose2.position.y - pose1.position.y;
    double dz = pose2.position.z - pose1.position.z;
    
    return std::sqrt(dx*dx + dy*dy + dz*dz);
}

double orientationDistance(const geometry_msgs::Pose& pose1, 
                         const geometry_msgs::Pose& pose2) {
    Eigen::Quaterniond q1(pose1.orientation.w, pose1.orientation.x, pose1.orientation.y, pose1.orientation.z);
    Eigen::Quaterniond q2(pose2.orientation.w, pose2.orientation.x, pose2.orientation.y, pose2.orientation.z);
    
    // Ensure we get the smallest angle between quaternions
    double inner_product = std::abs(q1.x() * q2.x() + q1.y() * q2.y() + q1.z() * q2.z() + q1.w() * q2.w());
    inner_product = std::min(1.0, std::max(-1.0, inner_product));
    
    return 2.0 * std::acos(inner_product);
}

double poseDistance(const geometry_msgs::Pose& pose1, 
                  const geometry_msgs::Pose& pose2,
                  double position_weight,
                  double orientation_weight) {
    double pos_dist = positionDistance(pose1, pose2);
    double orient_dist = orientationDistance(pose1, pose2);
    
    return position_weight * pos_dist + orientation_weight * orient_dist;
}

// Interpolation
geometry_msgs::Pose interpolatePoses(const geometry_msgs::Pose& start,
                                   const geometry_msgs::Pose& end,
                                   double t) {
    // Clamp t to [0, 1]
    t = clamp(t, 0.0, 1.0);
    
    geometry_msgs::Pose result;
    
    // Linear interpolation for position
    result.position.x = start.position.x + t * (end.position.x - start.position.x);
    result.position.y = start.position.y + t * (end.position.y - start.position.y);
    result.position.z = start.position.z + t * (end.position.z - start.position.z);
    
    // Spherical linear interpolation for orientation
    Eigen::Quaterniond q_start = orientationFromPose(start);
    Eigen::Quaterniond q_end = orientationFromPose(end);
    Eigen::Quaterniond q_result = q_start.slerp(t, q_end);
    
    result.orientation.w = q_result.w();
    result.orientation.x = q_result.x();
    result.orientation.y = q_result.y();
    result.orientation.z = q_result.z();
    
    return result;
}

std::vector<geometry_msgs::Pose> generatePosePath(const geometry_msgs::Pose& start,
                                               const geometry_msgs::Pose& end,
                                               double step_size) {
    // Calculate total distance
    double total_distance = positionDistance(start, end);
    int num_steps = std::max(2, static_cast<int>(std::ceil(total_distance / step_size)));
    
    std::vector<geometry_msgs::Pose> path;
    path.reserve(num_steps);
    
    for (int i = 0; i < num_steps; ++i) {
        double t = static_cast<double>(i) / (num_steps - 1);
        path.push_back(interpolatePoses(start, end, t));
    }
    
    return path;
}

// Workspace scaling and manipulation
geometry_msgs::Pose scalePosition(const geometry_msgs::Pose& pose, double scale) {
    geometry_msgs::Pose scaled_pose = pose;
    scaled_pose.position.x *= scale;
    scaled_pose.position.y *= scale;
    scaled_pose.position.z *= scale;
    return scaled_pose;
}

geometry_msgs::Pose scalePosition(const geometry_msgs::Pose& pose, 
                                const Eigen::Vector3d& scale_factors) {
    geometry_msgs::Pose scaled_pose = pose;
    scaled_pose.position.x *= scale_factors.x();
    scaled_pose.position.y *= scale_factors.y();
    scaled_pose.position.z *= scale_factors.z();
    return scaled_pose;
}

// ExponentialFilter implementation
ExponentialFilter::ExponentialFilter(double alpha)
    : alpha_(alpha), initialized_(false),
      last_position_(Eigen::Vector3d::Zero()),
      last_orientation_(Eigen::Quaterniond::Identity()) {
}

Eigen::Vector3d ExponentialFilter::filterPosition(const Eigen::Vector3d& new_position) {
    if (!initialized_) {
        last_position_ = new_position;
        initialized_ = true;
        return new_position;
    }
    
    last_position_ = alpha_ * new_position + (1.0 - alpha_) * last_position_;
    return last_position_;
}

Eigen::Quaterniond ExponentialFilter::filterOrientation(const Eigen::Quaterniond& new_orientation) {
    if (!initialized_) {
        last_orientation_ = new_orientation;
        initialized_ = true;
        return new_orientation;
    }
    
    // SLERP for orientation filtering
    last_orientation_ = last_orientation_.slerp(alpha_, new_orientation);
    return last_orientation_;
}

geometry_msgs::Pose ExponentialFilter::filterPose(const geometry_msgs::Pose& new_pose) {
    Eigen::Vector3d position = positionFromPose(new_pose);
    Eigen::Quaterniond orientation = orientationFromPose(new_pose);
    
    Eigen::Vector3d filtered_position = filterPosition(position);
    Eigen::Quaterniond filtered_orientation = filterOrientation(orientation);
    
    geometry_msgs::Pose filtered_pose;
    filtered_pose.position.x = filtered_position.x();
    filtered_pose.position.y = filtered_position.y();
    filtered_pose.position.z = filtered_position.z();
    
    filtered_pose.orientation.w = filtered_orientation.w();
    filtered_pose.orientation.x = filtered_orientation.x();
    filtered_pose.orientation.y = filtered_orientation.y();
    filtered_pose.orientation.z = filtered_orientation.z();
    
    return filtered_pose;
}

void ExponentialFilter::reset() {
    initialized_ = false;
}

// MovingAverageFilter implementation
MovingAverageFilter::MovingAverageFilter(unsigned int window_size)
    : window_size_(window_size) {
}

Eigen::Vector3d MovingAverageFilter::filterPosition(const Eigen::Vector3d& new_position) {
    // Add new position to the buffer
    position_buffer_.push_back(new_position);
    
    // If buffer is too large, remove oldest element
    if (position_buffer_.size() > window_size_) {
        position_buffer_.erase(position_buffer_.begin());
    }
    
    // Calculate average
    Eigen::Vector3d sum = Eigen::Vector3d::Zero();
    for (const auto& pos : position_buffer_) {
        sum += pos;
    }
    
    return sum / static_cast<double>(position_buffer_.size());
}

Eigen::Quaterniond MovingAverageFilter::filterOrientation(const Eigen::Quaterniond& new_orientation) {
    // Add new orientation to the buffer
    orientation_buffer_.push_back(new_orientation);
    
    // If buffer is too large, remove oldest element
    if (orientation_buffer_.size() > window_size_) {
        orientation_buffer_.erase(orientation_buffer_.begin());
    }
    
    // Calculate average using logarithmic mapping
    Eigen::Vector3d sum_log = Eigen::Vector3d::Zero();
    for (const auto& q : orientation_buffer_) {
        // Convert to axis-angle
        Eigen::AngleAxisd aa(q);
        // Create log representation (angle * axis)
        sum_log += aa.angle() * aa.axis();
    }
    
    sum_log /= static_cast<double>(orientation_buffer_.size());
    
    // Convert back to quaternion
    double angle = sum_log.norm();
    Eigen::Vector3d axis = sum_log.normalized();
    
    if (angle < 1e-10) {
        return Eigen::Quaterniond::Identity();
    }
    
    return Eigen::Quaterniond(Eigen::AngleAxisd(angle, axis));
}

geometry_msgs::Pose MovingAverageFilter::filterPose(const geometry_msgs::Pose& new_pose) {
    Eigen::Vector3d position = positionFromPose(new_pose);
    Eigen::Quaterniond orientation = orientationFromPose(new_pose);
    
    Eigen::Vector3d filtered_position = filterPosition(position);
    Eigen::Quaterniond filtered_orientation = filterOrientation(orientation);
    
    geometry_msgs::Pose filtered_pose;
    filtered_pose.position.x = filtered_position.x();
    filtered_pose.position.y = filtered_position.y();
    filtered_pose.position.z = filtered_position.z();
    
    filtered_pose.orientation.w = filtered_orientation.w();
    filtered_pose.orientation.x = filtered_orientation.x();
    filtered_pose.orientation.y = filtered_orientation.y();
    filtered_pose.orientation.z = filtered_orientation.z();
    
    return filtered_pose;
}

void MovingAverageFilter::reset() {
    position_buffer_.clear();
    orientation_buffer_.clear();
}

// Jacobian calculation (simplified 6-DOF robot with revolute joints)
Eigen::MatrixXd calculateJacobian(const std::vector<double>& joint_positions, 
                                const Eigen::VectorXd& link_lengths) {
    int num_joints = joint_positions.size();
    // Create 6xn Jacobian matrix (3 for position, 3 for orientation)
    Eigen::MatrixXd J = Eigen::MatrixXd::Zero(6, num_joints);
    
    // For a robot with revolute joints in a serial chain
    // Calculate forward kinematics to get joint positions in world frame
    std::vector<Eigen::Vector3d> joint_positions_world;
    std::vector<Eigen::Vector3d> z_axes;
    
    // Initialize with base at origin
    Eigen::Vector3d pos = Eigen::Vector3d::Zero();
    Eigen::Vector3d z_axis = Eigen::Vector3d::UnitZ(); // Z-axis of joint rotation
    
    joint_positions_world.push_back(pos);
    z_axes.push_back(z_axis);
    
    Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
    double cumulative_angle = 0.0;
    
    // Forward kinematics to compute joint positions
    for (int i = 0; i < num_joints; i++) {
        cumulative_angle += joint_positions[i];
        
        // Rotation matrix for current joint
        Eigen::Matrix3d R_i;
        R_i = Eigen::AngleAxisd(cumulative_angle, Eigen::Vector3d::UnitZ()).toRotationMatrix();
        R = R_i;
        
        // Update position (simplified planar case)
        pos += R * Eigen::Vector3d(link_lengths(i), 0, 0);
        
        // Z-axis after rotation
        z_axis = R * Eigen::Vector3d::UnitZ();
        
        joint_positions_world.push_back(pos);
        z_axes.push_back(z_axis);
    }
    
    // End-effector position
    Eigen::Vector3d end_effector_pos = joint_positions_world.back();
    
    // Calculate Jacobian
    for (int i = 0; i < num_joints; i++) {
        // Position part of Jacobian (cross product of z-axis with displacement)
        Eigen::Vector3d displacement = end_effector_pos - joint_positions_world[i];
        Eigen::Vector3d J_pos = z_axes[i].cross(displacement);
        
        // Orientation part of Jacobian (z-axis of rotation)
        Eigen::Vector3d J_ori = z_axes[i];
        
        // Fill Jacobian matrix
        J.block<3, 1>(0, i) = J_pos;
        J.block<3, 1>(3, i) = J_ori;
    }
    
    return J;
}

// Simplified inverse kinematics using Jacobian pseudoinverse
std::vector<double> inverseKinematics(const geometry_msgs::Pose& target_pose,
                                   const std::vector<double>& seed,
                                   const Eigen::VectorXd& link_lengths,
                                   double tolerance,
                                   int max_iterations) {
    // Start with the seed joint positions
    std::vector<double> joint_positions = seed;
    int dof = joint_positions.size();
    
    // Create target position and orientation vectors
    Eigen::Vector3d target_position(target_pose.position.x, target_pose.position.y, target_pose.position.z);
    Eigen::Quaterniond target_orientation(target_pose.orientation.w, target_pose.orientation.x, 
                                         target_pose.orientation.y, target_pose.orientation.z);
    
    // Convert to 6D pose vector
    Eigen::VectorXd target_pose_vector(6);
    target_pose_vector.head(3) = target_position;
    
    // Convert quaternion to RPY for simplicity
    Eigen::Vector3d target_rpy = quaternionToRPY(target_orientation);
    target_pose_vector.tail(3) = target_rpy;
    
    // Iterative resolution
    for (int iter = 0; iter < max_iterations; ++iter) {
        // Calculate forward kinematics
        // (In a real system, this would call the robot's FK function)
        
        // For now, let's assume we have a current pose from FK
        geometry_msgs::Pose current_pose;
        // ... calculate current_pose using FK from joint_positions ...
        
        // Convert current pose to 6D vector
        Eigen::Vector3d current_position(current_pose.position.x, current_pose.position.y, current_pose.position.z);
        Eigen::Quaterniond current_orientation(current_pose.orientation.w, current_pose.orientation.x, 
                                              current_pose.orientation.y, current_pose.orientation.z);
        
        Eigen::VectorXd current_pose_vector(6);
        current_pose_vector.head(3) = current_position;
        
        Eigen::Vector3d current_rpy = quaternionToRPY(current_orientation);
        current_pose_vector.tail(3) = current_rpy;
        
        // Calculate error
        Eigen::VectorXd error = target_pose_vector - current_pose_vector;
        
        // Check if we've reached the target
        if (error.norm() < tolerance) {
            break;
        }
        
        // Calculate Jacobian
        Eigen::MatrixXd jacobian = calculateJacobian(joint_positions, link_lengths);
        
        // Calculate pseudoinverse of the Jacobian
        Eigen::MatrixXd J_pinv = jacobian.transpose() * 
                                 (jacobian * jacobian.transpose()).inverse();
        
        // Calculate joint updates
        Eigen::VectorXd delta_theta = J_pinv * error;
        
        // Update joint positions
        for (int i = 0; i < dof; ++i) {
            joint_positions[i] += delta_theta(i);
        }
    }
    
    return joint_positions;
}

// Collision detection
bool checkSphereCollision(const Eigen::Vector3d& center1, double radius1,
                        const Eigen::Vector3d& center2, double radius2) {
    double distance = (center2 - center1).norm();
    return distance < (radius1 + radius2);
}

double calculateMinDistance(const Eigen::Vector3d& point,
                         const std::vector<Eigen::Vector3d>& obstacle_points) {
    if (obstacle_points.empty()) {
        return std::numeric_limits<double>::max();
    }
    
    double min_distance = std::numeric_limits<double>::max();
    
    for (const auto& obstacle_point : obstacle_points) {
        double distance = (obstacle_point - point).norm();
        min_distance = std::min(min_distance, distance);
    }
    
    return min_distance;
}

// Rotation conversions
Eigen::Vector3d quaternionToRPY(const Eigen::Quaterniond& q) {
    Eigen::Vector3d rpy;
    
    // Roll (x-axis rotation)
    double sinr_cosp = 2.0 * (q.w() * q.x() + q.y() * q.z());
    double cosr_cosp = 1.0 - 2.0 * (q.x() * q.x() + q.y() * q.y());
    rpy(0) = std::atan2(sinr_cosp, cosr_cosp);
    
    // Pitch (y-axis rotation)
    double sinp = 2.0 * (q.w() * q.y() - q.z() * q.x());
    if (std::abs(sinp) >= 1)
        rpy(1) = std::copysign(M_PI / 2, sinp); // Use 90 degrees if out of range
    else
        rpy(1) = std::asin(sinp);
    
    // Yaw (z-axis rotation)
    double siny_cosp = 2.0 * (q.w() * q.z() + q.x() * q.y());
    double cosy_cosp = 1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z());
    rpy(2) = std::atan2(siny_cosp, cosy_cosp);
    
    return rpy;
}

Eigen::Quaterniond rpyToQuaternion(const Eigen::Vector3d& rpy) {
    // Convert roll, pitch, yaw to quaternion
    Eigen::AngleAxisd rollAngle(rpy(0), Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitchAngle(rpy(1), Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yawAngle(rpy(2), Eigen::Vector3d::UnitZ());
    
    Eigen::Quaterniond q = yawAngle * pitchAngle * rollAngle;
    return q;
}

Eigen::Quaterniond RPYToQuaternion(double roll, double pitch, double yaw) {
    // Convert Euler angles to quaternion
    Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitZ());
    
    Eigen::Quaterniond q = yawAngle * pitchAngle * rollAngle;
    return q;
}

Eigen::Quaterniond RPYToQuaternion(const Eigen::Vector3d& rpy) {
    return RPYToQuaternion(rpy.x(), rpy.y(), rpy.z());
}

// Limits enforcement
Eigen::Vector3d enforcePositionLimits(const Eigen::Vector3d& position,
                                   const Eigen::Vector3d& min_limits,
                                   const Eigen::Vector3d& max_limits) {
    Eigen::Vector3d limited_position;
    limited_position.x() = clamp(position.x(), min_limits.x(), max_limits.x());
    limited_position.y() = clamp(position.y(), min_limits.y(), max_limits.y());
    limited_position.z() = clamp(position.z(), min_limits.z(), max_limits.z());
    return limited_position;
}

Eigen::Vector3d enforceVelocityLimits(const Eigen::Vector3d& velocity,
                                   double max_velocity) {
    double magnitude = velocity.norm();
    
    if (magnitude > max_velocity && magnitude > 0) {
        return velocity * (max_velocity / magnitude);
    }
    
    return velocity;
}

Eigen::Vector3d enforceVelocityLimits(const Eigen::Vector3d& velocity,
                                   const Eigen::Vector3d& max_velocities) {
    Eigen::Vector3d limited_velocity;
    limited_velocity.x() = clamp(velocity.x(), -max_velocities.x(), max_velocities.x());
    limited_velocity.y() = clamp(velocity.y(), -max_velocities.y(), max_velocities.y());
    limited_velocity.z() = clamp(velocity.z(), -max_velocities.z(), max_velocities.z());
    return limited_velocity;
}

// Utility functions
double clamp(double value, double min_val, double max_val) {
    return std::max(min_val, std::min(max_val, value));
}

bool isInBounds(const Eigen::Vector3d& value, 
              const Eigen::Vector3d& min_bounds,
              const Eigen::Vector3d& max_bounds) {
    return (value.x() >= min_bounds.x() && value.x() <= max_bounds.x() &&
            value.y() >= min_bounds.y() && value.y() <= max_bounds.y() &&
            value.z() >= min_bounds.z() && value.z() <= max_bounds.z());
}

Eigen::Quaterniond quaternionMsgToEigen(const geometry_msgs::Quaternion& q) {
    return Eigen::Quaterniond(q.w, q.x, q.y, q.z);
}

geometry_msgs::Quaternion quaternionEigenToMsg(const Eigen::Quaterniond& q) {
    geometry_msgs::Quaternion quat;
    quat.w = q.w();
    quat.x = q.x();
    quat.y = q.y();
    quat.z = q.z();
    return quat;
}

Eigen::Vector3d vector3MsgToEigen(const geometry_msgs::Vector3& v) {
    return Eigen::Vector3d(v.x, v.y, v.z);
}

geometry_msgs::Vector3 vector3EigenToMsg(const Eigen::Vector3d& v) {
    geometry_msgs::Vector3 vec;
    vec.x = v.x();
    vec.y = v.y();
    vec.z = v.z();
    return vec;
}

Eigen::Isometry3d poseMsgToEigen(const geometry_msgs::Pose& pose) {
    Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
    transform.translation() = Eigen::Vector3d(pose.position.x, pose.position.y, pose.position.z);
    transform.rotate(Eigen::Quaterniond(pose.orientation.w, pose.orientation.x, 
                                        pose.orientation.y, pose.orientation.z));
    return transform;
}

geometry_msgs::Pose poseEigenToMsg(const Eigen::Isometry3d& transform) {
    geometry_msgs::Pose pose;
    Eigen::Vector3d translation = transform.translation();
    Eigen::Quaterniond rotation(transform.rotation());
    
    pose.position.x = translation.x();
    pose.position.y = translation.y();
    pose.position.z = translation.z();
    
    pose.orientation.w = rotation.w();
    pose.orientation.x = rotation.x();
    pose.orientation.y = rotation.y();
    pose.orientation.z = rotation.z();
    
    return pose;
}

double lowPassFilter(double input, double prev_output, double alpha) {
    return alpha * input + (1.0 - alpha) * prev_output;
}

Eigen::Vector3d lowPassFilterVector3d(const Eigen::Vector3d& input, 
                                     const Eigen::Vector3d& prev_output, 
                                     double alpha) {
    Eigen::Vector3d result;
    for (int i = 0; i < 3; i++) {
        result(i) = lowPassFilter(input(i), prev_output(i), alpha);
    }
    return result;
}

Eigen::Quaterniond lowPassFilterQuaternion(const Eigen::Quaterniond& input, 
                                          const Eigen::Quaterniond& prev_output, 
                                          double alpha) {
    // Use SLERP for quaternion interpolation
    return prev_output.slerp(alpha, input);
}

double minimumJerkTrajectory(double t, double T, double p0, double pf) {
    if (t <= 0) return p0;
    if (t >= T) return pf;
    
    double normalized_time = t / T;
    double scale = 10 * std::pow(normalized_time, 3) - 15 * std::pow(normalized_time, 4) + 6 * std::pow(normalized_time, 5);
    
    return p0 + (pf - p0) * scale;
}

double minimumJerkTrajectoryVel(double t, double T, double p0, double pf) {
    if (t <= 0 || t >= T) return 0.0;
    
    double normalized_time = t / T;
    double scale_derivative = (30 * std::pow(normalized_time, 2) - 60 * std::pow(normalized_time, 3) + 30 * std::pow(normalized_time, 4)) / T;
    
    return (pf - p0) * scale_derivative;
}

double minimumJerkTrajectoryAcc(double t, double T, double p0, double pf) {
    if (t <= 0 || t >= T) return 0.0;
    
    double normalized_time = t / T;
    double scale_second_derivative = (60 * normalized_time - 180 * std::pow(normalized_time, 2) + 120 * std::pow(normalized_time, 3)) / (T * T);
    
    return (pf - p0) * scale_second_derivative;
}

} // namespace math_utils
} // namespace oculus_arm_control 