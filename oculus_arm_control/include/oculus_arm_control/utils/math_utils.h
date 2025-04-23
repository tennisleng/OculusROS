#ifndef OCULUS_ARM_CONTROL_MATH_UTILS_H
#define OCULUS_ARM_CONTROL_MATH_UTILS_H

#include <vector>
#include <cmath>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace oculus_arm_control {
namespace math_utils {

// Pose transformation and manipulation
Eigen::Isometry3d poseToEigen(const geometry_msgs::Pose& pose);
geometry_msgs::Pose eigenToPose(const Eigen::Isometry3d& transform);

tf2::Transform poseToTf(const geometry_msgs::Pose& pose);
geometry_msgs::Pose tfToPose(const tf2::Transform& transform);

Eigen::Vector3d positionFromPose(const geometry_msgs::Pose& pose);
Eigen::Quaterniond orientationFromPose(const geometry_msgs::Pose& pose);

geometry_msgs::Pose transformPose(const geometry_msgs::Pose& pose, 
                                  const Eigen::Isometry3d& transform);

// Calculate velocity from poses
geometry_msgs::Twist calculateTwist(const geometry_msgs::Pose& pose1, 
                                   const geometry_msgs::Pose& pose2, 
                                   double dt);

// Calculate distance between poses
double positionDistance(const geometry_msgs::Pose& pose1, 
                       const geometry_msgs::Pose& pose2);
                       
double orientationDistance(const geometry_msgs::Pose& pose1, 
                          const geometry_msgs::Pose& pose2);

double poseDistance(const geometry_msgs::Pose& pose1, 
                   const geometry_msgs::Pose& pose2,
                   double position_weight = 1.0,
                   double orientation_weight = 0.2);

// Interpolation
geometry_msgs::Pose interpolatePoses(const geometry_msgs::Pose& start,
                                    const geometry_msgs::Pose& end,
                                    double t);

std::vector<geometry_msgs::Pose> generatePosePath(const geometry_msgs::Pose& start,
                                                const geometry_msgs::Pose& end,
                                                double step_size);

// Workspace scaling and manipulation
geometry_msgs::Pose scalePosition(const geometry_msgs::Pose& pose, double scale);
geometry_msgs::Pose scalePosition(const geometry_msgs::Pose& pose, 
                                 const Eigen::Vector3d& scale_factors);

// Filtering
class ExponentialFilter {
public:
    ExponentialFilter(double alpha = 0.2);
    Eigen::Vector3d filterPosition(const Eigen::Vector3d& new_position);
    Eigen::Quaterniond filterOrientation(const Eigen::Quaterniond& new_orientation);
    geometry_msgs::Pose filterPose(const geometry_msgs::Pose& new_pose);
    void reset();
    
private:
    double alpha_;
    bool initialized_;
    Eigen::Vector3d last_position_;
    Eigen::Quaterniond last_orientation_;
};

class MovingAverageFilter {
public:
    MovingAverageFilter(unsigned int window_size = 10);
    Eigen::Vector3d filterPosition(const Eigen::Vector3d& new_position);
    Eigen::Quaterniond filterOrientation(const Eigen::Quaterniond& new_orientation);
    geometry_msgs::Pose filterPose(const geometry_msgs::Pose& new_pose);
    void reset();
    
private:
    unsigned int window_size_;
    std::vector<Eigen::Vector3d> position_buffer_;
    std::vector<Eigen::Quaterniond> orientation_buffer_;
};

// Joint and kinematics calculations
Eigen::MatrixXd calculateJacobian(const std::vector<double>& joint_positions, 
                                 const Eigen::VectorXd& link_lengths);

std::vector<double> inverseKinematics(const geometry_msgs::Pose& target_pose,
                                    const std::vector<double>& seed,
                                    const Eigen::VectorXd& link_lengths,
                                    double tolerance = 1e-6,
                                    int max_iterations = 100);

// Collision detection
bool checkSphereCollision(const Eigen::Vector3d& center1, double radius1,
                         const Eigen::Vector3d& center2, double radius2);
                         
double calculateMinDistance(const Eigen::Vector3d& point,
                          const std::vector<Eigen::Vector3d>& obstacle_points);

// Trajectory generation
struct TrajectoryParams {
    double max_velocity;
    double max_acceleration;
    double max_jerk;
    double time_step;
};

struct TrajectoryPoint {
    double position;
    double velocity;
    double acceleration;
    double time;
};

std::vector<TrajectoryPoint> generateTrapezoidal(double start, double goal, 
                                               const TrajectoryParams& params);

std::vector<TrajectoryPoint> generateMinimumJerk(double start, double goal, 
                                              double duration,
                                              double time_step);

// Path planning
std::vector<Eigen::Vector3d> generateRRT(const Eigen::Vector3d& start,
                                        const Eigen::Vector3d& goal,
                                        const std::vector<Eigen::Vector3d>& obstacles,
                                        double step_size,
                                        int max_iterations);

// Rotation conversions
Eigen::Vector3d quaternionToRPY(const Eigen::Quaterniond& q);
Eigen::Quaterniond RPYToQuaternion(double roll, double pitch, double yaw);
Eigen::Quaterniond RPYToQuaternion(const Eigen::Vector3d& rpy);

// Limits enforcement
Eigen::Vector3d enforcePositionLimits(const Eigen::Vector3d& position,
                                    const Eigen::Vector3d& min_limits,
                                    const Eigen::Vector3d& max_limits);

Eigen::Vector3d enforceVelocityLimits(const Eigen::Vector3d& velocity,
                                    double max_velocity);

Eigen::Vector3d enforceVelocityLimits(const Eigen::Vector3d& velocity,
                                    const Eigen::Vector3d& max_velocities);

// Utility functions
double clamp(double value, double min_val, double max_val);
bool isInBounds(const Eigen::Vector3d& value, 
               const Eigen::Vector3d& min_bounds,
               const Eigen::Vector3d& max_bounds);

/**
 * @brief Convert a geometry_msgs::Quaternion to an Eigen::Quaterniond
 * 
 * @param q The geometry_msgs::Quaternion to convert
 * @return Eigen::Quaterniond The converted quaternion
 */
Eigen::Quaterniond quaternionMsgToEigen(const geometry_msgs::Quaternion& q);

/**
 * @brief Convert an Eigen::Quaterniond to a geometry_msgs::Quaternion
 * 
 * @param q The Eigen::Quaterniond to convert
 * @return geometry_msgs::Quaternion The converted quaternion
 */
geometry_msgs::Quaternion quaternionEigenToMsg(const Eigen::Quaterniond& q);

/**
 * @brief Convert a geometry_msgs::Vector3 to an Eigen::Vector3d
 * 
 * @param v The geometry_msgs::Vector3 to convert
 * @return Eigen::Vector3d The converted vector
 */
Eigen::Vector3d vector3MsgToEigen(const geometry_msgs::Vector3& v);

/**
 * @brief Convert an Eigen::Vector3d to a geometry_msgs::Vector3
 * 
 * @param v The Eigen::Vector3d to convert
 * @return geometry_msgs::Vector3 The converted vector
 */
geometry_msgs::Vector3 vector3EigenToMsg(const Eigen::Vector3d& v);

/**
 * @brief Convert a geometry_msgs::Pose to an Eigen::Isometry3d
 * 
 * @param pose The geometry_msgs::Pose to convert
 * @return Eigen::Isometry3d The converted pose
 */
Eigen::Isometry3d poseMsgToEigen(const geometry_msgs::Pose& pose);

/**
 * @brief Convert an Eigen::Isometry3d to a geometry_msgs::Pose
 * 
 * @param transform The Eigen::Isometry3d to convert
 * @return geometry_msgs::Pose The converted pose
 */
geometry_msgs::Pose poseEigenToMsg(const Eigen::Isometry3d& transform);

/**
 * @brief Low-pass filter for smoothing signals
 * 
 * @param input The new input value
 * @param prev_output The previous filtered output
 * @param alpha The filter coefficient (0-1), smaller values mean more filtering
 * @return double The filtered output
 */
double lowPassFilter(double input, double prev_output, double alpha);

/**
 * @brief Apply low-pass filter to a vector
 * 
 * @param input The new input vector
 * @param prev_output The previous filtered output vector
 * @param alpha The filter coefficient (0-1)
 * @return Eigen::Vector3d The filtered output vector
 */
Eigen::Vector3d lowPassFilterVector3d(const Eigen::Vector3d& input, 
                                     const Eigen::Vector3d& prev_output, 
                                     double alpha);

/**
 * @brief Apply low-pass filter to a quaternion
 * 
 * @param input The new input quaternion
 * @param prev_output The previous filtered output quaternion
 * @param alpha The filter coefficient (0-1)
 * @return Eigen::Quaterniond The filtered output quaternion
 */
Eigen::Quaterniond lowPassFilterQuaternion(const Eigen::Quaterniond& input, 
                                          const Eigen::Quaterniond& prev_output, 
                                          double alpha);

/**
 * @brief Calculate the minimum jerk trajectory position
 * 
 * @param t Current time
 * @param T Total duration
 * @param p0 Start position
 * @param pf End position
 * @return double Position at time t
 */
double minimumJerkTrajectory(double t, double T, double p0, double pf);

/**
 * @brief Calculate the minimum jerk trajectory velocity
 * 
 * @param t Current time
 * @param T Total duration
 * @param p0 Start position
 * @param pf End position
 * @return double Velocity at time t
 */
double minimumJerkTrajectoryVel(double t, double T, double p0, double pf);

/**
 * @brief Calculate the minimum jerk trajectory acceleration
 * 
 * @param t Current time
 * @param T Total duration
 * @param p0 Start position
 * @param pf End position
 * @return double Acceleration at time t
 */
double minimumJerkTrajectoryAcc(double t, double T, double p0, double pf);

} // namespace math_utils
} // namespace oculus_arm_control

#endif // OCULUS_ARM_CONTROL_MATH_UTILS_H 