#ifndef OCULUS_ARM_CONTROL_TRAJECTORY_CONTROLLER_H
#define OCULUS_ARM_CONTROL_TRAJECTORY_CONTROLLER_H

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <nav_msgs/Path.h>
#include <vector>
#include <deque>
#include <string>
#include <mutex>
#include <Eigen/Dense>

#include "oculus_arm_control/config.h"
#include "oculus_arm_control/utils/math_utils.h"

namespace oculus_arm_control {
namespace control {

enum class TrajectoryExecutionState {
    IDLE,
    EXECUTING,
    PAUSED,
    COMPLETED,
    ERROR
};

enum class TrajectoryType {
    LINEAR,  // Straight line between waypoints
    SPLINE,  // Cubic spline interpolation
    TRAPEZOIDAL,  // Trapezoidal velocity profile
    MINIMUM_JERK  // Minimum jerk trajectory
};

// Structure to hold a trajectory waypoint
struct TrajectoryWaypoint {
    geometry_msgs::Pose pose;
    double time;  // Time to reach this waypoint
    bool is_stop_point;  // Whether to stop at this waypoint
};

// Class for controlling robot trajectories
class TrajectoryController {
public:
    TrajectoryController(ros::NodeHandle& nh);
    
    // Add a waypoint to the trajectory
    void addWaypoint(const geometry_msgs::Pose& waypoint, bool is_stop_point = false);
    
    // Add multiple waypoints to the trajectory
    void addWaypoints(const std::vector<geometry_msgs::Pose>& waypoints);
    
    // Clear all waypoints
    void clearWaypoints();
    
    // Start trajectory execution
    bool startExecution();
    
    // Pause trajectory execution
    void pauseExecution();
    
    // Resume trajectory execution
    void resumeExecution();
    
    // Stop trajectory execution
    void stopExecution();
    
    // Check if the trajectory is being executed
    bool isExecuting() const;
    
    // Update the controller (call this periodically)
    void update(const ros::Time& time);
    
    // Set maximum velocity, acceleration, and jerk
    void setMaxVelocity(double max_velocity);
    void setMaxAcceleration(double max_acceleration);
    void setMaxJerk(double max_jerk);
    
    // Set trajectory type
    void setTrajectoryType(TrajectoryType type);
    
    // Set time between waypoints (for trajectories without specified times)
    void setWaypointTiming(double time_between_waypoints);
    
    // Save and load trajectories
    bool saveTrajectory(const std::string& filename);
    bool loadTrajectory(const std::string& filename);
    
    // Record current pose as a waypoint
    void recordCurrentPose(const geometry_msgs::Pose& current_pose);
    
    // Start and stop trajectory recording
    void startRecording();
    void stopRecording();
    bool isRecording() const;
    
    // Get trajectory information
    std::vector<TrajectoryWaypoint> getWaypoints() const;
    TrajectoryExecutionState getExecutionState() const;
    double getTrajectoryProgress() const;  // 0.0 to 1.0
    
    // Get current commanded pose
    geometry_msgs::Pose getCurrentCommandedPose() const;
    
    // Get the entire trajectory path
    nav_msgs::Path getTrajectoryPath() const;
    
private:
    ros::NodeHandle nh_;
    
    // Publishers and subscribers
    ros::Publisher pose_pub_;
    ros::Publisher path_pub_;
    ros::Publisher state_pub_;
    
    // Trajectory parameters
    std::vector<TrajectoryWaypoint> waypoints_;
    TrajectoryType trajectory_type_;
    TrajectoryExecutionState execution_state_;
    
    // Execution parameters
    double max_velocity_;
    double max_acceleration_;
    double max_jerk_;
    double time_between_waypoints_;
    
    // Current execution state
    size_t current_waypoint_index_;
    double trajectory_time_;
    double trajectory_duration_;
    ros::Time last_update_time_;
    
    // Recording state
    bool is_recording_;
    double recording_interval_;
    ros::Time last_record_time_;
    
    // Mutex for thread safety
    mutable std::mutex waypoints_mutex_;
    
    // Internal methods
    void generateTrajectoryPath();
    void publishTrajectoryPath();
    void publishExecutionState();
    geometry_msgs::Pose interpolateWaypoints(size_t from_index, size_t to_index, double t);
    double calculateTrajectoryDuration();
    void computeWaypointTiming();
    
    // Trajectory generation methods
    std::vector<geometry_msgs::Pose> generateLinearTrajectory(const geometry_msgs::Pose& start, const geometry_msgs::Pose& end, int steps);
    std::vector<geometry_msgs::Pose> generateSplineTrajectory(const std::vector<geometry_msgs::Pose>& control_points, int steps_per_segment);
    std::vector<geometry_msgs::Pose> generateMinimumJerkTrajectory(const geometry_msgs::Pose& start, const geometry_msgs::Pose& end, double duration, double time_step);
};

} // namespace control
} // namespace oculus_arm_control

#endif // OCULUS_ARM_CONTROL_TRAJECTORY_CONTROLLER_H 