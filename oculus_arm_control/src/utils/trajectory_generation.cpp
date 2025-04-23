#include "oculus_arm_control/utils/math_utils.h"
#include <random>
#include <algorithm>

namespace oculus_arm_control {
namespace math_utils {

std::vector<TrajectoryPoint> generateTrapezoidal(double start, double goal, 
                                               const TrajectoryParams& params) {
    double distance = std::abs(goal - start);
    int direction = (goal >= start) ? 1 : -1;
    
    // Calculate time for acceleration and deceleration phases
    // For a trapezoidal profile, we accelerate, then maintain constant velocity, then decelerate
    double max_vel = params.max_velocity;
    double max_acc = params.max_acceleration;
    
    // Time to reach max velocity
    double t_acc = max_vel / max_acc;
    
    // Distance covered during acceleration and deceleration
    double d_acc = 0.5 * max_acc * t_acc * t_acc;
    double d_acc_dec = 2 * d_acc;
    
    // Check if we can reach max velocity
    bool can_reach_max_vel = (distance >= d_acc_dec);
    
    // Recalculate if we can't reach max velocity (triangular profile)
    if (!can_reach_max_vel) {
        // For triangular profile, calculate peak velocity
        max_vel = std::sqrt(distance * max_acc);
        t_acc = max_vel / max_acc;
        d_acc = 0.5 * max_acc * t_acc * t_acc;
        d_acc_dec = 2 * d_acc;
    }
    
    // Time at constant velocity
    double d_const = distance - d_acc_dec;
    double t_const = d_const / max_vel;
    
    // Total time
    double total_time = 2 * t_acc + t_const;
    
    // Generate trajectory points
    std::vector<TrajectoryPoint> trajectory;
    double t = 0;
    
    while (t <= total_time) {
        TrajectoryPoint point;
        point.time = t;
        
        // Acceleration phase
        if (t <= t_acc) {
            point.acceleration = direction * max_acc;
            point.velocity = direction * max_acc * t;
            point.position = start + direction * (0.5 * max_acc * t * t);
        }
        // Constant velocity phase
        else if (t <= t_acc + t_const) {
            point.acceleration = 0;
            point.velocity = direction * max_vel;
            point.position = start + direction * (d_acc + max_vel * (t - t_acc));
        }
        // Deceleration phase
        else {
            double t_dec = t - (t_acc + t_const);
            point.acceleration = -direction * max_acc;
            point.velocity = direction * (max_vel - max_acc * t_dec);
            point.position = start + direction * (d_acc + d_const + max_vel * t_dec - 0.5 * max_acc * t_dec * t_dec);
        }
        
        trajectory.push_back(point);
        t += params.time_step;
    }
    
    // Ensure the last point is exactly at the goal
    if (!trajectory.empty()) {
        trajectory.back().position = goal;
        trajectory.back().velocity = 0;
        trajectory.back().acceleration = 0;
    }
    
    return trajectory;
}

std::vector<TrajectoryPoint> generateMinimumJerk(double start, double goal, 
                                              double duration,
                                              double time_step) {
    std::vector<TrajectoryPoint> trajectory;
    
    // Minimum-jerk polynomial coefficients
    double a0 = start;
    double a1 = 0;  // zero initial velocity
    double a2 = 0;  // zero initial acceleration
    double a3 = 10 * (goal - start) / (duration * duration * duration);
    double a4 = -15 * (goal - start) / (duration * duration * duration * duration);
    double a5 = 6 * (goal - start) / (duration * duration * duration * duration * duration);
    
    double t = 0;
    while (t <= duration) {
        TrajectoryPoint point;
        point.time = t;
        
        // Position: a0 + a1*t + a2*t^2 + a3*t^3 + a4*t^4 + a5*t^5
        point.position = a0 + a1*t + a2*t*t + a3*t*t*t + a4*t*t*t*t + a5*t*t*t*t*t;
        
        // Velocity: a1 + 2*a2*t + 3*a3*t^2 + 4*a4*t^3 + 5*a5*t^4
        point.velocity = a1 + 2*a2*t + 3*a3*t*t + 4*a4*t*t*t + 5*a5*t*t*t*t;
        
        // Acceleration: 2*a2 + 6*a3*t + 12*a4*t^2 + 20*a5*t^3
        point.acceleration = 2*a2 + 6*a3*t + 12*a4*t*t + 20*a5*t*t*t;
        
        trajectory.push_back(point);
        t += time_step;
    }
    
    // Ensure the last point is exactly at the goal
    if (!trajectory.empty()) {
        trajectory.back().position = goal;
        trajectory.back().velocity = 0;
        trajectory.back().acceleration = 0;
    }
    
    return trajectory;
}

// RRT path planning algorithm
struct RRTNode {
    Eigen::Vector3d position;
    int parent_idx;
};

// Helper function for RRT
double distance(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2) {
    return (p2 - p1).norm();
}

// Helper function to check if a point is in collision
bool isColliding(const Eigen::Vector3d& point, 
                const std::vector<Eigen::Vector3d>& obstacles,
                double min_distance) {
    for (const auto& obstacle : obstacles) {
        if ((obstacle - point).norm() < min_distance) {
            return true;
        }
    }
    return false;
}

// Helper function to check if a path segment is in collision
bool isPathColliding(const Eigen::Vector3d& start, 
                    const Eigen::Vector3d& end,
                    const std::vector<Eigen::Vector3d>& obstacles,
                    double min_distance,
                    double step_size) {
    double path_length = distance(start, end);
    int steps = std::max(2, static_cast<int>(std::ceil(path_length / step_size)));
    
    for (int i = 0; i <= steps; ++i) {
        double t = static_cast<double>(i) / steps;
        Eigen::Vector3d point = start + t * (end - start);
        
        if (isColliding(point, obstacles, min_distance)) {
            return true;
        }
    }
    
    return false;
}

std::vector<Eigen::Vector3d> generateRRT(const Eigen::Vector3d& start,
                                       const Eigen::Vector3d& goal,
                                       const std::vector<Eigen::Vector3d>& obstacles,
                                       double step_size,
                                       int max_iterations) {
    // Setup RRT
    std::vector<RRTNode> tree;
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> rand_x(-1.0, 1.0); // Adjust range as needed
    std::uniform_real_distribution<> rand_y(-1.0, 1.0);
    std::uniform_real_distribution<> rand_z(0.0, 1.0);
    
    // Add the start node to the tree
    RRTNode start_node;
    start_node.position = start;
    start_node.parent_idx = -1; // No parent
    tree.push_back(start_node);
    
    double min_obstacle_distance = 0.1; // Minimum distance to obstacles
    double goal_bias = 0.1; // Probability of sampling the goal directly
    
    for (int i = 0; i < max_iterations; ++i) {
        // Sample a random point or goal with some probability
        Eigen::Vector3d random_point;
        if (std::uniform_real_distribution<>(0.0, 1.0)(gen) < goal_bias) {
            random_point = goal;
        } else {
            random_point = Eigen::Vector3d(rand_x(gen), rand_y(gen), rand_z(gen));
        }
        
        // Find the nearest node in the tree
        int nearest_idx = 0;
        double min_dist = std::numeric_limits<double>::max();
        
        for (size_t j = 0; j < tree.size(); ++j) {
            double dist = distance(tree[j].position, random_point);
            if (dist < min_dist) {
                min_dist = dist;
                nearest_idx = j;
            }
        }
        
        // Create a new node in the direction of the random point
        Eigen::Vector3d direction = (random_point - tree[nearest_idx].position).normalized();
        Eigen::Vector3d new_position = tree[nearest_idx].position + direction * step_size;
        
        // Check if the new position is in collision with obstacles
        if (isColliding(new_position, obstacles, min_obstacle_distance)) {
            continue; // Skip if in collision
        }
        
        // Check if the path to the new position is in collision
        if (isPathColliding(tree[nearest_idx].position, new_position, obstacles, min_obstacle_distance, step_size / 10.0)) {
            continue; // Skip if path is in collision
        }
        
        // Add the new node to the tree
        RRTNode new_node;
        new_node.position = new_position;
        new_node.parent_idx = nearest_idx;
        tree.push_back(new_node);
        
        // Check if we've reached the goal
        if (distance(new_position, goal) < step_size) {
            // Construct the path by traversing the tree backwards
            std::vector<Eigen::Vector3d> path;
            int current_idx = tree.size() - 1;
            
            while (current_idx != -1) {
                path.push_back(tree[current_idx].position);
                current_idx = tree[current_idx].parent_idx;
            }
            
            // Reverse the path to get it from start to goal
            std::reverse(path.begin(), path.end());
            
            // Add the exact goal point if not already there
            if (distance(path.back(), goal) > 1e-6) {
                path.push_back(goal);
            }
            
            return path;
        }
    }
    
    // If we get here, we couldn't find a path within the max iterations
    return std::vector<Eigen::Vector3d>();
}

} // namespace math_utils
} // namespace oculus_arm_control 