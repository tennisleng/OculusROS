#pragma once

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <std_msgs/UInt32.h>
#include <geometry_msgs/PoseStamped.h>
#include <chrono>
#include <vector>
#include <mutex>
#include <fstream>
#include <map>
#include <deque>

namespace oculus_arm_control {
namespace network {

/**
 * Struct for storing latency statistics
 */
struct LatencyStats {
    double mean;    // Mean latency in ms
    double min;     // Minimum latency in ms
    double max;     // Maximum latency in ms
    double stddev;  // Standard deviation in ms
    double jitter;  // Jitter (variance in latency) in ms
    double loss;    // Packet loss rate (0.0-1.0)
};

/**
 * Class for measuring and monitoring network latency in ROS communication
 * between the Oculus VR controllers and the robotic arm
 */
class LatencyMonitor {
public:
    /**
     * Constructor
     * @param nh ROS node handle
     * @param private_nh Private ROS node handle
     */
    LatencyMonitor(ros::NodeHandle& nh, ros::NodeHandle& private_nh);

    /**
     * Destructor
     */
    ~LatencyMonitor();

    /**
     * Initialize rosbridge connection
     * @param websocket_url The URL of the rosbridge websocket server
     * @return True if connection successful, false otherwise
     */
    bool initializeRosBridge(const std::string& websocket_url);

    /**
     * Start latency measurements
     */
    void start();

    /**
     * Stop latency measurements
     */
    void stop();
    
    /**
     * Get current average latency
     * @return Average latency in milliseconds
     */
    double getAverageLatency() const;

    /**
     * Get latest latency 
     * @return Latest latency measurement in milliseconds
     */
    double getLatestLatency() const;

    /**
     * Print latency statistics to ROS info
     */
    void printStatistics() const;

    /**
     * Get comprehensive statistics
     * @return Struct containing all latency statistics
     */
    LatencyStats getStatistics() const;

    /**
     * Start data logging to file
     * @param filename Path to save the log file
     * @return True if logging started successfully
     */
    bool startLogging(const std::string& filename);

    /**
     * Stop data logging
     */
    void stopLogging();

    /**
     * Get packet loss rate
     * @return Packet loss rate as a percentage (0.0-1.0)
     */
    double getPacketLossRate() const;

    /**
     * Get current network quality assessment
     * @return Quality score from 0 (worst) to 5 (best)
     */
    int getNetworkQualityScore() const;

    /**
     * Monitor VR-specific latency between Oculus and arm controller
     * @param enable Whether to enable VR-specific monitoring
     */
    void enableVRLatencyMonitoring(bool enable);

private:
    // Node handles
    ros::NodeHandle& nh_;
    ros::NodeHandle& private_nh_;
    
    // Publishers and subscribers
    ros::Publisher ping_pub_;
    ros::Subscriber pong_sub_;
    ros::Publisher latency_pub_;
    ros::Publisher jitter_pub_;
    ros::Publisher packet_loss_pub_;
    ros::Publisher quality_score_pub_;
    
    // VR-specific monitoring
    ros::Subscriber vr_pose_sub_;
    ros::Subscriber arm_status_sub_;
    ros::Publisher vr_to_arm_latency_pub_;
    
    // Parameters
    double ping_interval_;      // Time between pings in seconds
    int window_size_;           // Number of samples to average over
    double timeout_threshold_;  // Time after which a ping is considered lost in seconds
    bool log_to_console_;       // Whether to log detailed stats to console
    bool log_to_file_;          // Whether to log to file
    bool monitor_vr_latency_;   // Whether to monitor VR-specific latency
    
    // Latency tracking
    std::vector<double> latency_samples_;
    std::mutex latency_mutex_;
    ros::Time last_ping_time_;
    bool running_;
    int sequence_number_;
    std::map<int, ros::Time> pending_pings_;
    
    // Packet loss tracking
    int total_pings_sent_;
    int total_pongs_received_;
    std::deque<bool> recent_responses_; // For sliding window loss calculation
    
    // Jitter tracking
    double last_latency_;
    std::vector<double> jitter_samples_;
    
    // VR to arm latency tracking
    ros::Time last_vr_command_time_;
    std::vector<double> vr_to_arm_latency_samples_;
    std::mutex vr_latency_mutex_;
    
    // Timer for sending pings
    ros::Timer ping_timer_;
    ros::Timer timeout_timer_;
    
    // File logging
    std::ofstream log_file_;
    std::string log_filename_;
    
    // Callback for ping responses
    void pongCallback(const std_msgs::String::ConstPtr& msg);
    
    // Timer callback to send pings
    void pingTimerCallback(const ros::TimerEvent& event);
    
    // Timer callback to check for timeouts
    void timeoutCheckCallback(const ros::TimerEvent& event);
    
    // VR pose callback
    void vrPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    
    // Arm status callback
    void armStatusCallback(const std_msgs::Bool::ConstPtr& msg);
    
    // Utility functions
    void addLatencySample(double latency_ms);
    void publishLatency(double latency_ms);
    void logLatencyToFile(double latency_ms, int seq, bool is_timeout = false);
    void calculateAndPublishJitter();
    void calculateAndPublishPacketLoss();
    void calculateAndPublishQualityScore();
    void checkForTimeouts();
    double calculateJitter() const;
};

} // namespace network
} // namespace oculus_arm_control 