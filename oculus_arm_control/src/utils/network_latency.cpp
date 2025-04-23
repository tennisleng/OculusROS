#include "oculus_arm_control/utils/network_latency.h"
#include <numeric>
#include <sstream>
#include <iomanip>
#include <algorithm>
#include <ctime>

namespace oculus_arm_control {
namespace network {

LatencyMonitor::LatencyMonitor(ros::NodeHandle& nh, ros::NodeHandle& private_nh)
    : nh_(nh), 
      private_nh_(private_nh),
      running_(false),
      sequence_number_(0),
      total_pings_sent_(0),
      total_pongs_received_(0),
      last_latency_(0.0),
      log_to_file_(false),
      monitor_vr_latency_(false) {
    
    // Load parameters
    private_nh_.param("ping_interval", ping_interval_, 0.1);         // 10 Hz default
    private_nh_.param("window_size", window_size_, 100);             // Average over 100 samples
    private_nh_.param("timeout_threshold", timeout_threshold_, 1.0); // 1 second timeout
    private_nh_.param("log_to_console", log_to_console_, false);     // Detailed console logging off by default
    
    // Initialize publishers and subscribers for basic latency monitoring
    ping_pub_ = nh_.advertise<std_msgs::String>("/latency/ping", 10);
    pong_sub_ = nh_.subscribe("/latency/pong", 10, &LatencyMonitor::pongCallback, this);
    latency_pub_ = nh_.advertise<std_msgs::Float64>("/latency/measurement", 10);
    
    // Additional metrics publishers
    jitter_pub_ = nh_.advertise<std_msgs::Float64>("/latency/jitter", 10);
    packet_loss_pub_ = nh_.advertise<std_msgs::Float64>("/latency/packet_loss", 10);
    quality_score_pub_ = nh_.advertise<std_msgs::UInt32>("/latency/quality_score", 10);
    
    ROS_INFO("LatencyMonitor initialized with ping interval: %.2f s, window size: %d samples",
             ping_interval_, window_size_);
}

LatencyMonitor::~LatencyMonitor() {
    stop();
    stopLogging();
}

bool LatencyMonitor::initializeRosBridge(const std::string& websocket_url) {
    // Note: In a real implementation, this would establish a connection to rosbridge
    // For this example, we'll just log the attempt and assume success
    ROS_INFO("Connecting to rosbridge at: %s", websocket_url.c_str());
    
    // Here you would typically:
    // 1. Establish the websocket connection
    // 2. Set up message handlers
    // 3. Verify connection is established
    
    return true;  // Assume connection successful
}

void LatencyMonitor::start() {
    if (running_) {
        ROS_WARN("LatencyMonitor is already running");
        return;
    }
    
    // Clear previous measurements
    {
        std::lock_guard<std::mutex> lock(latency_mutex_);
        latency_samples_.clear();
        jitter_samples_.clear();
        pending_pings_.clear();
        recent_responses_.clear();
        
        // Initialize tracking variables
        total_pings_sent_ = 0;
        total_pongs_received_ = 0;
        sequence_number_ = 0;
        last_latency_ = 0.0;
    }
    
    // Start the ping timer
    ping_timer_ = nh_.createTimer(
        ros::Duration(ping_interval_),
        &LatencyMonitor::pingTimerCallback,
        this
    );
    
    // Start the timeout checking timer
    timeout_timer_ = nh_.createTimer(
        ros::Duration(timeout_threshold_ / 2.0), // Check for timeouts at twice the frequency
        &LatencyMonitor::timeoutCheckCallback,
        this
    );
    
    running_ = true;
    ROS_INFO("LatencyMonitor started");
}

void LatencyMonitor::stop() {
    if (!running_) {
        return;
    }
    
    // Stop the timers
    ping_timer_.stop();
    timeout_timer_.stop();
    running_ = false;
    
    ROS_INFO("LatencyMonitor stopped");
    printStatistics();
}

double LatencyMonitor::getAverageLatency() const {
    std::lock_guard<std::mutex> lock(latency_mutex_);
    
    if (latency_samples_.empty()) {
        return 0.0;
    }
    
    double sum = std::accumulate(latency_samples_.begin(), latency_samples_.end(), 0.0);
    return sum / latency_samples_.size();
}

double LatencyMonitor::getLatestLatency() const {
    std::lock_guard<std::mutex> lock(latency_mutex_);
    
    if (latency_samples_.empty()) {
        return 0.0;
    }
    
    return latency_samples_.back();
}

LatencyStats LatencyMonitor::getStatistics() const {
    std::lock_guard<std::mutex> lock(latency_mutex_);
    
    LatencyStats stats;
    stats.mean = 0.0;
    stats.min = 0.0;
    stats.max = 0.0;
    stats.stddev = 0.0;
    stats.jitter = 0.0;
    stats.loss = 0.0;
    
    if (latency_samples_.empty()) {
        return stats;
    }
    
    // Calculate statistics
    double sum = std::accumulate(latency_samples_.begin(), latency_samples_.end(), 0.0);
    stats.mean = sum / latency_samples_.size();
    
    // Find min and max
    stats.min = *std::min_element(latency_samples_.begin(), latency_samples_.end());
    stats.max = *std::max_element(latency_samples_.begin(), latency_samples_.end());
    
    // Calculate standard deviation
    double sq_sum = std::inner_product(
        latency_samples_.begin(), latency_samples_.end(),
        latency_samples_.begin(), 0.0,
        std::plus<double>(),
        [&stats](double x, double y) { return (x - stats.mean) * (y - stats.mean); }
    );
    stats.stddev = std::sqrt(sq_sum / latency_samples_.size());
    
    // Calculate jitter
    stats.jitter = calculateJitter();
    
    // Calculate packet loss
    stats.loss = getPacketLossRate();
    
    return stats;
}

void LatencyMonitor::printStatistics() const {
    LatencyStats stats = getStatistics();
    
    if (latency_samples_.empty()) {
        ROS_INFO("No latency samples collected yet");
        return;
    }
    
    ROS_INFO("Latency statistics:");
    ROS_INFO("  Samples: %zu", latency_samples_.size());
    ROS_INFO("  Mean:    %.2f ms", stats.mean);
    ROS_INFO("  Min:     %.2f ms", stats.min);
    ROS_INFO("  Max:     %.2f ms", stats.max);
    ROS_INFO("  StdDev:  %.2f ms", stats.stddev);
    ROS_INFO("  Jitter:  %.2f ms", stats.jitter);
    ROS_INFO("  Loss:    %.2f%%", stats.loss * 100.0);
    ROS_INFO("  Quality: %d/5", getNetworkQualityScore());
    
    if (monitor_vr_latency_) {
        std::lock_guard<std::mutex> lock(vr_latency_mutex_);
        if (!vr_to_arm_latency_samples_.empty()) {
            double vr_sum = std::accumulate(vr_to_arm_latency_samples_.begin(), vr_to_arm_latency_samples_.end(), 0.0);
            double vr_mean = vr_sum / vr_to_arm_latency_samples_.size();
            ROS_INFO("  VR-to-Arm Latency: %.2f ms", vr_mean);
        } else {
            ROS_INFO("  VR-to-Arm Latency: No samples yet");
        }
    }
}

bool LatencyMonitor::startLogging(const std::string& filename) {
    if (log_to_file_) {
        stopLogging();
    }
    
    log_filename_ = filename;
    log_file_.open(filename, std::ios::out | std::ios::trunc);
    
    if (!log_file_.is_open()) {
        ROS_ERROR("Failed to open log file: %s", filename.c_str());
        return false;
    }
    
    // Write header
    log_file_ << "timestamp,sequence,latency_ms,is_timeout,packet_loss,jitter" << std::endl;
    log_to_file_ = true;
    
    ROS_INFO("Latency logging started to file: %s", filename.c_str());
    return true;
}

void LatencyMonitor::stopLogging() {
    if (log_to_file_ && log_file_.is_open()) {
        log_file_.close();
        log_to_file_ = false;
        ROS_INFO("Latency logging stopped");
    }
}

double LatencyMonitor::getPacketLossRate() const {
    if (total_pings_sent_ == 0) {
        return 0.0;
    }
    
    // Use sliding window if we have enough data
    if (!recent_responses_.empty()) {
        int lost_count = std::count(recent_responses_.begin(), recent_responses_.end(), false);
        return static_cast<double>(lost_count) / recent_responses_.size();
    }
    
    // Otherwise use total count
    return 1.0 - (static_cast<double>(total_pongs_received_) / total_pings_sent_);
}

int LatencyMonitor::getNetworkQualityScore() const {
    LatencyStats stats = getStatistics();
    
    // Simplified scoring based on latency, jitter and loss
    // In a production system, this would be more sophisticated
    
    if (stats.loss > 0.2) return 0; // >20% loss is critical
    if (stats.mean > 300) return 1; // Very poor latency
    if (stats.loss > 0.1) return 1; // >10% loss is very poor
    
    if (stats.mean > 150) return 2; // Poor latency
    if (stats.jitter > 50) return 2; // High jitter
    if (stats.loss > 0.05) return 2; // 5-10% loss is poor
    
    if (stats.mean > 100) return 3; // Moderate latency
    if (stats.jitter > 20) return 3; // Moderate jitter
    if (stats.loss > 0.01) return 3; // 1-5% loss is moderate
    
    if (stats.mean > 50) return 4; // Good latency
    if (stats.jitter > 10) return 4; // Some jitter
    if (stats.loss > 0.001) return 4; // <1% loss is good
    
    return 5; // Excellent quality
}

void LatencyMonitor::enableVRLatencyMonitoring(bool enable) {
    if (enable == monitor_vr_latency_) {
        return;  // Already in the requested state
    }
    
    monitor_vr_latency_ = enable;
    
    if (enable) {
        // Subscribe to VR controller pose and arm status
        vr_pose_sub_ = nh_.subscribe("/oculus/right_controller/pose", 10, 
                                     &LatencyMonitor::vrPoseCallback, this);
        arm_status_sub_ = nh_.subscribe("/arm_controller/status", 10,
                                       &LatencyMonitor::armStatusCallback, this);
        vr_to_arm_latency_pub_ = nh_.advertise<std_msgs::Float64>("/latency/vr_to_arm", 10);
        
        ROS_INFO("VR-to-Arm latency monitoring enabled");
    } else {
        // Unsubscribe
        vr_pose_sub_.shutdown();
        arm_status_sub_.shutdown();
        vr_to_arm_latency_pub_.shutdown();
        
        // Clear samples
        std::lock_guard<std::mutex> lock(vr_latency_mutex_);
        vr_to_arm_latency_samples_.clear();
        
        ROS_INFO("VR-to-Arm latency monitoring disabled");
    }
}

void LatencyMonitor::pongCallback(const std_msgs::String::ConstPtr& msg) {
    // Extract the sequence number and timestamp from the message
    std::istringstream iss(msg->data);
    int seq;
    double send_time;
    
    if (!(iss >> seq >> send_time)) {
        ROS_WARN("Received malformed pong message: %s", msg->data.c_str());
        return;
    }
    
    // Calculate latency
    ros::Time now = ros::Time::now();
    double latency_ms = (now.toSec() - send_time) * 1000.0;
    
    // Update tracking
    {
        std::lock_guard<std::mutex> lock(latency_mutex_);
        
        // Record that we received a response
        total_pongs_received_++;
        
        // Remove from pending pings
        pending_pings_.erase(seq);
        
        // Add to recent responses (true = received)
        recent_responses_.push_back(true);
        if (recent_responses_.size() > window_size_) {
            recent_responses_.pop_front();
        }
    }
    
    // Record and publish the latency
    addLatencySample(latency_ms);
    publishLatency(latency_ms);
    calculateAndPublishJitter();
    calculateAndPublishPacketLoss();
    calculateAndPublishQualityScore();
    
    // Log to file if enabled
    if (log_to_file_) {
        logLatencyToFile(latency_ms, seq, false);
    }
    
    if (log_to_console_ || seq % 100 == 0) {
        ROS_INFO("Ping-pong latency: %.2f ms (seq: %d)", latency_ms, seq);
    }
}

void LatencyMonitor::pingTimerCallback(const ros::TimerEvent& event) {
    // Create a ping message with sequence number and current time
    std_msgs::String msg;
    std::ostringstream oss;
    ros::Time now = ros::Time::now();
    oss << sequence_number_ << " " << now.toSec();
    msg.data = oss.str();
    
    // Record the ping in the pending map
    {
        std::lock_guard<std::mutex> lock(latency_mutex_);
        pending_pings_[sequence_number_] = now;
        total_pings_sent_++;
    }
    
    // Publish the ping
    ping_pub_.publish(msg);
    
    // Increment sequence number
    sequence_number_++;
}

void LatencyMonitor::timeoutCheckCallback(const ros::TimerEvent& event) {
    checkForTimeouts();
}

void LatencyMonitor::vrPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    // Record the time we received a VR controller pose
    last_vr_command_time_ = msg->header.stamp;
    
    if (log_to_console_) {
        ROS_DEBUG("Received VR pose at time: %.3f", last_vr_command_time_.toSec());
    }
}

void LatencyMonitor::armStatusCallback(const std_msgs::Bool::ConstPtr& msg) {
    // This simulates receiving a status message from the arm after movement
    if (last_vr_command_time_.isZero()) {
        return;  // No VR command received yet
    }
    
    ros::Time now = ros::Time::now();
    double latency_ms = (now - last_vr_command_time_).toSec() * 1000.0;
    
    // Add to samples
    {
        std::lock_guard<std::mutex> lock(vr_latency_mutex_);
        vr_to_arm_latency_samples_.push_back(latency_ms);
        
        // Keep limited history
        if (vr_to_arm_latency_samples_.size() > window_size_) {
            vr_to_arm_latency_samples_.erase(vr_to_arm_latency_samples_.begin());
        }
    }
    
    // Publish
    std_msgs::Float64 latency_msg;
    latency_msg.data = latency_ms;
    vr_to_arm_latency_pub_.publish(latency_msg);
    
    if (log_to_console_) {
        ROS_DEBUG("VR-to-Arm latency: %.2f ms", latency_ms);
    }
}

void LatencyMonitor::addLatencySample(double latency_ms) {
    std::lock_guard<std::mutex> lock(latency_mutex_);
    
    // Calculate jitter from the last sample
    if (!latency_samples_.empty()) {
        double jitter = std::abs(latency_ms - last_latency_);
        jitter_samples_.push_back(jitter);
        
        // Keep only the most recent window_size_ jitter samples
        if (jitter_samples_.size() > window_size_) {
            jitter_samples_.erase(jitter_samples_.begin());
        }
    }
    
    // Add the latency sample
    latency_samples_.push_back(latency_ms);
    
    // Update last latency
    last_latency_ = latency_ms;
    
    // Keep only the most recent window_size_ samples
    if (latency_samples_.size() > window_size_) {
        latency_samples_.erase(latency_samples_.begin());
    }
}

void LatencyMonitor::publishLatency(double latency_ms) {
    std_msgs::Float64 msg;
    msg.data = latency_ms;
    latency_pub_.publish(msg);
}

void LatencyMonitor::logLatencyToFile(double latency_ms, int seq, bool is_timeout) {
    if (!log_to_file_ || !log_file_.is_open()) {
        return;
    }
    
    // Get current time for timestamp
    std::time_t t = std::time(nullptr);
    std::tm tm = *std::localtime(&t);
    
    log_file_ << std::put_time(&tm, "%Y-%m-%d %H:%M:%S") << ",";
    log_file_ << seq << ",";
    log_file_ << std::fixed << std::setprecision(2) << latency_ms << ",";
    log_file_ << (is_timeout ? "1" : "0") << ",";
    log_file_ << std::fixed << std::setprecision(4) << getPacketLossRate() << ",";
    log_file_ << std::fixed << std::setprecision(2) << calculateJitter();
    log_file_ << std::endl;
}

void LatencyMonitor::calculateAndPublishJitter() {
    double jitter = calculateJitter();
    
    std_msgs::Float64 msg;
    msg.data = jitter;
    jitter_pub_.publish(msg);
}

void LatencyMonitor::calculateAndPublishPacketLoss() {
    double loss_rate = getPacketLossRate();
    
    std_msgs::Float64 msg;
    msg.data = loss_rate;
    packet_loss_pub_.publish(msg);
}

void LatencyMonitor::calculateAndPublishQualityScore() {
    int score = getNetworkQualityScore();
    
    std_msgs::UInt32 msg;
    msg.data = score;
    quality_score_pub_.publish(msg);
}

void LatencyMonitor::checkForTimeouts() {
    std::vector<int> timedout_seqs;
    ros::Time now = ros::Time::now();
    
    // Check for timeouts
    {
        std::lock_guard<std::mutex> lock(latency_mutex_);
        
        for (auto& ping : pending_pings_) {
            if ((now - ping.second).toSec() >= timeout_threshold_) {
                timedout_seqs.push_back(ping.first);
                
                // Add to recent responses (false = timeout)
                recent_responses_.push_back(false);
                if (recent_responses_.size() > window_size_) {
                    recent_responses_.pop_front();
                }
            }
        }
        
        // Remove timed out pings from pending map
        for (int seq : timedout_seqs) {
            pending_pings_.erase(seq);
        }
    }
    
    // Log timeouts
    for (int seq : timedout_seqs) {
        if (log_to_file_) {
            // Use a high latency value for timeouts
            double timeout_latency = timeout_threshold_ * 1000.0;
            logLatencyToFile(timeout_latency, seq, true);
        }
        
        if (log_to_console_) {
            ROS_WARN("Ping timed out (seq: %d)", seq);
        }
    }
    
    // Update metrics if we had timeouts
    if (!timedout_seqs.empty()) {
        calculateAndPublishPacketLoss();
        calculateAndPublishQualityScore();
    }
}

double LatencyMonitor::calculateJitter() const {
    std::lock_guard<std::mutex> lock(latency_mutex_);
    
    if (jitter_samples_.empty()) {
        return 0.0;
    }
    
    double sum = std::accumulate(jitter_samples_.begin(), jitter_samples_.end(), 0.0);
    return sum / jitter_samples_.size();
}

} // namespace network
} // namespace oculus_arm_control 