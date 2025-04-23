#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/UInt32.h>
#include <boost/program_options.hpp>
#include <signal.h>
#include "oculus_arm_control/utils/network_latency.h"

namespace po = boost::program_options;

// Global latency monitor for signal handler
oculus_arm_control::network::LatencyMonitor* g_latency_monitor = nullptr;

// Signal handler for clean shutdown
void signalHandler(int sig) {
    ROS_INFO("Received shutdown signal, stopping latency monitor...");
    if (g_latency_monitor) {
        g_latency_monitor->stop();
    }
    ros::shutdown();
}

int main(int argc, char** argv) {
    // Initialize ROS node
    ros::init(argc, argv, "network_latency_node", ros::init_options::NoSigintHandler);
    
    // Register signal handler
    signal(SIGINT, signalHandler);
    
    // Create node handles
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    
    // Parse command line options
    po::options_description desc("Allowed options");
    desc.add_options()
        ("help", "produce help message")
        ("rosbridge_url", po::value<std::string>(), "rosbridge WebSocket URL")
        ("ping_interval", po::value<double>(), "interval between pings in seconds")
        ("window_size", po::value<int>(), "number of samples to use for statistics")
        ("timeout", po::value<double>(), "timeout threshold in seconds")
        ("log_file", po::value<std::string>(), "log file to write latency data")
        ("vr_monitor", po::value<bool>(), "enable VR-specific latency monitoring")
        ("verbose", po::value<bool>(), "enable verbose console logging");
    
    po::variables_map vm;
    try {
        po::store(po::parse_command_line(argc, argv, desc), vm);
        po::notify(vm);
    } catch (const std::exception& e) {
        ROS_ERROR("Error parsing command line: %s", e.what());
        std::cout << desc << "\n";
        return 1;
    }
    
    if (vm.count("help")) {
        std::cout << desc << "\n";
        return 0;
    }
    
    // Create latency monitor
    oculus_arm_control::network::LatencyMonitor latency_monitor(nh, private_nh);
    g_latency_monitor = &latency_monitor;
    
    // Set parameters from command line if provided
    if (vm.count("ping_interval")) {
        private_nh.setParam("ping_interval", vm["ping_interval"].as<double>());
    }
    
    if (vm.count("window_size")) {
        private_nh.setParam("window_size", vm["window_size"].as<int>());
    }
    
    if (vm.count("timeout")) {
        private_nh.setParam("timeout_threshold", vm["timeout"].as<double>());
    }
    
    if (vm.count("verbose")) {
        private_nh.setParam("log_to_console", vm["verbose"].as<bool>());
    }
    
    // Get rosbridge URL
    std::string rosbridge_url;
    if (vm.count("rosbridge_url")) {
        rosbridge_url = vm["rosbridge_url"].as<std::string>();
    } else {
        private_nh.param<std::string>("rosbridge_url", rosbridge_url, "ws://localhost:9090");
    }
    
    // Initialize rosbridge connection
    if (!latency_monitor.initializeRosBridge(rosbridge_url)) {
        ROS_ERROR("Failed to connect to rosbridge at %s", rosbridge_url.c_str());
        return 1;
    }
    
    // Enable VR latency monitoring if requested
    bool monitor_vr = false;
    if (vm.count("vr_monitor")) {
        monitor_vr = vm["vr_monitor"].as<bool>();
    } else {
        private_nh.param("monitor_vr_latency", monitor_vr, false);
    }
    
    if (monitor_vr) {
        latency_monitor.enableVRLatencyMonitoring(true);
    }
    
    // Start data logging if a log file was specified
    if (vm.count("log_file")) {
        std::string log_file = vm["log_file"].as<std::string>();
        if (!latency_monitor.startLogging(log_file)) {
            ROS_WARN("Failed to start logging to file: %s", log_file.c_str());
        }
    }
    
    // Start monitoring latency
    latency_monitor.start();
    
    // Set up a timer to periodically print statistics
    double stats_interval;
    private_nh.param("stats_interval", stats_interval, 5.0);  // Print stats every 5 seconds by default
    
    ros::Timer stats_timer = nh.createTimer(
        ros::Duration(stats_interval),
        [&latency_monitor](const ros::TimerEvent&) {
            latency_monitor.printStatistics();
        }
    );
    
    // Set up a timer to publish network quality assessment
    ros::Timer quality_timer = nh.createTimer(
        ros::Duration(1.0), // Publish network quality every second
        [&latency_monitor](const ros::TimerEvent&) {
            // We could do additional processing here if needed
            // Currently the quality score is calculated and published in the latency monitor
        }
    );
    
    ROS_INFO("Network latency node started");
    ROS_INFO("Monitoring latency with rosbridge at: %s", rosbridge_url.c_str());
    
    // Spin to process callbacks
    ros::spin();
    
    // Stop monitoring before shutdown
    latency_monitor.stop();
    
    return 0;
} 