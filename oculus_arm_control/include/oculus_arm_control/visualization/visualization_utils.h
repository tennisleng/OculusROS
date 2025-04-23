#ifndef OCULUS_ARM_CONTROL_VISUALIZATION_UTILS_H
#define OCULUS_ARM_CONTROL_VISUALIZATION_UTILS_H

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <std_msgs/ColorRGBA.h>
#include <tf2/LinearMath/Quaternion.h>
#include <Eigen/Dense>
#include <vector>
#include <string>

#include "oculus_arm_control/config.h"

namespace oculus_arm_control {
namespace visualization {

// Helper struct for marker creation
struct MarkerConfig {
    visualization::MarkerType type;
    std::string frame_id;
    std::string ns;
    int id;
    std::vector<float> color;  // RGBA
    std::vector<float> scale;  // XYZ
    double lifetime;  // seconds, 0 for permanent
    bool frame_locked;
};

// Class to handle visualization of various elements
class VisualizationManager {
public:
    VisualizationManager(ros::NodeHandle& nh);
    
    // Visualize the VR controller as a marker
    void visualizeController(const geometry_msgs::Pose& pose, 
                           bool is_left,
                           bool is_active = true);
    
    // Visualize the robot workspace as a box
    void visualizeWorkspace(const std::vector<double>& workspace_limits,
                          const std::string& frame_id);
    
    // Visualize a trajectory as a line strip
    void visualizeTrajectory(const std::vector<geometry_msgs::Pose>& trajectory,
                           const std::string& frame_id,
                           const std::string& ns = "trajectory");
    
    // Visualize end effector position
    void visualizeEndEffector(const geometry_msgs::Pose& pose,
                            const std::string& frame_id);
    
    // Visualize a coordinate frame
    void visualizeFrame(const geometry_msgs::Pose& pose,
                      const std::string& frame_id,
                      const std::string& name = "coordinate_frame",
                      double scale = 0.1);
    
    // Visualize collisions or safety warnings
    void visualizeSafetyWarning(const geometry_msgs::Pose& pose,
                              const std::string& frame_id,
                              const std::string& warning_text = "Warning");
    
    // Visualize a text message at a specified pose
    void visualizeText(const std::string& text,
                     const geometry_msgs::Pose& pose,
                     const std::string& frame_id,
                     const std::string& ns = "text",
                     const std::vector<float>& color = visualization::WARNING_COLOR);
    
    // Clear all markers in a namespace
    void clearMarkers(const std::string& ns = "");
    
    // Create a standard marker with common properties
    visualization_msgs::Marker createMarker(const MarkerConfig& config);
    
    // Create a standard color
    std_msgs::ColorRGBA createColor(float r, float g, float b, float a = 1.0);
    
private:
    ros::NodeHandle nh_;
    ros::Publisher marker_pub_;
    ros::Publisher marker_array_pub_;
    
    // Counter for marker IDs
    int marker_id_counter_;
    
    // Helper functions for marker creation
    visualization_msgs::Marker createSphereMarker(const geometry_msgs::Pose& pose,
                                                const std::string& frame_id,
                                                const std::string& ns,
                                                const std::vector<float>& color,
                                                float radius);
                                                
    visualization_msgs::Marker createCubeMarker(const geometry_msgs::Pose& pose,
                                             const std::string& frame_id,
                                             const std::string& ns,
                                             const std::vector<float>& color,
                                             const std::vector<float>& dimensions);
                                             
    visualization_msgs::Marker createArrowMarker(const geometry_msgs::Pose& start_pose,
                                              const geometry_msgs::Pose& end_pose,
                                              const std::string& frame_id,
                                              const std::string& ns,
                                              const std::vector<float>& color,
                                              float shaft_diameter,
                                              float head_diameter);
                                              
    visualization_msgs::Marker createLineStripMarker(const std::vector<geometry_msgs::Point>& points,
                                                  const std::string& frame_id,
                                                  const std::string& ns,
                                                  const std::vector<float>& color,
                                                  float width);
                                                  
    visualization_msgs::Marker createTextMarker(const std::string& text,
                                             const geometry_msgs::Pose& pose,
                                             const std::string& frame_id,
                                             const std::string& ns,
                                             const std::vector<float>& color,
                                             float height);
                                             
    visualization_msgs::Marker createMeshMarker(const geometry_msgs::Pose& pose,
                                             const std::string& frame_id,
                                             const std::string& ns,
                                             const std::vector<float>& color,
                                             const std::string& mesh_resource,
                                             const std::vector<float>& scale);
};

// Helper function to create a pose from position and orientation
geometry_msgs::Pose createPose(double x, double y, double z,
                            double qx, double qy, double qz, double qw);

// Helper function to create a pose from position and RPY angles
geometry_msgs::Pose createPoseFromRPY(double x, double y, double z,
                                   double roll, double pitch, double yaw);

// Convert a 3D point to a geometry_msgs::Point
geometry_msgs::Point toPoint(double x, double y, double z);

// Convert an Eigen vector to a geometry_msgs::Point
geometry_msgs::Point eigenToPoint(const Eigen::Vector3d& vec);

} // namespace visualization
} // namespace oculus_arm_control

#endif // OCULUS_ARM_CONTROL_VISUALIZATION_UTILS_H 