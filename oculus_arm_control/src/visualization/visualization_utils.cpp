#include "oculus_arm_control/visualization/visualization_utils.h"
#include "oculus_arm_control/utils/math_utils.h"
#include <tf2/LinearMath/Quaternion.h>

namespace oculus_arm_control {
namespace visualization {

VisualizationManager::VisualizationManager(ros::NodeHandle& nh)
    : nh_(nh), marker_id_counter_(0) {
    marker_pub_ = nh_.advertise<visualization_msgs::Marker>(topics::VIZ_MARKER, 10);
    marker_array_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(topics::VIZ_MARKER + "_array", 10);
}

// Helper function to create a pose from position and orientation
geometry_msgs::Pose createPose(double x, double y, double z,
                            double qx, double qy, double qz, double qw) {
    geometry_msgs::Pose pose;
    pose.position.x = x;
    pose.position.y = y;
    pose.position.z = z;
    pose.orientation.x = qx;
    pose.orientation.y = qy;
    pose.orientation.z = qz;
    pose.orientation.w = qw;
    return pose;
}

// Helper function to create a pose from position and RPY angles
geometry_msgs::Pose createPoseFromRPY(double x, double y, double z,
                                   double roll, double pitch, double yaw) {
    geometry_msgs::Pose pose;
    pose.position.x = x;
    pose.position.y = y;
    pose.position.z = z;
    
    tf2::Quaternion q;
    q.setRPY(roll, pitch, yaw);
    pose.orientation.x = q.x();
    pose.orientation.y = q.y();
    pose.orientation.z = q.z();
    pose.orientation.w = q.w();
    
    return pose;
}

// Convert a 3D point to a geometry_msgs::Point
geometry_msgs::Point toPoint(double x, double y, double z) {
    geometry_msgs::Point point;
    point.x = x;
    point.y = y;
    point.z = z;
    return point;
}

// Convert an Eigen vector to a geometry_msgs::Point
geometry_msgs::Point eigenToPoint(const Eigen::Vector3d& vec) {
    return toPoint(vec.x(), vec.y(), vec.z());
}

// Create a standard color
std_msgs::ColorRGBA VisualizationManager::createColor(float r, float g, float b, float a) {
    std_msgs::ColorRGBA color;
    color.r = r;
    color.g = g;
    color.b = b;
    color.a = a;
    return color;
}

// Create a standard marker with common properties
visualization_msgs::Marker VisualizationManager::createMarker(const MarkerConfig& config) {
    visualization_msgs::Marker marker;
    
    // Set the frame ID and timestamp
    marker.header.frame_id = config.frame_id;
    marker.header.stamp = ros::Time::now();
    
    // Set the namespace and ID
    marker.ns = config.ns;
    marker.id = config.id;
    
    // Set the marker type
    switch (config.type) {
        case visualization::SPHERE:
            marker.type = visualization_msgs::Marker::SPHERE;
            break;
        case visualization::CUBE:
            marker.type = visualization_msgs::Marker::CUBE;
            break;
        case visualization::CYLINDER:
            marker.type = visualization_msgs::Marker::CYLINDER;
            break;
        case visualization::ARROW:
            marker.type = visualization_msgs::Marker::ARROW;
            break;
        case visualization::TEXT:
            marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
            break;
        case visualization::MESH:
            marker.type = visualization_msgs::Marker::MESH_RESOURCE;
            break;
        default:
            marker.type = visualization_msgs::Marker::SPHERE;
    }
    
    // Set the marker action to add/modify
    marker.action = visualization_msgs::Marker::ADD;
    
    // Set the scale
    marker.scale.x = config.scale[0];
    marker.scale.y = config.scale.size() > 1 ? config.scale[1] : config.scale[0];
    marker.scale.z = config.scale.size() > 2 ? config.scale[2] : config.scale[0];
    
    // Set the color
    marker.color.r = config.color[0];
    marker.color.g = config.color[1];
    marker.color.b = config.color[2];
    marker.color.a = config.color.size() > 3 ? config.color[3] : 1.0;
    
    // Set the lifetime (0 means forever)
    marker.lifetime = ros::Duration(config.lifetime);
    
    // Set whether the marker should be frame-locked
    marker.frame_locked = config.frame_locked;
    
    return marker;
}

// Create specific marker types
visualization_msgs::Marker VisualizationManager::createSphereMarker(
    const geometry_msgs::Pose& pose,
    const std::string& frame_id,
    const std::string& ns,
    const std::vector<float>& color,
    float radius) {
    
    MarkerConfig config;
    config.type = visualization::SPHERE;
    config.frame_id = frame_id;
    config.ns = ns;
    config.id = marker_id_counter_++;
    config.color = color;
    config.scale = {radius * 2.0f, radius * 2.0f, radius * 2.0f}; // Diameter
    config.lifetime = 0.0; // Forever
    config.frame_locked = false;
    
    visualization_msgs::Marker marker = createMarker(config);
    marker.pose = pose;
    
    return marker;
}

visualization_msgs::Marker VisualizationManager::createCubeMarker(
    const geometry_msgs::Pose& pose,
    const std::string& frame_id,
    const std::string& ns,
    const std::vector<float>& color,
    const std::vector<float>& dimensions) {
    
    MarkerConfig config;
    config.type = visualization::CUBE;
    config.frame_id = frame_id;
    config.ns = ns;
    config.id = marker_id_counter_++;
    config.color = color;
    config.scale = dimensions;
    config.lifetime = 0.0; // Forever
    config.frame_locked = false;
    
    visualization_msgs::Marker marker = createMarker(config);
    marker.pose = pose;
    
    return marker;
}

visualization_msgs::Marker VisualizationManager::createArrowMarker(
    const geometry_msgs::Pose& start_pose,
    const geometry_msgs::Pose& end_pose,
    const std::string& frame_id,
    const std::string& ns,
    const std::vector<float>& color,
    float shaft_diameter,
    float head_diameter) {
    
    MarkerConfig config;
    config.type = visualization::ARROW;
    config.frame_id = frame_id;
    config.ns = ns;
    config.id = marker_id_counter_++;
    config.color = color;
    config.scale = {shaft_diameter, head_diameter, 0.1f}; // x: shaft diameter, y: head diameter, z: head length
    config.lifetime = 0.0; // Forever
    config.frame_locked = false;
    
    visualization_msgs::Marker marker = createMarker(config);
    
    // For an arrow, we need to set the start and end points
    marker.points.push_back(start_pose.position);
    marker.points.push_back(end_pose.position);
    
    return marker;
}

visualization_msgs::Marker VisualizationManager::createLineStripMarker(
    const std::vector<geometry_msgs::Point>& points,
    const std::string& frame_id,
    const std::string& ns,
    const std::vector<float>& color,
    float width) {
    
    MarkerConfig config;
    config.type = visualization::ARROW; // We'll override this
    config.frame_id = frame_id;
    config.ns = ns;
    config.id = marker_id_counter_++;
    config.color = color;
    config.scale = {width, 0.0f, 0.0f}; // Only x is used for line width
    config.lifetime = 0.0; // Forever
    config.frame_locked = false;
    
    visualization_msgs::Marker marker = createMarker(config);
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.points = points;
    
    return marker;
}

visualization_msgs::Marker VisualizationManager::createTextMarker(
    const std::string& text,
    const geometry_msgs::Pose& pose,
    const std::string& frame_id,
    const std::string& ns,
    const std::vector<float>& color,
    float height) {
    
    MarkerConfig config;
    config.type = visualization::TEXT;
    config.frame_id = frame_id;
    config.ns = ns;
    config.id = marker_id_counter_++;
    config.color = color;
    config.scale = {0.0f, 0.0f, height}; // Only z is used for text height
    config.lifetime = 0.0; // Forever
    config.frame_locked = true; // Text is usually frame-locked for readability
    
    visualization_msgs::Marker marker = createMarker(config);
    marker.pose = pose;
    marker.text = text;
    
    return marker;
}

visualization_msgs::Marker VisualizationManager::createMeshMarker(
    const geometry_msgs::Pose& pose,
    const std::string& frame_id,
    const std::string& ns,
    const std::vector<float>& color,
    const std::string& mesh_resource,
    const std::vector<float>& scale) {
    
    MarkerConfig config;
    config.type = visualization::MESH;
    config.frame_id = frame_id;
    config.ns = ns;
    config.id = marker_id_counter_++;
    config.color = color;
    config.scale = scale;
    config.lifetime = 0.0; // Forever
    config.frame_locked = false;
    
    visualization_msgs::Marker marker = createMarker(config);
    marker.pose = pose;
    marker.mesh_resource = mesh_resource;
    marker.mesh_use_embedded_materials = true;
    
    return marker;
}

// Implementation of visualization methods
void VisualizationManager::visualizeController(const geometry_msgs::Pose& pose, 
                                           bool is_left,
                                           bool is_active) {
    std::string ns = is_left ? "left_controller" : "right_controller";
    std::vector<float> color = CONTROLLER_COLOR;
    
    // If the controller is inactive, make it more transparent
    if (!is_active) {
        color[3] = 0.3f; // Reduce alpha
    }
    
    // Create a sphere for the controller position
    visualization_msgs::Marker sphere = createSphereMarker(
        pose, 
        frames::WORLD_FRAME, 
        ns, 
        color, 
        CONTROLLER_MARKER_SCALE
    );
    
    marker_pub_.publish(sphere);
    
    // Create a coordinate frame to show the orientation
    visualizeFrame(pose, frames::WORLD_FRAME, ns + "_frame", 0.05);
}

void VisualizationManager::visualizeWorkspace(const std::vector<double>& workspace_limits,
                                          const std::string& frame_id) {
    if (workspace_limits.size() < 6) {
        ROS_WARN("Workspace limits vector must have at least 6 elements (xmin, xmax, ymin, ymax, zmin, zmax)");
        return;
    }
    
    double x_min = workspace_limits[0];
    double x_max = workspace_limits[1];
    double y_min = workspace_limits[2];
    double y_max = workspace_limits[3];
    double z_min = workspace_limits[4];
    double z_max = workspace_limits[5];
    
    // Calculate the center and dimensions of the workspace
    double x_center = (x_max + x_min) / 2.0;
    double y_center = (y_max + y_min) / 2.0;
    double z_center = (z_max + z_min) / 2.0;
    
    double x_size = x_max - x_min;
    double y_size = y_max - y_min;
    double z_size = z_max - z_min;
    
    // Create a pose for the center of the workspace
    geometry_msgs::Pose center_pose = createPose(
        x_center, y_center, z_center,
        0.0, 0.0, 0.0, 1.0
    );
    
    // Create a semi-transparent box to represent the workspace
    visualization_msgs::Marker box = createCubeMarker(
        center_pose,
        frame_id,
        "workspace",
        WORKSPACE_COLOR,
        {static_cast<float>(x_size), static_cast<float>(y_size), static_cast<float>(z_size)}
    );
    
    marker_pub_.publish(box);
}

void VisualizationManager::visualizeTrajectory(const std::vector<geometry_msgs::Pose>& trajectory,
                                          const std::string& frame_id,
                                          const std::string& ns) {
    if (trajectory.empty()) {
        return;
    }
    
    // Convert the trajectory poses to points
    std::vector<geometry_msgs::Point> points;
    for (const auto& pose : trajectory) {
        points.push_back(pose.position);
    }
    
    // Create a line strip marker for the trajectory
    visualization_msgs::Marker line = createLineStripMarker(
        points,
        frame_id,
        ns,
        TRAJECTORY_COLOR,
        TRAJECTORY_MARKER_SCALE
    );
    
    marker_pub_.publish(line);
    
    // Add a sphere at the end of the trajectory
    visualization_msgs::Marker end_point = createSphereMarker(
        trajectory.back(),
        frame_id,
        ns + "_end",
        TRAJECTORY_COLOR,
        TRAJECTORY_MARKER_SCALE * 2.0f
    );
    
    marker_pub_.publish(end_point);
}

void VisualizationManager::visualizeEndEffector(const geometry_msgs::Pose& pose,
                                           const std::string& frame_id) {
    // Create a sphere at the end effector position
    visualization_msgs::Marker sphere = createSphereMarker(
        pose,
        frame_id,
        "end_effector",
        {0.0f, 1.0f, 0.0f, 0.8f}, // Green, semi-transparent
        CONTROLLER_MARKER_SCALE
    );
    
    marker_pub_.publish(sphere);
    
    // Also visualize the end effector frame
    visualizeFrame(pose, frame_id, "end_effector_frame", 0.05);
}

void VisualizationManager::visualizeFrame(const geometry_msgs::Pose& pose,
                                      const std::string& frame_id,
                                      const std::string& name,
                                      double scale) {
    visualization_msgs::MarkerArray marker_array;
    
    // Create three arrows for the x, y, and z axes
    Eigen::Isometry3d pose_eigen = math_utils::poseToEigen(pose);
    
    // X-axis (Red)
    Eigen::Vector3d x_axis = pose_eigen.rotation().col(0);
    Eigen::Vector3d origin = pose_eigen.translation();
    Eigen::Vector3d x_end = origin + scale * x_axis;
    
    geometry_msgs::Pose origin_pose = pose;
    geometry_msgs::Pose x_end_pose = math_utils::eigenToPose(
        Eigen::Translation3d(x_end) * pose_eigen.rotation()
    );
    
    visualization_msgs::Marker x_arrow = createArrowMarker(
        origin_pose,
        x_end_pose,
        frame_id,
        name + "_x",
        {1.0f, 0.0f, 0.0f, 1.0f}, // Red
        scale * 0.1,
        scale * 0.2
    );
    
    // Y-axis (Green)
    Eigen::Vector3d y_axis = pose_eigen.rotation().col(1);
    Eigen::Vector3d y_end = origin + scale * y_axis;
    
    geometry_msgs::Pose y_end_pose = math_utils::eigenToPose(
        Eigen::Translation3d(y_end) * pose_eigen.rotation()
    );
    
    visualization_msgs::Marker y_arrow = createArrowMarker(
        origin_pose,
        y_end_pose,
        frame_id,
        name + "_y",
        {0.0f, 1.0f, 0.0f, 1.0f}, // Green
        scale * 0.1,
        scale * 0.2
    );
    
    // Z-axis (Blue)
    Eigen::Vector3d z_axis = pose_eigen.rotation().col(2);
    Eigen::Vector3d z_end = origin + scale * z_axis;
    
    geometry_msgs::Pose z_end_pose = math_utils::eigenToPose(
        Eigen::Translation3d(z_end) * pose_eigen.rotation()
    );
    
    visualization_msgs::Marker z_arrow = createArrowMarker(
        origin_pose,
        z_end_pose,
        frame_id,
        name + "_z",
        {0.0f, 0.0f, 1.0f, 1.0f}, // Blue
        scale * 0.1,
        scale * 0.2
    );
    
    marker_array.markers.push_back(x_arrow);
    marker_array.markers.push_back(y_arrow);
    marker_array.markers.push_back(z_arrow);
    
    marker_array_pub_.publish(marker_array);
}

void VisualizationManager::visualizeSafetyWarning(const geometry_msgs::Pose& pose,
                                             const std::string& frame_id,
                                             const std::string& warning_text) {
    // Create a red sphere at the warning location
    visualization_msgs::Marker sphere = createSphereMarker(
        pose,
        frame_id,
        "safety_warning",
        WARNING_COLOR,
        CONTROLLER_MARKER_SCALE * 1.5f
    );
    
    marker_pub_.publish(sphere);
    
    // Create text marker with the warning message
    if (!warning_text.empty()) {
        // Offset the text slightly above the warning sphere
        geometry_msgs::Pose text_pose = pose;
        text_pose.position.z += CONTROLLER_MARKER_SCALE * 2.0f;
        
        visualizeText(warning_text, text_pose, frame_id, "safety_warning_text", WARNING_COLOR);
    }
}

void VisualizationManager::visualizeText(const std::string& text,
                                     const geometry_msgs::Pose& pose,
                                     const std::string& frame_id,
                                     const std::string& ns,
                                     const std::vector<float>& color) {
    visualization_msgs::Marker text_marker = createTextMarker(
        text,
        pose,
        frame_id,
        ns,
        color,
        TEXT_MARKER_SCALE
    );
    
    marker_pub_.publish(text_marker);
}

void VisualizationManager::clearMarkers(const std::string& ns) {
    visualization_msgs::Marker delete_marker;
    delete_marker.action = visualization_msgs::Marker::DELETEALL;
    
    if (!ns.empty()) {
        delete_marker.ns = ns;
    }
    
    marker_pub_.publish(delete_marker);
}

} // namespace visualization
} // namespace oculus_arm_control 