#ifndef OCULUS_ARM_CONTROL_CONFIG_H
#define OCULUS_ARM_CONTROL_CONFIG_H

#include <string>
#include <vector>
#include <map>
#include <ros/ros.h>

namespace oculus_arm_control {

// Topic names
namespace topics {
    // Oculus topics
    const std::string OCULUS_LEFT_CONTROLLER_POSE = "/oculus/left_controller/pose";
    const std::string OCULUS_RIGHT_CONTROLLER_POSE = "/oculus/right_controller/pose";
    const std::string OCULUS_BUTTON_STATES = "/oculus/button_states";
    const std::string OCULUS_TOUCH_STATES = "/oculus/touch_states";
    const std::string OCULUS_HEAD_POSE = "/oculus/head_pose";
    
    // Arm controller topics
    const std::string ARM_TARGET_POSE = "/arm_controller/target_pose";
    const std::string ARM_GRIPPER_COMMAND = "/arm_controller/gripper_command";
    const std::string ARM_JOINT_STATES = "/arm_controller/joint_states";
    const std::string ARM_CARTESIAN_PATH = "/arm_controller/cartesian_path";
    
    // Robot driver topics
    const std::string ROBOT_CARTESIAN_POSE_COMMAND = "/robot/cartesian_pose_command";
    const std::string ROBOT_GRIPPER_COMMAND = "/robot/gripper_command";
    const std::string ROBOT_JOINT_COMMAND = "/robot/joint_command";
    const std::string ROBOT_JOINT_STATES = "/robot/joint_states";
    
    // Visualization topics
    const std::string VIZ_MARKER = "/visualization/marker";
    const std::string VIZ_TRAJECTORY = "/visualization/trajectory";
    const std::string VIZ_WORKSPACE = "/visualization/workspace";
    const std::string VIZ_CONSTRAINTS = "/visualization/constraints";
    
    // Safety topics
    const std::string SAFETY_COLLISION = "/safety/collision";
    const std::string SAFETY_VELOCITY_LIMIT = "/safety/velocity_limit";
    const std::string SAFETY_JOINT_LIMIT = "/safety/joint_limit";
    const std::string SAFETY_EMERGENCY_STOP = "/safety/emergency_stop";
}

// Frame names
namespace frames {
    const std::string WORLD_FRAME = "world";
    const std::string OCULUS_BASE_FRAME = "oculus_base";
    const std::string ARM_BASE_FRAME = "arm_base_link";
    const std::string ARM_EE_FRAME = "arm_ee_link";
    const std::string LEFT_CONTROLLER_FRAME = "left_controller";
    const std::string RIGHT_CONTROLLER_FRAME = "right_controller";
    const std::string HEAD_FRAME = "head";
    const std::string VIRTUAL_WORKSPACE_FRAME = "virtual_workspace";
}

// Controller button mapping
namespace buttons {
    enum OculusButton {
        A_BUTTON = 0,
        B_BUTTON = 1,
        X_BUTTON = 2,
        Y_BUTTON = 3,
        LEFT_TRIGGER = 4,
        RIGHT_TRIGGER = 5,
        LEFT_GRIP = 6,
        RIGHT_GRIP = 7,
        LEFT_THUMBSTICK_PRESS = 8,
        RIGHT_THUMBSTICK_PRESS = 9,
        LEFT_THUMBSTICK_TOUCH = 10,
        RIGHT_THUMBSTICK_TOUCH = 11
    };
    
    // Button to function mapping
    const std::map<OculusButton, std::string> BUTTON_FUNCTION_MAP = {
        {LEFT_TRIGGER, "toggle_gripper"},
        {RIGHT_TRIGGER, "toggle_control_mode"},
        {A_BUTTON, "reset_position"},
        {B_BUTTON, "toggle_workspace_visualization"},
        {X_BUTTON, "emergency_stop"},
        {Y_BUTTON, "save_waypoint"},
        {LEFT_GRIP, "start_recording_trajectory"},
        {RIGHT_GRIP, "stop_recording_trajectory"},
        {LEFT_THUMBSTICK_PRESS, "toggle_left_thumbstick_mode"},
        {RIGHT_THUMBSTICK_PRESS, "toggle_right_thumbstick_mode"}
    };
}

// Default parameters
namespace params {
    const bool DEFAULT_USE_RIGHT_CONTROLLER = true;
    const double DEFAULT_SCALE_FACTOR = 0.5;
    const double DEFAULT_MAX_VELOCITY = 0.1;
    const double DEFAULT_MAX_ACCELERATION = 0.5;
    const double DEFAULT_MAX_JERK = 1.0;
    const double DEFAULT_GRIPPER_SPEED = 0.5;
    
    const double DEFAULT_POSITION_TOLERANCE = 0.001; // meters
    const double DEFAULT_ORIENTATION_TOLERANCE = 0.01; // radians
    
    const double JOINT_LIMIT_SAFETY_MARGIN = 0.1; // radians
    const double VELOCITY_LIMIT_SAFETY_MARGIN = 0.2; // percentage
    
    const int DEFAULT_CONTROL_RATE = 100; // Hz
    const int DEFAULT_VISUALIZATION_RATE = 30; // Hz
    const int DEFAULT_SAFETY_CHECK_RATE = 50; // Hz
    
    const std::vector<double> DEFAULT_WORKSPACE_LIMITS = {
        -0.5, 0.5,  // X min, max
        -0.5, 0.5,  // Y min, max
        0.0, 0.5    // Z min, max
    };
    
    const std::string DEFAULT_CONTROL_MODE = "position"; // position, velocity, or impedance
}

// Control modes
namespace control_modes {
    enum ControlMode {
        POSITION_CONTROL,
        VELOCITY_CONTROL,
        IMPEDANCE_CONTROL,
        JOINT_CONTROL,
        TRAJECTORY_CONTROL,
        WAYPOINT_CONTROL,
        TELEOPERATION
    };
    
    enum GripperMode {
        BINARY_GRIPPER,
        CONTINUOUS_GRIPPER,
        FORCE_CONTROLLED_GRIPPER
    };
}

// Safety thresholds
namespace safety {
    const double MAX_VELOCITY_THRESHOLD = 0.5; // m/s
    const double MAX_ACCELERATION_THRESHOLD = 1.0; // m/s^2
    const double MIN_OBSTACLE_DISTANCE = 0.1; // meters
    const double EMERGENCY_STOP_DECELERATION = 2.0; // m/s^2
    
    enum SafetyLevel {
        LOW,
        MEDIUM,
        HIGH,
        MAXIMUM
    };
    
    const std::map<SafetyLevel, double> VELOCITY_LIMITS = {
        {LOW, 0.5},
        {MEDIUM, 0.3},
        {HIGH, 0.2},
        {MAXIMUM, 0.1}
    };
}

// Visualization settings
namespace visualization {
    enum MarkerType {
        SPHERE,
        CUBE,
        CYLINDER,
        ARROW,
        TEXT,
        MESH
    };
    
    const float CONTROLLER_MARKER_SCALE = 0.05f;
    const float WORKSPACE_MARKER_SCALE = 0.01f;
    const float TRAJECTORY_MARKER_SCALE = 0.02f;
    const float TEXT_MARKER_SCALE = 0.05f;
    
    const std::vector<float> CONTROLLER_COLOR = {0.2f, 0.5f, 1.0f, 0.8f}; // RGBA
    const std::vector<float> WORKSPACE_COLOR = {0.1f, 0.1f, 0.5f, 0.3f}; // RGBA
    const std::vector<float> TRAJECTORY_COLOR = {1.0f, 0.5f, 0.0f, 1.0f}; // RGBA
    const std::vector<float> WARNING_COLOR = {1.0f, 0.3f, 0.3f, 0.8f}; // RGBA
}

// Error codes
namespace errors {
    enum ErrorCode {
        NO_ERROR = 0,
        CONNECTION_ERROR = 1,
        TRANSFORMATION_ERROR = 2,
        JOINT_LIMIT_ERROR = 3,
        VELOCITY_LIMIT_ERROR = 4,
        WORKSPACE_LIMIT_ERROR = 5,
        COLLISION_ERROR = 6,
        COMMUNICATION_ERROR = 7,
        HARDWARE_ERROR = 8,
        TIMEOUT_ERROR = 9,
        UNKNOWN_ERROR = 100
    };
    
    const std::map<ErrorCode, std::string> ERROR_MESSAGES = {
        {NO_ERROR, "No error"},
        {CONNECTION_ERROR, "Failed to connect to device"},
        {TRANSFORMATION_ERROR, "Failed to transform between frames"},
        {JOINT_LIMIT_ERROR, "Joint limit exceeded"},
        {VELOCITY_LIMIT_ERROR, "Velocity limit exceeded"},
        {WORKSPACE_LIMIT_ERROR, "Workspace limit exceeded"},
        {COLLISION_ERROR, "Collision detected"},
        {COMMUNICATION_ERROR, "Communication error"},
        {HARDWARE_ERROR, "Hardware error"},
        {TIMEOUT_ERROR, "Operation timed out"},
        {UNKNOWN_ERROR, "Unknown error"}
    };
}

} // namespace oculus_arm_control

#endif // OCULUS_ARM_CONTROL_CONFIG_H 