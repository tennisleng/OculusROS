#include "oculus_arm_control/arm_controller.h"

namespace oculus_arm_control {

ArmController::ArmController() : 
    private_nh_("~"), 
    tf_listener_(tf_buffer_) {
    
    // Load parameters
    private_nh_.param<std::string>("arm_base_frame", arm_base_frame_, "arm_base_link");
    private_nh_.param<std::string>("arm_ee_frame", arm_ee_frame_, "arm_ee_link");
    private_nh_.param("max_velocity", max_velocity_, 0.1);
    
    // Initialize subscribers
    target_pose_sub_ = nh_.subscribe("/arm_controller/target_pose", 
                                    10, 
                                    &ArmController::targetPoseCallback, 
                                    this);
    
    gripper_command_sub_ = nh_.subscribe("/arm_controller/gripper_command", 
                                        10, 
                                        &ArmController::gripperCommandCallback, 
                                        this);
    
    // Initialize publishers
    // These would connect to the actual robot driver or MoveIt
    arm_command_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/robot/cartesian_pose_command", 10);
    gripper_state_pub_ = nh_.advertise<std_msgs::Bool>("/robot/gripper_command", 10);
    
    ROS_INFO("Arm controller node initialized");
}

void ArmController::targetPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    // Check that the target pose is in a valid frame
    if (msg->header.frame_id != arm_base_frame_) {
        geometry_msgs::PoseStamped transformed_pose;
        try {
            tf_buffer_.transform(*msg, transformed_pose, arm_base_frame_);
            arm_command_pub_.publish(transformed_pose);
        } catch (tf2::TransformException &ex) {
            ROS_WARN("Transform failed: %s", ex.what());
            return;
        }
    } else {
        arm_command_pub_.publish(*msg);
    }
}

void ArmController::gripperCommandCallback(const std_msgs::Bool::ConstPtr& msg) {
    // Forward the gripper command to the robot
    gripper_state_pub_.publish(*msg);
}

} // namespace oculus_arm_control

int main(int argc, char** argv) {
    ros::init(argc, argv, "arm_controller_node");
    oculus_arm_control::ArmController arm_controller;
    ros::spin();
    return 0;
} 