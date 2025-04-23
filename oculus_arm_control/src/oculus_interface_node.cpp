#include "oculus_arm_control/oculus_interface.h"

namespace oculus_arm_control {

OculusInterface::OculusInterface() : private_nh_("~"), is_gripper_closed_(false) {
    // Load parameters
    private_nh_.param("use_right_controller", use_right_controller_, true);
    private_nh_.param("scale_factor", scale_factor_, 0.5);
    
    // Initialize subscribers
    left_controller_pose_sub_ = nh_.subscribe("/oculus/left_controller/pose", 
                                             10, 
                                             &OculusInterface::leftControllerCallback, 
                                             this);
    
    right_controller_pose_sub_ = nh_.subscribe("/oculus/right_controller/pose", 
                                              10, 
                                              &OculusInterface::rightControllerCallback, 
                                              this);
    
    button_sub_ = nh_.subscribe("/oculus/button_states", 
                               10, 
                               &OculusInterface::buttonCallback, 
                               this);
    
    // Initialize publishers
    target_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/arm_controller/target_pose", 10);
    gripper_command_pub_ = nh_.advertise<std_msgs::Bool>("/arm_controller/gripper_command", 10);
    
    ROS_INFO("Oculus interface node initialized");
}

void OculusInterface::leftControllerCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    if (!use_right_controller_) {
        publishTargetPose(msg);
    }
}

void OculusInterface::rightControllerCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    if (use_right_controller_) {
        publishTargetPose(msg);
    }
}

void OculusInterface::buttonCallback(const sensor_msgs::Joy::ConstPtr& msg) {
    // Assuming trigger button (index 1) controls the gripper
    if (msg->buttons[1] == 1) {
        std_msgs::Bool gripper_cmd;
        is_gripper_closed_ = !is_gripper_closed_;
        gripper_cmd.data = is_gripper_closed_;
        gripper_command_pub_.publish(gripper_cmd);
    }
}

void OculusInterface::publishTargetPose(const geometry_msgs::PoseStamped::ConstPtr& controller_pose) {
    geometry_msgs::PoseStamped target_pose = *controller_pose;
    
    // Scale the position based on the scale factor
    target_pose.pose.position.x *= scale_factor_;
    target_pose.pose.position.y *= scale_factor_;
    target_pose.pose.position.z *= scale_factor_;
    
    // Change frame if needed
    target_pose.header.frame_id = "arm_base_link";
    target_pose.header.stamp = ros::Time::now();
    
    target_pose_pub_.publish(target_pose);
}

} // namespace oculus_arm_control

int main(int argc, char** argv) {
    ros::init(argc, argv, "oculus_interface_node");
    oculus_arm_control::OculusInterface oculus_interface;
    ros::spin();
    return 0;
} 