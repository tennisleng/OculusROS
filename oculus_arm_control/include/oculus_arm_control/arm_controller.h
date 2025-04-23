#ifndef ARM_CONTROLLER_H
#define ARM_CONTROLLER_H

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Bool.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace oculus_arm_control {

class ArmController {
private:
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    
    // Subscribers
    ros::Subscriber target_pose_sub_;
    ros::Subscriber gripper_command_sub_;
    
    // Publishers
    ros::Publisher arm_command_pub_;
    ros::Publisher gripper_state_pub_;
    
    // TF
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    
    // Parameters
    std::string arm_base_frame_;
    std::string arm_ee_frame_;
    double max_velocity_;

public:
    ArmController();
    void targetPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void gripperCommandCallback(const std_msgs::Bool::ConstPtr& msg);
};

} // namespace oculus_arm_control

#endif // ARM_CONTROLLER_H 