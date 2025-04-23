#ifndef OCULUS_INTERFACE_H
#define OCULUS_INTERFACE_H

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Bool.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>

namespace oculus_arm_control {

class OculusInterface {
private:
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    
    // Subscribers for Oculus data
    ros::Subscriber left_controller_pose_sub_;
    ros::Subscriber right_controller_pose_sub_;
    ros::Subscriber button_sub_;
    
    // Publishers for arm control
    ros::Publisher target_pose_pub_;
    ros::Publisher gripper_command_pub_;
    
    // Parameters
    bool use_right_controller_;
    double scale_factor_;
    bool is_gripper_closed_;

public:
    OculusInterface();
    void leftControllerCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void rightControllerCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void buttonCallback(const sensor_msgs::Joy::ConstPtr& msg);
    void publishTargetPose(const geometry_msgs::PoseStamped::ConstPtr& controller_pose);
};

} // namespace oculus_arm_control

#endif // OCULUS_INTERFACE_H 