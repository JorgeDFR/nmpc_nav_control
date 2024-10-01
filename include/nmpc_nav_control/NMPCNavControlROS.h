#pragma once

#include <memory>

#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/utils.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>

#include <itrci_nav/ParametricPathSet.h>

#include "nmpc_nav_control/NMPCNavControl.h"

namespace nmpc_nav_control {

class NMPCNavControlROS {
    private:
        ros::NodeHandle nh_;
        ros::NodeHandle nh_priv_;

        tf2_ros::Buffer *tf_buffer_;
        tf2_ros::TransformListener *tf_listener_;

        ros::Subscriber sub_goal_pose_;
        ros::Publisher pub_cmd_vel_;

        std::unique_ptr<NMPCNavControl> mpc_control_;

        std::string global_frame_id_;
        std::string base_frame_id_;
        int control_freq_;
        double transform_timeout_;

        std::list<NMPCNavControl::Pose> goal_pose_array_;
        NMPCNavControl::Pose robot_pose_;
        NMPCNavControl::CmdVel robot_vel_ref_;
        double cpu_time_;

    public:
        NMPCNavControlROS();
        ~NMPCNavControlROS() = default;

    private:
        void readParam();

        void goalPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
        void pubCmdVel();
        bool getRobotPose();

        void mainLoop();
};

} // namespace nmpc_nav_control