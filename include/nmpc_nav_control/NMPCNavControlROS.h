#ifndef NMPC_NAV_CONTROL_ROS_H
#define NMPC_NAV_CONTROL_ROS_H

#include <memory>
#include <list>

#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/utils.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>

#include <itrci_nav/ParametricPathSet.h>
#include <parametric_trajectories_common/path_types_conversions.h>

#include "nmpc_nav_control/NMPCNavControl.h"
#include "nmpc_nav_control/PathDiscretizer.h"

namespace nmpc_nav_control {

using TPathList = std::list<parametric_trajectories_common::TPath>;

class NMPCNavControlROS {
    private:
        ros::NodeHandle nh_;
        ros::NodeHandle nh_priv_;

        std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
        std::unique_ptr<tf2_ros::TransformListener> tf_listener_;

        ros::Subscriber sub_goal_pose_;
        ros::Subscriber sub_path_no_stack_up_;
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

        TPathList active_path_;
        TPathList upcoming_path_;
        double max_active_path_length_;
        double active_path_u_;

        enum class Mode {
            Idle,
            GoToPose,
            FollowPath
        };
        Mode current_mode_ = Mode::Idle;

    public:
        NMPCNavControlROS();
        ~NMPCNavControlROS() = default;

    private:
        void readParam();

        void goalPoseReceivedCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
        void pathNoStackUpReceivedCallback(const itrci_nav::ParametricPathSet::ConstPtr& msg);
        void pubCmdVel();
        bool getRobotPose();

        void mainLoop();

        // Other topics that can receive paths
        // void pathNoStackUpReceivedCallback(const itrci_nav::ParametricPathSet::ConstPtr& msg); // Topic: 'PathNoStackUp'
        // void pathNoStackUpReceived2Callback(const itrci_nav::ParametricPathSet2::ConstPtr& msg); // Topic: 'PathNoStackUp2'
        // void pathWithEndOffsetReceivedCallback(const itrci_nav::ParametricPathSetWithEndOffset::ConstPtr &msg); // Topic: 'PathWithEndOffset'
        
        void processPathBuffers(const double active_path_u);
        void processNearestPoint();
        double calcDistToEnd();
};

} // namespace nmpc_nav_control

#endif // NMPC_NAV_CONTROL_ROS_H