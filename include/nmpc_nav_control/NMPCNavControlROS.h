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
#include <std_msgs/String.h>

#include <itrci_nav/ParametricPathSet.h>
#include <itrci_nav/ParametricPathSet2.h>
#include <itrci_nav/parametric_trajectories_control_status.h>
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
        ros::Subscriber sub_path_no_stack_up_2_;
        ros::Subscriber sub_control_command_;
        ros::Publisher pub_cmd_vel_;
        ros::Publisher pub_control_status_;
        ros::Publisher pub_actual_path_;

        ros::Timer main_timer_;

        std::unique_ptr<NMPCNavControl> mpc_control_;

        std::string global_frame_id_;
        std::string base_frame_id_;
        int control_freq_;
        double transform_timeout_;

        enum class Status {
            Idle,
            GoToPose,
            FollowPath,
            Break,
            Error
        };
        Status current_status_ = Status::Idle;

        NMPCNavControl::Pose robot_pose_;
        NMPCNavControl::CmdVel robot_vel_ref_;

        geometry_msgs::PoseStamped goal_pose_;

        TPathList active_path_;
        TPathList upcoming_path_;
        double max_active_path_length_;
        double active_path_u_;
        unsigned int path_request_id_;

    public:
        NMPCNavControlROS();
        ~NMPCNavControlROS() = default;

    private:
        void readParam();

        void goalPoseReceivedCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
        void pathNoStackUpReceivedCallback(const itrci_nav::ParametricPathSet::ConstPtr& msg);
        void pathNoStackUp2ReceivedCallback(const itrci_nav::ParametricPathSet2::ConstPtr& msg);
        void controlCommandReceivedCallback(const std_msgs::String::ConstPtr& msg);
        void pubCmdVel();
        void pubControlStatus();
        void pubActualPath();
        void getRobotPose();

        void mainTimerCallBack(const ros::TimerEvent&);
        void mainCycle();

        // Other topics that can receive paths
        // void pathNoStackUpReceivedCallback(const itrci_nav::ParametricPathSet::ConstPtr& msg); // Topic: 'PathNoStackUp'
        // void pathWithEndOffsetReceivedCallback(const itrci_nav::ParametricPathSetWithEndOffset::ConstPtr &msg); // Topic: 'PathWithEndOffset'
        
        void processPathReceived(const itrci_nav::ParametricPathSet& path);
        void processPathBuffers(const double active_path_u);
        void processNearestPoint();
        void processBreak();
        void processGoToPose();
        void processFollowPath();
        void executeNMPC(const std::list<NMPCNavControl::Pose>& goal_pose_array);

        double calcDistToEnd();
};

} // namespace nmpc_nav_control

#endif // NMPC_NAV_CONTROL_ROS_H