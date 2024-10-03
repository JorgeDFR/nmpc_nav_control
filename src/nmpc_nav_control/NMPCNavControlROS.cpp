#include "nmpc_nav_control/NMPCNavControlROS.h"

namespace nmpc_nav_control {

NMPCNavControlROS::NMPCNavControlROS() : nh_priv_("~")
{
    try {
        readParam();
    } catch (std::exception& ex) {
        ROS_FATAL("Error reading the node parameters (%s)", ex.what());
        ros::shutdown();
        return;
    }
    
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>();
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
    
    pub_cmd_vel_          = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 10);
    sub_goal_pose_        = nh_.subscribe<geometry_msgs::PoseStamped>("pose_goal", 10, 
                            &NMPCNavControlROS::goalPoseReceivedCallback, this);
    sub_path_no_stack_up_ = nh_.subscribe<itrci_nav::ParametricPathSet>("path_no_stack_up", 10, 
                            &NMPCNavControlROS::pathNoStackUpReceivedCallback, this);

    ros::Duration(2.0).sleep(); // Delay to fill tf buffer
    mainLoop();
}

void NMPCNavControlROS::readParam()
{
    nh_priv_.param<std::string>("global_frame_id", global_frame_id_, "map");
    nh_priv_.param<std::string>("base_frame_id", base_frame_id_, "base_footprint");
    nh_priv_.param<int>("control_freq", control_freq_, 40);
    nh_priv_.param<double>("transform_timeout", transform_timeout_, 0.1);
    nh_priv_.param<double>("max_active_path_length_", max_active_path_length_, 5.0);

    double l1, l2, l1_plus_l2, dt;
    nh_priv_.param<double>("rob_dist_between_front_back_wh", l1, 0.265);
    nh_priv_.param<double>("rob_dist_between_left_right_wh", l2, 0.270);
    l1_plus_l2 = l1 + l2;
    dt = 1.0 / double(control_freq_);

    ROS_INFO("[%s] Global frame ID: %s", ros::this_node::getName().c_str(), global_frame_id_.c_str());
    ROS_INFO("[%s] Base frame ID: %s", ros::this_node::getName().c_str(), base_frame_id_.c_str());
    ROS_INFO("[%s] Controller frequency: %d Hz", ros::this_node::getName().c_str(), control_freq_);
    ROS_INFO("[%s] Transform timeout: %.02f s", ros::this_node::getName().c_str(), transform_timeout_);
    ROS_INFO("[%s] Maximum active path length: %.02f M", ros::this_node::getName().c_str(), max_active_path_length_);
    ROS_INFO("[%s] Robot distances (l1 + l2): %lf m", ros::this_node::getName().c_str(), l1_plus_l2);

    try {
        mpc_control_ = std::make_unique<NMPCNavControl>(l1_plus_l2, dt);
    } catch (const std::exception& e) {
        ROS_ERROR("[%s] Exception caught: %s. Terminating node.", ros::this_node::getName().c_str(), e.what());
        ros::shutdown();
    }
}

void NMPCNavControlROS::goalPoseReceivedCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    current_mode_ = Mode::GoToPose;
    
    NMPCNavControl::Pose goal_pose;
    // TODO: Check msg->header.frame_id
    goal_pose.x = msg->pose.position.x;
    goal_pose.y = msg->pose.position.y;
    goal_pose.theta = tf2::getYaw(msg->pose.orientation);

    goal_pose_array_.clear();
    goal_pose_array_.push_back(goal_pose);
}

void NMPCNavControlROS::pathNoStackUpReceivedCallback(const itrci_nav::ParametricPathSet::ConstPtr& msg)
{
    current_mode_ = Mode::FollowPath;

    if (msg->PathSet.empty()) { 
        ROS_WARN("Received an empty Path, ignoring...");
        return;
    }
    parametric_trajectories_common::TPathSetRosDecode path_set_ros_decode;
    TPathList path_set = path_set_ros_decode.fromRos(*msg);
		
    upcoming_path_.clear();
    active_path_.clear();
    for (TPathList::const_iterator it = path_set.begin(); it != path_set.end(); it++) {
        if (it->GetFrameId() == "") { continue; }
        upcoming_path_.push_back(*it);
        upcoming_path_.back().SetPathLength(1000);
    }
    processPathBuffers(0.0);
}

void NMPCNavControlROS::pubCmdVel() 
{
    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = robot_vel_ref_.v;
    cmd_vel.linear.y = robot_vel_ref_.vn;
    cmd_vel.angular.z = robot_vel_ref_.w;
    pub_cmd_vel_.publish(cmd_vel);
}

bool NMPCNavControlROS::getRobotPose() 
{
	ros::Time currentTime = ros::Time::now();
    bool result;
    try {
        geometry_msgs::TransformStamped robot_pose;
        robot_pose = tf_buffer_->lookupTransform(global_frame_id_, base_frame_id_, ros::Time(0));
        ros::Time time_stamp = robot_pose.header.stamp;
        
        robot_pose_.x = robot_pose.transform.translation.x;
        robot_pose_.y = robot_pose.transform.translation.y;
        robot_pose_.theta = tf2::getYaw(robot_pose.transform.rotation);

        ros::Duration deltaTime = currentTime - time_stamp;
        if (deltaTime.toSec() > transform_timeout_) {
            ROS_WARN("[%s] Robot pose data error. Data age = %f s", ros::this_node::getName().c_str(), 
                                                                    deltaTime.toSec());
            result = false;
        } else { result = true; }

    } catch (tf2::TransformException &ex) {
        ROS_WARN("[%s] %s", ros::this_node::getName().c_str(), ex.what());
        result = false;
    }
    return result;
}

void NMPCNavControlROS::mainLoop()
{
    ROS_INFO("[%s] NMPC navigation controller node initialized successfully. Ready for operation..", ros::this_node::getName().c_str());

    ros::Rate loop_rate(control_freq_);
    while (ros::ok()) {
        
        // ros::WallTime start_time = ros::WallTime::now(); // Use system time

        if (getRobotPose()) {
            if (current_mode_ == Mode::FollowPath) {
                processNearestPoint();
                processPathBuffers(active_path_u_);

                PathDiscretizer pathDiscretizer(mpc_control_->getDeltaTime(), mpc_control_->getHorizon(), false);
                std::vector<PathDiscretizer::Pose> next_poses;
                pathDiscretizer.getNextNPoses(active_path_, active_path_u_, next_poses);
                goal_pose_array_.clear();

                for (size_t i = 0; i < next_poses.size(); i++) {
                    NMPCNavControl::Pose pose;
                    pose.x = next_poses[i].x;
                    pose.y = next_poses[i].y;
                    pose.theta = next_poses[i].theta;
                    goal_pose_array_.push_back(pose);
                }
            }
            
            if (goal_pose_array_.empty()) { current_mode_ = Mode::Idle; }

            if (current_mode_ != Mode::Idle) {
                try {
                    bool pub_vel, reach_end_traj;
                    mpc_control_->run(robot_pose_, goal_pose_array_, robot_vel_ref_, cpu_time_, pub_vel, reach_end_traj);
                    if (pub_vel) { pubCmdVel(); }
                    if (reach_end_traj) { current_mode_ = Mode::Idle; } // TODO: Enable stack up trajectory
                    ROS_DEBUG_NAMED("nmpc_solver", "CPU time: %lf ms", cpu_time_);
                } catch (const std::exception& e) {
                    ROS_ERROR("[%s] Exception caught: %s.", ros::this_node::getName().c_str(), e.what());
                    current_mode_ = Mode::Idle;
                }
            } 
        }
        
        // ros::WallDuration timeLoad = ros::WallTime::now() - start_time;
        // ROS_INFO("Control loop time load: %lf ms", timeLoad.toNSec()*1.0e-6);

        ros::spinOnce();
        loop_rate.sleep();
    }
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////
/******************************************* Help Functions **********************************************/
///////////////////////////////////////////////////////////////////////////////////////////////////////////

void NMPCNavControlROS::processPathBuffers(const double active_path_u)
{
    double path_length = 0.0;
    for (TPathList::iterator it = active_path_.begin(); it != active_path_.end(); it++) {
        if (it == active_path_.begin()) { path_length += it->GetPathLength() * (1 - active_path_u); } // aproximation
        else { path_length += it->GetPathLength(); }
	}

    while ((path_length < max_active_path_length_) && (upcoming_path_.size() > 0)) {
        if (active_path_.size() > 0) {
            parametric_trajectories_common::TPath active_path_test = active_path_.back();
            parametric_trajectories_common::TPath upcoming_path_test = upcoming_path_.front();
            if ((active_path_test.GetVelocity() * upcoming_path_test.GetVelocity()) < 0) { break; }
            if (active_path_test.GetFrameId() != upcoming_path_test.GetFrameId()) { break; }
        }
        active_path_.push_back(upcoming_path_.front());
        upcoming_path_.pop_front();
        path_length += active_path_.back().GetPathLength();
    }
}

void NMPCNavControlROS::processNearestPoint()
{
    parametric_trajectories_common::TPathProcessMinDist min_dist_opti(10, 0.01);
    double x, y, theta, theta_holonomic;
    active_path_u_ = min_dist_opti.GetMinDist(active_path_, robot_pose_.x, robot_pose_.y, robot_pose_.theta, 
                                              x, y, theta, theta_holonomic);

    unsigned int active_path_num = std::floor(active_path_u_);
    if (active_path_.size() > active_path_num) {
        for (unsigned int i = 0; i < active_path_num; i++) {
            active_path_.pop_front();
            active_path_u_ = active_path_u_ - 1;
        }
    }
}

double NMPCNavControlROS::calcDistToEnd()
{
    double distance_to_end = 0.0;
    for (TPathList::iterator it = active_path_.begin(); it != active_path_.end(); it++) {
        if (it == active_path_.begin()) { distance_to_end += it->GetPathLength() * (1 - active_path_u_); } // aproximation
        else { distance_to_end += it->GetPathLength(); }
    }
    return distance_to_end;
}


} // namespace nmpc_nav_control