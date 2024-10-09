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
    
    sub_goal_pose_          = nh_.subscribe<geometry_msgs::PoseStamped>("pose_goal", 10, 
                              &NMPCNavControlROS::goalPoseReceivedCallback, this);
    sub_path_no_stack_up_   = nh_.subscribe<itrci_nav::ParametricPathSet>("path_no_stack_up", 10, 
                              &NMPCNavControlROS::pathNoStackUpReceivedCallback, this);
    sub_path_no_stack_up_2_ = nh_.subscribe<itrci_nav::ParametricPathSet2>("path_no_stack_up_2", 10, 
                              &NMPCNavControlROS::pathNoStackUp2ReceivedCallback, this);
    sub_control_command_    = nh_.subscribe<std_msgs::String>("control_command", 10, 
                              &NMPCNavControlROS::controlCommandReceivedCallback, this);
    pub_cmd_vel_            = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 10);
    pub_control_status_     = nh_.advertise<itrci_nav::parametric_trajectories_control_status>("control_status", 10);
    pub_actual_path_        = nh_.advertise<itrci_nav::ParametricPathSet>("actual_path", 10);
  
	ros::Duration timer_period = ros::Duration(static_cast<double>(1.0/control_freq_));
    main_timer_ = nh_priv_.createTimer(timer_period, &NMPCNavControlROS::mainTimerCallBack, this, false, false);

    ROS_INFO("[%s] NMPC navigation controller node initialized successfully. Ready for operation..", 
             ros::this_node::getName().c_str()); 
    ros::Duration(2.0).sleep(); // Delay to fill tf buffer
    main_timer_.start();
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
    current_status_ = Status::GoToPose;
    goal_pose_.header = msg->header;
    goal_pose_.pose = msg->pose;
}

void NMPCNavControlROS::pathNoStackUpReceivedCallback(const itrci_nav::ParametricPathSet::ConstPtr& msg)
{
    path_request_id_ = 0; // TODO: ?????
    processPathReceived(*msg);
}

void NMPCNavControlROS::pathNoStackUp2ReceivedCallback(const itrci_nav::ParametricPathSet2::ConstPtr& msg)
{
    itrci_nav::ParametricPathSet path;
    path.PathSet = msg->PathSet;
    path.AuxNum0 = msg->AuxNum0;
    path_request_id_ = msg->request_id; // TODO: ?????
    processPathReceived(path);
}

void NMPCNavControlROS::controlCommandReceivedCallback(const std_msgs::String::ConstPtr& msg)
{
    std::string command = msg->data;
    if (command == "break") { current_status_ = Status::Break; }
    else if(command == "idle") { current_status_ = Status::Idle; }
    else { ROS_ERROR("[%s] %s is a a invalide control command!", 
           ros::this_node::getName().c_str(), command.c_str()); }
}

void NMPCNavControlROS::pubCmdVel() 
{
    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = robot_vel_ref_.v;
    cmd_vel.linear.y = robot_vel_ref_.vn;
    cmd_vel.angular.z = robot_vel_ref_.w;
    pub_cmd_vel_.publish(cmd_vel);
}

void NMPCNavControlROS::pubControlStatus()
{
    itrci_nav::parametric_trajectories_control_status msg;
    double patch_remains;
    switch (current_status_) {
        case Status::Idle:
        case Status::Break:
            msg.status = itrci_nav::parametric_trajectories_control_status::STATUS_IDLE;
            break;
        case Status::FollowPath:
            patch_remains = active_path_.size() + upcoming_path_.size();
            if (patch_remains > 0) { patch_remains = patch_remains - active_path_u_; }
            msg.request_id = path_request_id_; // TODO: ?????
            msg.patch_remains = patch_remains; // TODO: ?????
            msg.status = itrci_nav::parametric_trajectories_control_status::STATUS_WORKING;
            break;
        case Status::GoToPose:
            msg.status = itrci_nav::parametric_trajectories_control_status::STATUS_WORKING;
            break;
        case Status::Error:
            msg.status = itrci_nav::parametric_trajectories_control_status::STATUS_ERROR;
            break;
    }
    pub_control_status_.publish(msg);
}

void NMPCNavControlROS::pubActualPath()
{
    if (active_path_.size() <= 0) { return; }
    parametric_trajectories_common::TPathRosDecode path_set_ros_decode;
    itrci_nav::ParametricPath actual_path = path_set_ros_decode.toRos(active_path_.front());
    itrci_nav::ParametricPathSet msg;
    msg.PathSet.push_back(actual_path);
    msg.AuxNum0 = active_path_u_;
    pub_actual_path_.publish(msg);
}

void NMPCNavControlROS::getRobotPose() 
{
	ros::Time currentTime = ros::Time::now();
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
            current_status_ = Status::Error;
        }
    } catch (tf2::TransformException &ex) {
        ROS_WARN("[%s] %s", ros::this_node::getName().c_str(), ex.what());
        current_status_ = Status::Error;
    }
}

void NMPCNavControlROS::mainTimerCallBack(const ros::TimerEvent&)
{
    ros::WallTime start_time = ros::WallTime::now();
    mainCycle();
    ros::WallDuration time_load = ros::WallTime::now() - start_time;
    ROS_DEBUG_NAMED("main_cycle", "Control loop time load: %lf ms", time_load.toNSec()*1.0e-6);
}

void NMPCNavControlROS::mainCycle()
{
    getRobotPose();

    switch (current_status_) {
        case Status::GoToPose:
            processGoToPose();
            break;
        case Status::FollowPath:
            processFollowPath();
            break;
        case Status::Break:
            processBreak();
            break;
        case Status::Error:
            break;
        case Status::Idle:
            break;
    }

    pubControlStatus();
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////
/******************************************* Help Functions **********************************************/
///////////////////////////////////////////////////////////////////////////////////////////////////////////

void NMPCNavControlROS::processPathReceived(const itrci_nav::ParametricPathSet& path)
{
    current_status_ = Status::FollowPath;

    if (path.PathSet.empty()) { 
        ROS_WARN("Received an empty Path, ignoring...");
        return;
    }
    parametric_trajectories_common::TPathSetRosDecode path_set_ros_decode;
    TPathList path_set = path_set_ros_decode.fromRos(path);
		
    upcoming_path_.clear();
    active_path_.clear();
    for (TPathList::const_iterator it = path_set.begin(); it != path_set.end(); it++) {
        if (it->GetFrameId() == "") { continue; }
        upcoming_path_.push_back(*it);
        upcoming_path_.back().SetPathLength(1000);
    }
    processPathBuffers(0.0);
}

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

void NMPCNavControlROS::processBreak()
{
    robot_vel_ref_.v = 0.0;
    robot_vel_ref_.vn = 0.0;
    robot_vel_ref_.w = 0.0;
    pubCmdVel();
    current_status_ = Status::Idle;
}

void NMPCNavControlROS::processGoToPose()
{
    std::list<NMPCNavControl::Pose> goal_pose_array;
    NMPCNavControl::Pose pose;
    // TODO: goal_pose_.header.frame_id 
    pose.x = goal_pose_.pose.position.x;
    pose.y = goal_pose_.pose.position.y;
    pose.theta = tf2::getYaw(goal_pose_.pose.orientation);
    goal_pose_array.push_back(pose);

    executeNMPC(goal_pose_array);
}

void NMPCNavControlROS::processFollowPath()
{
    processNearestPoint();
    processPathBuffers(active_path_u_);

    PathDiscretizer pathDiscretizer(mpc_control_->getDeltaTime(), mpc_control_->getHorizon(), false);
    std::vector<PathDiscretizer::Pose> next_poses;
    pathDiscretizer.getNextNPoses(active_path_, active_path_u_, next_poses);
    
    std::list<NMPCNavControl::Pose> goal_pose_array;
    for (size_t i = 0; i < next_poses.size(); i++) {
        NMPCNavControl::Pose pose;
        pose.x = next_poses[i].x;
        pose.y = next_poses[i].y;
        pose.theta = next_poses[i].theta;
        goal_pose_array.push_back(pose);
    }

    pubActualPath();
    executeNMPC(goal_pose_array);
}

void NMPCNavControlROS::executeNMPC(const std::list<NMPCNavControl::Pose>& goal_pose_array)
{
    if (goal_pose_array.empty()) { 
        current_status_ = Status::Idle;
        return;
    }

    try {
        bool pub_vel, reach_end_traj;
        double cpu_time;
        mpc_control_->run(robot_pose_, goal_pose_array, robot_vel_ref_, cpu_time, pub_vel, reach_end_traj);
        if (pub_vel) { pubCmdVel(); }
        if (reach_end_traj) { current_status_ = Status::Idle; } // TODO: Enable stack up trajectory
        ROS_DEBUG_NAMED("nmpc_solver", "CPU time: %lf ms", cpu_time);
    } catch (const std::exception& e) {
        ROS_ERROR("[%s] Exception caught: %s.", ros::this_node::getName().c_str(), e.what());
        current_status_ = Status::Error;
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