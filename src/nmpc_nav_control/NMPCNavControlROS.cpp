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
    
    tf_buffer_ = new tf2_ros::Buffer;
	tf_listener_ = new tf2_ros::TransformListener(*tf_buffer_);
    
    pub_cmd_vel_   = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 10);
    sub_goal_pose_ = nh_.subscribe<geometry_msgs::PoseStamped>("pose_goal", 10, 
                     &NMPCNavControlROS::goalPoseCallback, this);

    mainLoop();
}

void NMPCNavControlROS::readParam()
{
    nh_priv_.param<std::string>("global_frame_id", global_frame_id_, "map");
    nh_priv_.param<std::string>("base_frame_id", base_frame_id_, "base_footprint");
    nh_priv_.param<int>("control_freq", control_freq_, 40);
    nh_priv_.param<double>("transform_timeout", transform_timeout_, 0.1);

    double l1, l2, l1_plus_l2, dt;
    nh_priv_.param<double>("rob_dist_between_front_back_wh", l1, 0.265);
    nh_priv_.param<double>("rob_dist_between_left_right_wh", l2, 0.270);
    l1_plus_l2 = l1+l2;
    dt = 1.0/double(control_freq_);    

    ROS_INFO("[%s] Global frame ID: %s", ros::this_node::getName().c_str(), global_frame_id_.c_str());
    ROS_INFO("[%s] Base frame ID: %s", ros::this_node::getName().c_str(), base_frame_id_.c_str());
    ROS_INFO("[%s] Controller frequency: %d Hz", ros::this_node::getName().c_str(), control_freq_);
    ROS_INFO("[%s] Transform timeout: %.02f s", ros::this_node::getName().c_str(), transform_timeout_);
    ROS_INFO("[%s] Robot distances (l1 + l2): %lf m", ros::this_node::getName().c_str(), l1_plus_l2);

    try {
        mpc_control_ = std::make_unique<NMPCNavControl>(l1_plus_l2, dt);
    } catch (const std::exception& e) {
        ROS_ERROR("[%s] Exception caught: %s. Terminating node.", ros::this_node::getName().c_str(), e.what());
        ros::shutdown();
    }
}

void NMPCNavControlROS::goalPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    NMPCNavControl::Pose goal_pose;
    goal_pose.x = msg->pose.position.x;
    goal_pose.y = msg->pose.position.y;
    goal_pose.theta = tf2::getYaw(msg->pose.orientation);

    goal_pose_array_.clear();
    goal_pose_array_.push_back(goal_pose);
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
        if (!goal_pose_array_.empty() && getRobotPose()) {
            bool pub_vel = mpc_control_->run(robot_pose_, goal_pose_array_, robot_vel_ref_, cpu_time_);
            if (pub_vel) { pubCmdVel(); }
            ROS_DEBUG_NAMED("nmpc_solver", "CPU time: %lf s", cpu_time_);
        } 

        ros::spinOnce();
        loop_rate.sleep();
    }
}

} // namespace nmpc_nav_control