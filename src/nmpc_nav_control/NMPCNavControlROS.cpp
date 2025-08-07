#include "nmpc_nav_control/NMPCNavControlROS.h"

#include "nmpc_nav_control/NMPCNavControlOmni4.h"
#include "nmpc_nav_control/NMPCNavControlDiff.h"
#include "nmpc_nav_control/NMPCNavControlTric.h"
#include "nmpc_nav_control/utils.h"

namespace nmpc_nav_control {

NMPCNavControlROS::NMPCNavControlROS() : nh_priv_("~")
{
    try {
        readParam();
    } catch (std::exception& ex) {
        ROS_FATAL("[%s] Error reading the node parameters (%s)", ros::this_node::getName().c_str(), ex.what());
        ros::shutdown();
        return;
    }

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>();
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

    sub_goal_pose_              = nh_.subscribe<geometry_msgs::PoseStamped>("pose_goal", 10,
                                  &NMPCNavControlROS::goalPoseReceivedCallback, this);
    sub_path_no_stack_up_       = nh_.subscribe<itrci_nav::ParametricPathSet>("path_no_stack_up", 10,
                                  &NMPCNavControlROS::pathNoStackUpReceivedCallback, this);
    sub_path_no_stack_up_2_     = nh_.subscribe<itrci_nav::ParametricPathSet2>("path_no_stack_up_2", 10,
                                  &NMPCNavControlROS::pathNoStackUp2ReceivedCallback, this);
    sub_control_command_        = nh_.subscribe<std_msgs::String>("control_command", 10,
                                  &NMPCNavControlROS::controlCommandReceivedCallback, this);
    pub_cmd_vel_                = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 10);
    pub_control_status_         = nh_.advertise<itrci_nav::parametric_trajectories_control_status>("control_status", 10);
    pub_actual_path_            = nh_.advertise<itrci_nav::ParametricPathSet>("actual_path", 10);
    pub_debug_discretized_path_ = nh_.advertise<nav_msgs::Path>("debug_discretized_path", 10);

    ros::Duration timer_period = ros::Duration(static_cast<double>(1.0/control_freq_));
    main_timer_ = nh_priv_.createTimer(timer_period, &NMPCNavControlROS::mainTimerCallBack, this, false, false);

    ROS_INFO("[%s] NMPC navigation controller node initialized successfully. Ready for operation..", ros::this_node::getName().c_str());
    ros::Duration(2.0).sleep(); // Delay to fill tf buffer
    main_timer_.start();
}

void NMPCNavControlROS::readParam()
{
    if (!nh_priv_.hasParam("steering_geometry")) {
        throw std::runtime_error("The node nmpc_nav_control requires the definition "
                                 "of the steering_geometry parameter");
    }
    ROS_INFO("[%s] Modified Version", ros::this_node::getName().c_str());
    nh_priv_.param<std::string>("global_frame_id", global_frame_id_, "map");
    nh_priv_.param<std::string>("base_frame_id", base_frame_id_, "base_footprint");
    nh_priv_.param<int>("control_freq", control_freq_, 40);
    nh_priv_.param<double>("transform_timeout", transform_timeout_, 0.1);
    nh_priv_.param<double>("max_active_path_length", max_active_path_length_, 5.0);

    nh_priv_.param<double>("final_position_error", final_position_error_, 0.01);
    nh_priv_.param<double>("final_orientation_error", final_orientation_error_, 1.0);
    final_orientation_error_ = final_orientation_error_ * M_PI/180.0;

    nh_priv_.param<bool>("enable_safe_conditions", enable_safe_conditions_, true);
    nh_priv_.param<double>("max_goal_pose_dist", max_goal_pose_dist_, 2.0);
    nh_priv_.param<double>("max_pos_error_to_path", max_pos_error_to_path_, 0.5);
    nh_priv_.param<double>("max_ori_error_to_path", max_ori_error_to_path_, 60.0);
    max_ori_error_to_path_ = max_ori_error_to_path_ * M_PI/180.0;

    ROS_INFO("[%s] Global frame ID: %s", ros::this_node::getName().c_str(), global_frame_id_.c_str());
    ROS_INFO("[%s] Base frame ID: %s", ros::this_node::getName().c_str(), base_frame_id_.c_str());
    ROS_INFO("[%s] Controller frequency: %d Hz", ros::this_node::getName().c_str(), control_freq_);
    ROS_INFO("[%s] Transform timeout: %lf s", ros::this_node::getName().c_str(), transform_timeout_);
    ROS_INFO("[%s] Maximum active path length: %lf m", ros::this_node::getName().c_str(), max_active_path_length_);
    ROS_INFO("[%s] Final position error: %lf m", ros::this_node::getName().c_str(), final_position_error_);
    ROS_INFO("[%s] Final orientation error: %lf deg", ros::this_node::getName().c_str(), final_orientation_error_*180/M_PI);
    ROS_INFO("[%s] Enable safe conditions: %s", ros::this_node::getName().c_str(), enable_safe_conditions_ ? "yes" : "no");
    ROS_INFO("[%s] Maximum goal pose distance: %lf m", ros::this_node::getName().c_str(), max_goal_pose_dist_);
    ROS_INFO("[%s] Maximum position error to path: %lf m", ros::this_node::getName().c_str(), max_pos_error_to_path_);
    ROS_INFO("[%s] Maximum orientation error to path: %lf deg", ros::this_node::getName().c_str(), max_ori_error_to_path_*180/M_PI);

    nh_priv_.getParam("steering_geometry", steering_geometry_);
    ROS_INFO("[%s] Steering geometry: %s", ros::this_node::getName().c_str(), steering_geometry_.c_str());

    double dt = 1.0 / double(control_freq_);
    if (steering_geometry_ == kOmni4Str) {
        if (!nh_priv_.hasParam("rob_dist_between_front_back_wh") ||
            !nh_priv_.hasParam("rob_dist_between_left_right_wh") ||
            !nh_priv_.hasParam("rob_wh_vel_time_const") ||
            !nh_priv_.hasParam("rob_wh_max_vel") ||
            !nh_priv_.hasParam("rob_wh_max_ace")) {
            throw std::runtime_error("The steering geometry " + steering_geometry_ + " requires the "
                                     "definition of the following parameters: "
                                     "rob_dist_between_front_back_wh, rob_dist_between_left_right_wh, "
                                     "rob_wh_vel_time_const, rob_wh_max_vel, rob_wh_max_ace, "
                                     "cost_matrix_weights_state_diag, cost_matrix_weights_input_diag");
        }
        double l1, l2, l1_plus_l2, tau_v, v_max, a_max;
        nh_priv_.getParam("rob_dist_between_front_back_wh", l1);
        nh_priv_.getParam("rob_dist_between_left_right_wh", l2);
        l1_plus_l2 = l1 + l2;
        nh_priv_.getParam("rob_wh_vel_time_const", tau_v);
        nh_priv_.getParam("rob_wh_max_vel", v_max);
        nh_priv_.getParam("rob_wh_max_ace", a_max);

        ROS_INFO("[%s] Robot distance between wheels (l1 + l2): %lf m", ros::this_node::getName().c_str(), l1_plus_l2);
        ROS_INFO("[%s] Robot wheel velocity time constant (tau_v): %lf s", ros::this_node::getName().c_str(), tau_v);
        ROS_INFO("[%s] Robot wheel maximum linear velocity: %lf m/s", ros::this_node::getName().c_str(), v_max);
        ROS_INFO("[%s] Robot wheel maximum linear acceleration: %lf m/s^2", ros::this_node::getName().c_str(), a_max);

        XmlRpc::XmlRpcValue Q_param, R_param;
        nh_priv_.getParam("cost_matrix_weights_state_diag", Q_param);
        nh_priv_.getParam("cost_matrix_weights_input_diag", R_param);
        if (Q_param.getType() != XmlRpc::XmlRpcValue::TypeArray || Q_param.size() != 11) {
            throw std::runtime_error("Parameter 'cost_matrix_weights_state_diag' must be an array of 11 numeric values.");
        }
        if (R_param.getType() != XmlRpc::XmlRpcValue::TypeArray || R_param.size() != 4) {
            throw std::runtime_error("Parameter 'cost_matrix_weights_input_diag' must be an array of 4 numeric values.");
        }

        std::vector<double> Q_diag(11), R_diag(4);
        for (int i = 0; i < 11; i++) {
            if (Q_param[i].getType() != XmlRpc::XmlRpcValue::TypeDouble &&
                Q_param[i].getType() != XmlRpc::XmlRpcValue::TypeInt) {
                throw std::runtime_error("Parameter 'cost_matrix_weights_state_diag' must be an array of 11 numeric values.");
            }
            Q_diag[i] = static_cast<double>(Q_param[i]);
        }
        for (int i = 0; i < 4; i++) {
            if (R_param[i].getType() != XmlRpc::XmlRpcValue::TypeDouble &&
                R_param[i].getType() != XmlRpc::XmlRpcValue::TypeInt) {
                throw std::runtime_error("Parameter 'cost_matrix_weights_input_diag' must be an array of 4 numeric values.");
            }
            R_diag[i] = static_cast<double>(R_param[i]);
        }

        ROS_INFO("[%s] Cost matrix weights regarding the state (diagonal elements): [%lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf]",
                 ros::this_node::getName().c_str(), Q_diag[0], Q_diag[1], Q_diag[2], Q_diag[3], Q_diag[4], Q_diag[5],
                                                    Q_diag[6], Q_diag[7], Q_diag[8], Q_diag[9], Q_diag[10]);
        ROS_INFO("[%s] Cost matrix weights regarding the input (diagonal elements): [%lf, %lf, %lf, %lf]",
                 ros::this_node::getName().c_str(), R_diag[0], R_diag[1], R_diag[2], R_diag[4]);

        std::vector<double> W_diag(15);
        for (int i = 0; i < 11; i++) { W_diag[i] = Q_diag[i]; }
        for (int i = 0; i < 4; i++) { W_diag[11+i] = R_diag[i]; }

        try {
            mpc_control_ = std::make_unique<NMPCNavControlOmni4>(dt, l1_plus_l2, tau_v, v_max, a_max, W_diag);
            robot_vel_ref_ = std::make_unique<NMPCNavControlOmni4::CmdVelOmni4>();
        } catch (const std::exception& e) {
            ROS_ERROR("[%s] Exception caught: %s. Terminating node.", ros::this_node::getName().c_str(), e.what());
            ros::shutdown();
        }
    } else if (steering_geometry_ == kDiffStr) {
        if (!nh_priv_.hasParam("rob_dist_between_wh") ||
            !nh_priv_.hasParam("rob_wh_vel_time_const") ||
            !nh_priv_.hasParam("rob_wh_max_vel") ||
            !nh_priv_.hasParam("rob_wh_max_ace") ||
            !nh_priv_.hasParam("cost_matrix_weights_state_diag") ||
            !nh_priv_.hasParam("cost_matrix_weights_input_diag")) {
            throw std::runtime_error("The steering geometry " + steering_geometry_ + " requires the "
                                     "definition of the following parameter: "
                                     "rob_dist_between_wh, rob_wh_vel_time_const, rob_wh_max_vel, rob_wh_max_ace, "
                                     "cost_matrix_weights_state_diag, cost_matrix_weights_input_diag");
        }
        double dist_b, tau_v, v_max, a_max;
        nh_priv_.getParam("rob_dist_between_wh", dist_b);
        nh_priv_.getParam("rob_wh_vel_time_const", tau_v);
        nh_priv_.getParam("rob_wh_max_vel", v_max);
        nh_priv_.getParam("rob_wh_max_ace", a_max);

        ROS_INFO("[%s] Robot distance between wheels (b): %lf m", ros::this_node::getName().c_str(), dist_b);
        ROS_INFO("[%s] Robot wheel velocity time constant (tau_v): %lf s", ros::this_node::getName().c_str(), tau_v);
        ROS_INFO("[%s] Robot wheel maximum linear velocity: %lf m/s", ros::this_node::getName().c_str(), v_max);
        ROS_INFO("[%s] Robot wheel maximum linear acceleration: %lf m/s^2", ros::this_node::getName().c_str(), a_max);

        XmlRpc::XmlRpcValue Q_param, R_param;
        nh_priv_.getParam("cost_matrix_weights_state_diag", Q_param);
        nh_priv_.getParam("cost_matrix_weights_input_diag", R_param);
        if (Q_param.getType() != XmlRpc::XmlRpcValue::TypeArray || Q_param.size() != 7) {
            throw std::runtime_error("Parameter 'cost_matrix_weights_state_diag' must be an array of 7 numeric values.");
        }
        if (R_param.getType() != XmlRpc::XmlRpcValue::TypeArray || R_param.size() != 2) {
            throw std::runtime_error("Parameter 'cost_matrix_weights_input_diag' must be an array of 2 numeric values.");
        }

        std::vector<double> Q_diag(7), R_diag(2);
        for (int i = 0; i < 7; i++) {
            if (Q_param[i].getType() != XmlRpc::XmlRpcValue::TypeDouble &&
                Q_param[i].getType() != XmlRpc::XmlRpcValue::TypeInt) {
                throw std::runtime_error("Parameter 'cost_matrix_weights_state_diag' must be an array of 7 numeric values.");
            }
            Q_diag[i] = static_cast<double>(Q_param[i]);
        }
        for (int i = 0; i < 2; i++) {
            if (R_param[i].getType() != XmlRpc::XmlRpcValue::TypeDouble &&
                R_param[i].getType() != XmlRpc::XmlRpcValue::TypeInt) {
                throw std::runtime_error("Parameter 'cost_matrix_weights_input_diag' must be an array of 2 numeric values.");
            }
            R_diag[i] = static_cast<double>(R_param[i]);
        }

        ROS_INFO("[%s] Cost matrix weights regarding the state (diagonal elements): [%lf, %lf, %lf, %lf, %lf, %lf, %lf]",
                 ros::this_node::getName().c_str(), Q_diag[0], Q_diag[1], Q_diag[2], Q_diag[3], Q_diag[4], Q_diag[5], Q_diag[6]);
        ROS_INFO("[%s] Cost matrix weights regarding the input (diagonal elements): [%lf, %lf]",
                 ros::this_node::getName().c_str(), R_diag[0], R_diag[1]);

        std::vector<double> W_diag(9);
        for (int i = 0; i < 7; i++) { W_diag[i] = Q_diag[i]; }
        for (int i = 0; i < 2; i++) { W_diag[7+i] = R_diag[i]; }

        try {
            mpc_control_ = std::make_unique<NMPCNavControlDiff>(dt, dist_b, tau_v, v_max, a_max, W_diag);
            robot_vel_ref_ = std::make_unique<NMPCNavControlDiff::CmdVelDiff>();
        } catch (const std::exception& e) {
            ROS_ERROR("[%s] Exception caught: %s. Terminating node.", ros::this_node::getName().c_str(), e.what());
            ros::shutdown();
        }
    } else if (steering_geometry_ == kTricStr) {
        if (!nh_priv_.hasParam("steering_wheel_frame_id") ||
            !nh_priv_.hasParam("rob_dist_between_steering_back_wh") ||
            !nh_priv_.hasParam("rob_wh_vel_time_const") ||
            !nh_priv_.hasParam("rob_steer_wh_angle_time_const") ||
            !nh_priv_.hasParam("rob_wh_max_vel") ||
            !nh_priv_.hasParam("rob_wh_max_ace") ||
            !nh_priv_.hasParam("rob_steer_wh_min_angle") ||
            !nh_priv_.hasParam("rob_steer_wh_max_angle") ||
            !nh_priv_.hasParam("rob_steer_wh_max_angle_var")) {
            throw std::runtime_error("The steering geometry " + steering_geometry_ + " requires the "
                                     "definition of the following parameters: "
                                     "steering_wheel_frame_id, rob_dist_between_steering_back_wh, rob_wh_vel_time_const, "
                                     "rob_wh_max_vel, rob_wh_max_ace, rob_steer_wh_min_angle, rob_steer_wh_max_angle, "
                                     "rob_steer_wh_max_angle_var, "
                                     "cost_matrix_weights_state_diag, cost_matrix_weights_input_diag");
        }
        double dist_d, tau_v, tau_a, v_max, a_max, alpha_min, alpha_max, dalpha_max;
        nh_priv_.getParam("steering_wheel_frame_id", steering_wheel_frame_id_);
        nh_priv_.getParam("rob_dist_between_steering_back_wh", dist_d);
        nh_priv_.getParam("rob_wh_vel_time_const", tau_v);
        nh_priv_.getParam("rob_steer_wh_angle_time_const", tau_a);
        nh_priv_.getParam("rob_wh_max_vel", v_max);
        nh_priv_.getParam("rob_wh_max_ace", a_max);
        nh_priv_.getParam("rob_steer_wh_min_angle", alpha_min);
        nh_priv_.getParam("rob_steer_wh_max_angle", alpha_max);
        nh_priv_.getParam("rob_steer_wh_max_angle_var", dalpha_max);
        alpha_min = alpha_min * M_PI/180.0;
        alpha_max = alpha_max * M_PI/180.0;
        dalpha_max = dalpha_max * M_PI/180.0;

        ROS_INFO("[%s] Steering wheel frame ID: %s", ros::this_node::getName().c_str(), steering_wheel_frame_id_.c_str());
        ROS_INFO("[%s] Robot distance between wheels (d): %lf m", ros::this_node::getName().c_str(), dist_d);
        ROS_INFO("[%s] Robot wheel velocity time constant (tau_v): %lf s", ros::this_node::getName().c_str(), tau_v);
        ROS_INFO("[%s] Robot steering wheel angle time constant (tau_a): %lf s", ros::this_node::getName().c_str(), tau_a);
        ROS_INFO("[%s] Robot wheel maximum linear velocity: %lf m/s", ros::this_node::getName().c_str(), v_max);
        ROS_INFO("[%s] Robot wheel maximum linear acceleration: %lf m/s^2", ros::this_node::getName().c_str(), a_max);
        ROS_INFO("[%s] Robot steering wheel minimum angle: %lf deg", ros::this_node::getName().c_str(), alpha_min*180.0/M_PI);
        ROS_INFO("[%s] Robot steering wheel minimum angle: %lf deg", ros::this_node::getName().c_str(), alpha_max*180.0/M_PI);
        ROS_INFO("[%s] Robot steering wheel maximum angle variation: %lf deg/s", ros::this_node::getName().c_str(), dalpha_max*180.0/M_PI);

        XmlRpc::XmlRpcValue Q_param, R_param;
        nh_priv_.getParam("cost_matrix_weights_state_diag", Q_param);
        nh_priv_.getParam("cost_matrix_weights_input_diag", R_param);
        if (Q_param.getType() != XmlRpc::XmlRpcValue::TypeArray || Q_param.size() != 7) {
            throw std::runtime_error("Parameter 'cost_matrix_weights_state_diag' must be an array of 7 numeric values.");
        }
        if (R_param.getType() != XmlRpc::XmlRpcValue::TypeArray || R_param.size() != 2) {
            throw std::runtime_error("Parameter 'cost_matrix_weights_input_diag' must be an array of 2 numeric values.");
        }

        std::vector<double> Q_diag(7), R_diag(2);
        for (int i = 0; i < 7; i++) {
            if (Q_param[i].getType() != XmlRpc::XmlRpcValue::TypeDouble &&
                Q_param[i].getType() != XmlRpc::XmlRpcValue::TypeInt) {
                throw std::runtime_error("Parameter 'cost_matrix_weights_state_diag' must be an array of 7 numeric values.");
            }
            Q_diag[i] = static_cast<double>(Q_param[i]);
        }
        for (int i = 0; i < 2; i++) {
            if (R_param[i].getType() != XmlRpc::XmlRpcValue::TypeDouble &&
                R_param[i].getType() != XmlRpc::XmlRpcValue::TypeInt) {
                throw std::runtime_error("Parameter 'cost_matrix_weights_input_diag' must be an array of 2 numeric values.");
            }
            R_diag[i] = static_cast<double>(R_param[i]);
        }

        ROS_INFO("[%s] Cost matrix weights regarding the state (diagonal elements): [%lf, %lf, %lf, %lf, %lf, %lf, %lf]",
                 ros::this_node::getName().c_str(), Q_diag[0], Q_diag[1], Q_diag[2], Q_diag[3], Q_diag[4], Q_diag[5], Q_diag[6]);
        ROS_INFO("[%s] Cost matrix weights regarding the input (diagonal elements): [%lf, %lf]",
                 ros::this_node::getName().c_str(), R_diag[0], R_diag[1]);

        std::vector<double> W_diag(9);
        for (int i = 0; i < 7; i++) { W_diag[i] = Q_diag[i]; }
        for (int i = 0; i < 2; i++) { W_diag[7+i] = R_diag[i]; }

        try {
            mpc_control_ = std::make_unique<NMPCNavControlTric>(dt, dist_d, tau_v, tau_a, v_max, a_max, alpha_min, alpha_max, dalpha_max, W_diag);
            robot_vel_ref_ = std::make_unique<NMPCNavControlTric::CmdVelTric>();
        } catch (const std::exception& e) {
            ROS_ERROR("[%s] Exception caught: %s. Terminating node.", ros::this_node::getName().c_str(), e.what());
            ros::shutdown();
        }
    } else {
        throw std::runtime_error("Invalid steering_geometry (check documentation for supported ones)");
    }
}

void NMPCNavControlROS::goalPoseReceivedCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    current_status_ = Status::GoToPose;
    goal_pose_.header = msg->header;
    goal_pose_.pose = msg->pose;
    mpc_control_->reset_mpc();
}

void NMPCNavControlROS::pathNoStackUpReceivedCallback(const itrci_nav::ParametricPathSet::ConstPtr& msg)
{
    path_request_id_ = 0; // TODO: ?????
    processPathReceived(*msg);
    mpc_control_->reset_mpc();
}

void NMPCNavControlROS::pathNoStackUp2ReceivedCallback(const itrci_nav::ParametricPathSet2::ConstPtr& msg)
{
    itrci_nav::ParametricPathSet path;
    path.PathSet = msg->PathSet;
    path.AuxNum0 = msg->AuxNum0;
    path_request_id_ = msg->request_id; // TODO: ?????
    processPathReceived(path);
    mpc_control_->reset_mpc();
}

void NMPCNavControlROS::controlCommandReceivedCallback(const std_msgs::String::ConstPtr& msg)
{
    std::string command = msg->data;
    if (command == "break") { current_status_ = Status::Break; }
    else if(command == "idle") { current_status_ = Status::Idle; }
    else { ROS_ERROR("[%s] %s is a a invalide control command!",
           ros::this_node::getName().c_str(), command.c_str()); }
}

void NMPCNavControlROS::pubCmdVel(bool stop)
{
    geometry_msgs::Twist cmd_vel;
    if (steering_geometry_ == kOmni4Str) {
        auto vel_ref = dynamic_cast<NMPCNavControlOmni4::CmdVelOmni4*>(robot_vel_ref_.get());
        cmd_vel.linear.x  = stop ? 0.0 : vel_ref->v;
        cmd_vel.linear.y  = stop ? 0.0 : vel_ref->vn;
        cmd_vel.angular.z = stop ? 0.0 : vel_ref->w;
    } else if (steering_geometry_ == kDiffStr) {
        auto vel_ref = dynamic_cast<NMPCNavControlDiff::CmdVelDiff*>(robot_vel_ref_.get());
        cmd_vel.linear.x  = stop ? 0.0 : vel_ref->v;
        cmd_vel.linear.y  = 0.0;
        cmd_vel.angular.z = stop ? 0.0 : vel_ref->w;
    } else if (steering_geometry_ == kTricStr) {
        auto vel_ref = dynamic_cast<NMPCNavControlTric::CmdVelTric*>(robot_vel_ref_.get());
        cmd_vel.linear.x  = stop ? 0.0 : vel_ref->v;
        cmd_vel.linear.y  = 0.0;
        cmd_vel.angular.z = robot_steering_wheel_angle_;
    } else {
        ROS_ERROR("[%s] Unsupported steering geometry in pubCmdVel()", ros::this_node::getName().c_str());
        return;
    }

    pub_cmd_vel_.publish(cmd_vel);
}

void NMPCNavControlROS::pubControlStatus()
{
    itrci_nav::parametric_trajectories_control_status msg;
    double path_remains;
    switch (current_status_) {
        case Status::Idle:
        case Status::Break:
            msg.status = itrci_nav::parametric_trajectories_control_status::STATUS_IDLE;
            break;
        case Status::FollowPath:
            path_remains = active_path_.size() + upcoming_path_.size();
            if (path_remains > 0) { path_remains = path_remains - active_path_u_; }
            msg.request_id = path_request_id_; // TODO: ?????
            msg.patch_remains = path_remains; // TODO: ?????
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

bool NMPCNavControlROS::getRobotPose(const std::string& global_frame_id, ros::Time& time_stamp)
{
    ros::Time current_time = ros::Time::now();
    try {
        geometry_msgs::TransformStamped robot_pose;
        robot_pose = tf_buffer_->lookupTransform(global_frame_id, base_frame_id_, ros::Time(0));
        time_stamp = robot_pose.header.stamp;

        robot_pose_.x = robot_pose.transform.translation.x;
        robot_pose_.y = robot_pose.transform.translation.y;
        // robot_pose_.theta = tf2::getYaw(robot_pose.transform.rotation);

        // Hack: Bug fix for the angle wrap in acados solver
        double last_theta = robot_pose_.theta;
        double curr_theta = tf2::getYaw(robot_pose.transform.rotation);

        double delta = curr_theta - last_theta;
        if (delta > M_PI) { curr_theta -= 2.0 * M_PI; }
        else if (delta < -M_PI) { curr_theta += 2.0 * M_PI; }

        while (curr_theta >= 2.0 * M_PI) { curr_theta -= 2.0 * M_PI; }
        while (curr_theta <= -2.0 * M_PI) { curr_theta += 2.0 * M_PI; }
        robot_pose_.theta = curr_theta;

        ros::Duration delta_time = current_time - time_stamp;
        if (delta_time.toSec() > transform_timeout_) {
            ROS_ERROR("[%s] Robot pose data error. Data age = %f s", ros::this_node::getName().c_str(),
                                                                     delta_time.toSec());
            return false;
        }
    } catch (tf2::TransformException &ex) {
        ROS_ERROR("[%s] %s", ros::this_node::getName().c_str(), ex.what());
        return false;
    }
    return true;
}

bool NMPCNavControlROS::getRobotVel(const std::string& global_frame_id, const ros::Time& time_stamp)
{
    try {
        ros::Duration delta(1.0/control_freq_);
        ros::Time t1 = time_stamp - delta;
        ros::Time t2 = time_stamp;

        geometry_msgs::TransformStamped tf1 = tf_buffer_->lookupTransform(global_frame_id, base_frame_id_, t1);
        geometry_msgs::TransformStamped tf2 = tf_buffer_->lookupTransform(global_frame_id, base_frame_id_, t2);

        double dt = (tf2.header.stamp - tf1.header.stamp).toSec();
        if (dt <= 0.0 || dt > transform_timeout_) {
            ROS_WARN("[%s] Velocity calculation skipped due to large or invalid delta time: %f s",
            ros::this_node::getName().c_str(), dt);
            return false;
        }

        // Delta position in global frame
        double dx = tf2.transform.translation.x - tf1.transform.translation.x;
        double dy = tf2.transform.translation.y - tf1.transform.translation.y;

        // Orientation difference (yaw)
        double yaw1 = tf2::getYaw(tf1.transform.rotation);
        double yaw2 = tf2::getYaw(tf2.transform.rotation);
        double dyaw = normAngRad(yaw2 - yaw1);

        // Midpoint orientation for rotation into robot frame
        double mid_yaw = yaw1 + dyaw / 2.0;

        // Rotate global velocity into robot frame
        double vx_global = dx / dt;
        double vy_global = dy / dt;

        double cos_yaw = std::cos(-mid_yaw); // negative for inverse rotation
        double sin_yaw = std::sin(-mid_yaw);

        robot_vel_.v  = vx_global * cos_yaw - vy_global * sin_yaw; // forward velocity
        robot_vel_.vn = vx_global * sin_yaw + vy_global * cos_yaw; // lateral velocity
        robot_vel_.w  = dyaw / dt;                                 // angular velocity

    } catch (tf2::TransformException &ex) {
        ROS_WARN("[%s] Could not calculate velocity: %s", ros::this_node::getName().c_str(), ex.what());
        return false;
    }

    return true;
}

bool NMPCNavControlROS::getSteeringWheelAngle(const ros::Time &time_stamp)
{
    try {
        std::string error_msg;
        bool wheelTransformAvailable = tf_buffer_->canTransform(base_frame_id_, steering_wheel_frame_id_,
                                              time_stamp, ros::Duration(transform_timeout_), &error_msg);
        if (wheelTransformAvailable) {
            geometry_msgs::TransformStamped wheel_pose;
            wheel_pose = tf_buffer_->lookupTransform(base_frame_id_, steering_wheel_frame_id_, time_stamp);
            robot_steering_wheel_angle_ = tf2::getYaw(wheel_pose.transform.rotation);
        } else {
            ROS_ERROR("[%s] Error calculating steering wheel angle: %s", ros::this_node::getName().c_str(),
                                                                               error_msg.c_str());
            return false;
        }
    } catch (tf2::TransformException &ex) {
        ROS_ERROR("[%s] %s", ros::this_node::getName().c_str(), ex.what());
        return false;
    }
    return true;
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
    switch (current_status_) {
        case Status::GoToPose:
            getInputData(goal_pose_.header.frame_id);
            processGoToPose();
            break;
        case Status::FollowPath:
            getInputData(active_path_.front().GetFrameId());
            processFollowPath();
            break;
        case Status::Break:
            getInputData(global_frame_id_);
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

void NMPCNavControlROS::getInputData(const std::string& global_frame_id)
{
    bool valid_data;
    ros::Time time_stamp;
    valid_data = getRobotPose(global_frame_id, time_stamp);
    valid_data = getRobotVel(global_frame_id, time_stamp);
    if (steering_geometry_ == kTricStr) { valid_data &= getSteeringWheelAngle(time_stamp); }
    if (!valid_data) { current_status_ = Status::Error; }
}

void NMPCNavControlROS::processPathReceived(const itrci_nav::ParametricPathSet& path)
{
    current_status_ = Status::FollowPath;

    if (path.PathSet.empty()) {
        ROS_WARN("[%s] Received an empty Path, ignoring...", ros::this_node::getName().c_str());
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
            if ((active_path_test.GetVelocity() * upcoming_path_test.GetVelocity()) < 0.0) { break; }
            if (active_path_test.GetFrameId() != upcoming_path_test.GetFrameId()) { break; }
        }
        active_path_.push_back(upcoming_path_.front());
        upcoming_path_.pop_front();
        path_length += active_path_.back().GetPathLength();
    }
}

void NMPCNavControlROS::processNearestPoint(double& x, double& y, double& theta, double& theta_holonomic)
{
    parametric_trajectories_common::TPathProcessMinDist min_dist_opti(10, 0.01);
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
    pubCmdVel(true);
    current_status_ = Status::Idle;
}

void NMPCNavControlROS::processGoToPose()
{
    double dist_to_goal_pose = dist(goal_pose_.pose.position.x, goal_pose_.pose.position.y,
                                    robot_pose_.x, robot_pose_.y);
    if (enable_safe_conditions_ && (dist_to_goal_pose >= max_goal_pose_dist_)) {
        pubCmdVel(true);
        current_status_ = Status::Idle;
        ROS_WARN("[%s] Goal pose to far away from the robot, aborting...", ros::this_node::getName().c_str());
        return;
    }

    std::list<NMPCNavControl::Pose> goal_pose_array;
    NMPCNavControl::Pose pose;
    pose.x = goal_pose_.pose.position.x;
    pose.y = goal_pose_.pose.position.y;
    pose.theta = tf2::getYaw(goal_pose_.pose.orientation);
    goal_pose_array.push_back(pose);

    // Check end of trajectory
    double d = dist(robot_pose_.x, robot_pose_.y, pose.x, pose.y);
    double ang = normAngRad(robot_pose_.theta - pose.theta);
    if ((d <= final_position_error_) && (ang <= final_orientation_error_)) {
        pubCmdVel(true);
        current_status_ = Status::Idle;
        return;
    }

    executeNMPC(goal_pose_array);
}

void NMPCNavControlROS::processFollowPath()
{
    double x, y, theta, theta_holonomic;
    processNearestPoint(x, y, theta, theta_holonomic);
    processPathBuffers(active_path_u_);

    if (steering_geometry_ == kOmni4Str) { theta = theta_holonomic; }
    else if (active_path_.front().GetVelocity() < 0.0) { theta += M_PI; }
    double pos_error_to_path = dist(x, y, robot_pose_.x, robot_pose_.y);
    double ori_error_to_path = fabs(normAngRad(theta - robot_pose_.theta));
    if (enable_safe_conditions_ && ((pos_error_to_path >= max_pos_error_to_path_) ||
                                    (ori_error_to_path >= max_ori_error_to_path_))) {
        pubCmdVel(true);
        current_status_ = Status::Error;
        ROS_ERROR("[%s] Error to path too big, aborting...", ros::this_node::getName().c_str());
        return;
    }

    PathDiscretizer pathDiscretizer(mpc_control_->getDeltaTime(), mpc_control_->getHorizon()+1, false);
    std::vector<PathDiscretizer::Pose> next_poses;
    pathDiscretizer.getNextNPoses(active_path_, active_path_u_, next_poses);

    pubDebugDiscretizedPath(next_poses);

    std::list<NMPCNavControl::Pose> goal_pose_array;
    for (size_t i = 0; i < next_poses.size(); i++) {
        NMPCNavControl::Pose pose;
        pose.x = next_poses[i].x;
        pose.y = next_poses[i].y;
        pose.theta = next_poses[i].theta;
        goal_pose_array.push_back(pose);
    }

    // Check end of trajectory
    double d = dist(robot_pose_.x, robot_pose_.y, goal_pose_array.back().x, goal_pose_array.back().y);
    double ang = normAngRad(robot_pose_.theta - goal_pose_array.back().theta);
    if ((d <= final_position_error_) && (ang <= final_orientation_error_)) {
        if (upcoming_path_.size() == 0) { current_status_ = Status::Idle; }
        else {
            active_path_.pop_front();
            active_path_.push_back(upcoming_path_.front());
            upcoming_path_.pop_front();
        }

        pubCmdVel(true);
        return;
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
        if (steering_geometry_ == kTricStr) {
            auto* tric_control = dynamic_cast<NMPCNavControlTric*>(mpc_control_.get());
            tric_control->setSteeringWheelAngle(robot_steering_wheel_angle_);
        }
        double cpu_time;
        bool pub_vel = mpc_control_->run(robot_pose_, robot_vel_, goal_pose_array, *robot_vel_ref_, cpu_time);
        if (pub_vel) { pubCmdVel(); }
        ROS_DEBUG_NAMED("nmpc_solver", "CPU time: %lf ms", cpu_time);
    } catch (const std::exception& e) {
        ROS_ERROR("[%s] Exception caught: %s.", ros::this_node::getName().c_str(), e.what());
        current_status_ = Status::Error;
    }
}

void NMPCNavControlROS::pubDebugDiscretizedPath(std::vector<PathDiscretizer::Pose> poses)
{
    nav_msgs::Path debug_path_msg;
    debug_path_msg.header.stamp = ros::Time::now();
    debug_path_msg.header.frame_id = global_frame_id_;
    for (const auto& pose : poses) {
        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header = debug_path_msg.header;
        pose_stamped.pose.position.x = pose.x;
        pose_stamped.pose.position.y = pose.y;
        tf2::Quaternion q;
        q.setRPY(0, 0, pose.theta);
        pose_stamped.pose.orientation = tf2::toMsg(q);
        debug_path_msg.poses.push_back(pose_stamped);
    }
    pub_debug_discretized_path_.publish(debug_path_msg);
}

} // namespace nmpc_nav_control