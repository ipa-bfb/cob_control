/*!
 *****************************************************************
 * \file
 *
 * \note
 *   Copyright (c) 2017 \n
 *   Fraunhofer Institute for Manufacturing Engineering
 *   and Automation (IPA) \n\n
 *
 *****************************************************************
 *
 * \note
 *   Project name: care-o-bot
 * \note
 *   ROS stack name: cob_control
 * \note
 *   ROS package name: cob_nonlinear_mpc
 *
 * \author
 *   Author: Bruno Brito  email: Bruno.Brito@ipa.fraunhofer.de
 *   Christoph Mark, email: Christoph.Mark@ipa.fraunhofer.de
 *
 * \date Date of creation: April, 2017
 *
 * \brief
 *
 *
 ****************************************************************/
#include <string>
#include <vector>
#include <limits>
#include <ros/ros.h>

#include <cob_nonlinear_mpc/cob_nonlinear_mpc.h>

#include <kdl_conversions/kdl_msg.h>
#include <tf/transform_datatypes.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <cob_srvs/SetString.h>
#include <string>
#include <Eigen/Dense>
#include <casadi/core/function/sx_function.hpp>
#include <stdlib.h>

bool CobNonlinearMPC::initialize()
{
    ros::NodeHandle nh_nmpc("nmpc");
    ros::NodeHandle nh_nmpc_dh("nmpc/dh");
    ros::NodeHandle nh_nmpc_base_dh("nmpc/base/dh");
    ros::NodeHandle nh_nmpc_constraints("nmpc/constraints");

    // JointNames
    if (!nh_.getParam("joint_names", joint_names))
    {
        ROS_ERROR("Parameter 'joint_names' not set");
        return false;
    }
    // nh_nmpc
    int num_shooting_nodes;
    if (!nh_nmpc.getParam("shooting_nodes", num_shooting_nodes))
    {
        ROS_ERROR("Parameter 'num_shooting_nodes_' not set");
        return false;
    }
    double time_horizon;
    if (!nh_nmpc.getParam("time_horizon", time_horizon))
    {
        ROS_ERROR("Parameter 'time_horizon' not set");
        return false;
    }
    int state_dim;
    if (!nh_nmpc.getParam("state_dim", state_dim))
    {
        ROS_ERROR("Parameter 'state_dim' not set");
        return false;
    }
    int control_dim;
    if (!nh_nmpc.getParam("control_dim", control_dim))
    {
        ROS_ERROR("Parameter 'control_dim' not set");
        return false;
    }

    mpc_ctr_.reset(new MPC(num_shooting_nodes,time_horizon ,state_dim,control_dim));
    if (!nh_nmpc.getParam("base/base_active", robot_.base_active_))
    {
        ROS_ERROR("Parameter 'base/base_active' not set");
        return false;
    }

    // nh_nmpc_constraints
    vector<double> state_path_constraints_min;
    if (!nh_nmpc_constraints.getParam("state/path_constraints/min", state_path_constraints_min))
    {
        ROS_ERROR("Parameter 'state/path_constraints/min' not set");
        return false;
    }
    vector<double> state_path_constraints_max;
    if (!nh_nmpc_constraints.getParam("state/path_constraints/max", state_path_constraints_max))
    {
        ROS_ERROR("Parameter 'state/path_constraints/max' not set");
        return false;
    }
    mpc_ctr_->set_path_constraints(state_path_constraints_min,state_path_constraints_max);

    vector<double> state_terminal_constraints_min;
    if (!nh_nmpc_constraints.getParam("state/terminal_constraints/min", state_terminal_constraints_min))
    {
        ROS_ERROR("Parameter 'state/terminal_constraints/min' not set");
        return false;
    }
    vector<double> state_terminal_constraints_max;
    if (!nh_nmpc_constraints.getParam("state/terminal_constraints/max", state_terminal_constraints_max))
    {
        ROS_ERROR("Parameter 'state/terminal_constraints/max' not set");
        return false;
    }
    mpc_ctr_->set_state_constraints(state_terminal_constraints_min,state_terminal_constraints_max);
    vector<double> input_constraints_min;
    if (!nh_nmpc_constraints.getParam("input/input_constraints/min", input_constraints_min))
    {
        ROS_ERROR("Parameter 'input/input_constraints/min' not set");
        return false;
    }
    vector<double> input_constraints_max;
    if (!nh_nmpc_constraints.getParam("input/input_constraints/max", input_constraints_max))
    {
        ROS_ERROR("Parameter 'input/input_constraints/max' not set");
        return false;
    }
    mpc_ctr_->set_input_constraints(input_constraints_min,input_constraints_max);

    if (!nh_nmpc_constraints.getParam("min_distance", min_dist))
    {
            ROS_ERROR("Parameter 'input/input_constraints/min' not set");
            return false;
    }

    // Chain
    if (!nh_.getParam("chain_base_link", chain_base_link_))
    {
        ROS_ERROR("Parameter 'chain_base_link' not set");
        return false;
    }

    if (!nh_.getParam("chain_tip_link", chain_tip_link_))
    {
        ROS_ERROR("Parameter 'chain_tip_link' not set");
        return false;
    }

    this->process_KDL_tree();

    mpc_ctr_->init();

    mpc_ctr_->generate_symbolic_forward_kinematics(&robot_);

    mpc_ctr_->generate_bounding_volumes(&robot_);

    joint_state_ = KDL::JntArray(robot_.kinematic_chain.getNrOfJoints());
    jointstate_sub_ = nh_.subscribe("joint_states", 1, &CobNonlinearMPC::jointstateCallback, this);

    if(robot_.base_active_){
        odometry_state_ = KDL::JntArray(3);
        odometry_sub_ = nh_.subscribe("base/odometry", 1, &CobNonlinearMPC::odometryCallback, this);
        base_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("base/command", 1);
    }

    frame_tracker_sub_ = nh_.subscribe("command_pose", 1, &CobNonlinearMPC::FrameTrackerCallback, this);
    pub_ = nh_.advertise<std_msgs::Float64MultiArray>("joint_group_velocity_controller/command", 1);

    ROS_WARN_STREAM(nh_.getNamespace() << "/NMPC...initialized!");
    return true;
}

void CobNonlinearMPC::FrameTrackerCallback(const geometry_msgs::Pose::ConstPtr& msg)
{
    KDL::JntArray state = getJointState();

    Eigen::MatrixXd qdot = mpc_ctr_->mpc_step(*msg, state,&robot_);

    geometry_msgs::Twist base_vel_msg;
    std_msgs::Float64MultiArray vel_msg;

    if(robot_.base_active_){
        base_vel_msg.linear.x = qdot(0);
        base_vel_msg.linear.y = qdot(1);
        base_vel_msg.linear.z = 0;
        base_vel_msg.angular.x = 0;
        base_vel_msg.angular.y = 0;
        base_vel_msg.angular.z = qdot(2);

        base_vel_pub_.publish(base_vel_msg);

        for (unsigned int i = 3; i < joint_state_.rows()+3; i++)
        {
            vel_msg.data.push_back(qdot(i));
        }
        pub_.publish(vel_msg);

    }
    else{
        for (unsigned int i = 0; i < joint_state_.rows(); i++)
        {
            vel_msg.data.push_back(qdot(i));
        }
        pub_.publish(vel_msg);
    }
}


void CobNonlinearMPC::jointstateCallback(const sensor_msgs::JointState::ConstPtr& msg)
{

    KDL::JntArray q_temp = joint_state_;

    int count = 0;

    for (uint16_t j = 0; j < joint_state_.rows(); j++)
    {
        for (uint16_t i = 0; i < msg->name.size(); i++)
        {
            if (strcmp(msg->name[i].c_str(), joint_names[j].c_str()) == 0)
            {

                q_temp(j) = msg->position[i];
                count++;
                break;
            }
        }
    }
    joint_state_ = q_temp;
}

void CobNonlinearMPC::odometryCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    KDL::JntArray temp = odometry_state_;
    KDL::Frame odom_frame_bl;
    tf::StampedTransform odom_transform_bl;

    temp(0) = msg->pose.pose.position.x;
    temp(1) = msg->pose.pose.position.y;
    temp(2) = msg->pose.pose.orientation.z;

    odometry_state_ = temp;
}


KDL::JntArray CobNonlinearMPC::getJointState()
{
    KDL:: JntArray tmp(joint_state_.rows() + odometry_state_.rows());
//    KDL:: JntArray tmp(joint_state_.rows());

    ROS_INFO("STATE SIZE: %i Odometry State SIze %i", tmp.rows(), odometry_state_.rows());

    for(int i = 0; i < odometry_state_.rows(); i++)
    {
        tmp(i) = odometry_state_(i);
    }

    for(int i = 0 ; i < joint_state_.rows(); i++)
    {
        tmp(i+odometry_state_.rows()) = this->joint_state_(i);
    }

    return tmp;
}
