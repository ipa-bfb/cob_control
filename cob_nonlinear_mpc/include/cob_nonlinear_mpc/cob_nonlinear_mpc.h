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

#ifndef COB_NONLINEAR_MPC_COB_NONLINEAR_MPC_H
#define COB_NONLINEAR_MPC_COB_NONLINEAR_MPC_H

#include <ros/ros.h>

#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64MultiArray.h>
#include <nav_msgs/Odometry.h>

#include <urdf/model.h>

#include <kdl_parser/kdl_parser.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/jntarrayvel.hpp>
#include <kdl/frames.hpp>

#include <tf/transform_listener.h>
#include <tf/tf.h>
#include <visualization_msgs/MarkerArray.h>

#include <ctime>
#include <casadi/casadi.hpp>

#include <cob_nonlinear_mpc/nonlinear_mpc.h>

#include <boost/thread/mutex.hpp>
#include <boost/shared_ptr.hpp>

using namespace casadi;
using namespace std;

struct Robot
{
    int dof;
    urdf::Model urdf;
    KDL::Chain kinematic_chain;
    std::vector<KDL::Joint> joints;
    std::vector<KDL::Segment> forward_kinematics;
    bool base_active_;
};

struct DH
{
    double alpha;
    double a;
    std::string d;
    std::string theta;

    std::string type;
};

struct T_BVH
{
    SX T;
    SX BVH_p;
    string link;
    bool constraint = false;
};

class CobNonlinearMPC
{
private:
    ros::NodeHandle nh_;
    std::vector<std::string> transformation_names_;
    std::vector<std::string> transformation_names_base_;
    std::vector<std::string> joint_names;


    std::string chain_base_link_;
    std::string chain_tip_link_;

    int dof;
    urdf::Model model;
    Robot robot_;
    KDL::Chain chain_;

    DH dh_param;
    boost::shared_ptr<MPC> mpc_ctr_;
    std::vector<DH> dh_params;
    std::vector<DH> dh_params_base_;

    std::vector<SX> transformation_vector_dual_quat_;

    std::vector<SX> bvh_points_;

    std::vector<T_BVH> transform_vec_bvh_;
    std::vector<SX> transform_base_vec_;

//    std::vector<vector<SX>> bvh_matrix;
    std::map<string,vector<vector<SX>>> bvh_matrix;

    XmlRpc::XmlRpcValue scm_;
    XmlRpc::XmlRpcValue bvb_;

    std::unordered_map<std::string, std::vector<std::string> > self_collision_map_;
    vector<double> bvb_positions_;
    vector<double> bvb_radius_;

    visualization_msgs::MarkerArray marker_array_;

    // Declare variables
    SX u_;
    SX x_;
    SX fk_;
    SX fk_link4_;
    std::vector<SX> fk_vector_;
    SX fk_base_;

    SX fk_dual_quat_;
    SX p_;
    tf::TransformListener tf_listener_;

    ros::Subscriber jointstate_sub_;
    ros::Subscriber odometry_sub_;
    ros::Subscriber frame_tracker_sub_;

    ros::Publisher base_vel_pub_;
    ros::Publisher pub_;
    ros::Publisher marker_pub_;

    KDL::JntArray joint_state_;
    KDL::JntArray odometry_state_;
    KDL::Tree robot_tree_;
    vector<vector<double>> u_open_loop_;
    vector<vector<double>> x_open_loop_;

    vector<double> u_init_;


public:
    CobNonlinearMPC()
    {
//            u_init_ = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
        marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("nmpc/bvh", 1);
    }
    ~CobNonlinearMPC(){}


    bool initialize();

    void jointstateCallback(const sensor_msgs::JointState::ConstPtr& msg);
    void odometryCallback(const nav_msgs::Odometry::ConstPtr& msg);

    void FrameTrackerCallback(const geometry_msgs::Pose::ConstPtr& msg);
    KDL::JntArray getJointState();

    Eigen::MatrixXd mpc_step(const geometry_msgs::Pose pose,
                             const KDL::JntArray& state);

    Function create_integrator(const unsigned int state_dim, const unsigned int control_dim, const double T,
                               const unsigned int N, SX ode, SX x, SX u, SX L);

    void visualizeBVH(const geometry_msgs::Point point, double radius, int id);

    SX quaternion_product(SX q1, SX q2);
    SX dual_quaternion_product(SX q1, SX q2);

    bool process_KDL_tree();

    void generate_symbolic_forward_kinematics();

    void generate_bounding_volumes();
};


#endif  // COB_NONLINEAR_MPC_COB_NONLINEAR_MPC_H
