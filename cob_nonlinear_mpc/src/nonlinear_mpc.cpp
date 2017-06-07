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

#include <cob_nonlinear_mpc/nonlinear_mpc.h>

void MPC::init(){
    // Total number of NLP variables
    NV = state_dim_*(num_shooting_nodes_+1) +control_dim_*num_shooting_nodes_;

    vector<double> tmp;
    for(int k=0; k < num_shooting_nodes_; ++k)
    {
        tmp.clear();
        for(int i=0; i < control_dim_; ++i)
        {
            tmp.push_back(0);
        }
        u_open_loop_.push_back(tmp);
    }

    for(int k=1; k <= num_shooting_nodes_; ++k)
    {
        tmp.clear();

        for(int i=0; i < state_dim_; ++i)
        {
            tmp.push_back(0);
        }

        x_open_loop_.push_back(tmp);
    }

    for(int i = 0; i < control_dim_; i++)
    {
        ROS_INFO_STREAM("CONTROL DIM: "<<control_dim_);
        u_init_.push_back(0);
    }
}

int MPC::get_num_shooting_nodes(){
    return this->num_shooting_nodes_;
}

double MPC::get_time_horizon(){
    return this->time_horizon_;
}

int MPC::get_state_dim(){
    return this->state_dim_;
}

int MPC::get_control_dim(){
    return this->control_dim_;
}

void MPC::set_path_constraints(vector<double> state_path_constraints_min,vector<double> state_path_constraints_max){
    this->state_path_constraints_min_=state_path_constraints_min;
    this->state_path_constraints_max_=state_path_constraints_max;
}

void MPC::set_state_constraints(vector<double> state_terminal_constraints_min,vector<double> state_terminal_constraints_max){
    this->state_terminal_constraints_min_=state_terminal_constraints_min;
    this->state_terminal_constraints_max_=state_terminal_constraints_max;
}

void MPC::set_input_constraints(vector<double> input_constraints_min,vector<double> input_constraints_max){
    this->input_constraints_min_=input_constraints_min;
    this->input_constraints_max_=input_constraints_max;
}

void MPC::generate_symbolic_forward_kinematics(Robot* robot){
    // Casadi symbolics
    ROS_INFO("MPC::generate_symbolic_forward_kinematics");
    u_ = SX::sym("u", control_dim_);  // control
    x_ = SX::sym("x", state_dim_); // states

    SX T = SX::sym("T",4,4);
    if(robot->base_active_){
    ////generic rotation matrix around z and translation vector for x and y
        T(0,0) = cos(x_(2)); T(0,1) = -sin(x_(2));  T(0,2) = 0.0; T(0,3) = x_(0);
        T(1,0) = sin(x_(2)); T(1,1) = cos(x_(2));   T(1,2) = 0.0; T(1,3) = x_(1);
        T(2,0) = 0.0;        T(2,1) = 0.0;          T(2,2) = 1.0; T(2,3) = 0;
        T(3,0) = 0.0;        T(3,1) = 0.0;          T(3,2) = 0.0; T(3,3) = 1.0;
        fk_base_ = T; //
        ROS_INFO("Base forward kinematics");
    }
    int offset;

    for(int i=0;i<robot->joint_frames.size();i++){

        KDL::Vector pos;
        KDL::Rotation rot;
        rot=robot->joint_frames.at(i).M;
        pos=robot->joint_frames.at(i).p;
//#ifdef __DEBUG__
        ROS_WARN("Rotation matrix %f %f %f \n %f %f %f \n %f %f %f \n",rot(0,0),rot(0,1),rot(0,2),rot(1,0),rot(1,1),rot(1,2),rot(2,0),rot(2,1),rot(2,2));
        ROS_INFO_STREAM("Joint position of transformation"<< " X: " << pos.x()<< " Y: " << pos.y()<< " Z: " << pos.z());
//#endif
        if(robot->base_active_){ // if base active first initial control variable belong to the base
        // here each joint is considered to be revolute.. code needs to be updated for prismatic
        //rotation matrix of the joint * homogenic transformation matrix of the next joint relative to the previous
        //still needs to be improved... if the chain is composed by joint than not joint than joint again this is going to be wrong
            T(0,0) = rot(0,0)*cos(x_(i+3))+rot(0,1)*sin(x_(i+3));
            T(0,1) = -rot(0,0)*sin(x_(i+3))+rot(0,1)*cos(x_(i+3));
            T(0,2) = rot(0,2); T(0,3) = pos.x();
            T(1,0) = rot(1,0)*cos(x_(i+3))+rot(1,1)*sin(x_(i+3));
            T(1,1) = -rot(1,0)*sin(x_(i+3))+rot(1,1)*cos(x_(i+3));
            T(1,2) = rot(1,2); T(1,3) = pos.y();
            T(2,0) = rot(2,0)*cos(x_(i+3))+rot(2,1)*sin(x_(i+3));
            T(2,1) = -rot(2,0)*sin(x_(i+3))+rot(2,1)*cos(x_(i+3));
            T(2,2) = rot(2,2); T(2,3) = pos.z();
            T(3,0) = 0.0; T(3,1) = 0.0; T(3,2) = 0.0; T(3,3) = 1.0;
        }
        else{
            T(0,0) = rot(0,0)*cos(x_(i))+rot(0,1)*sin(x_(i));
            T(0,1) = -rot(0,0)*sin(x_(i))+rot(0,1)*cos(x_(i));
            T(0,2) = rot(0,2); T(0,3) = pos.x();
            T(1,0) = rot(1,0)*cos(x_(i))+rot(1,1)*sin(x_(i));
            T(1,1) = -rot(1,0)*sin(x_(i))+rot(1,1)*cos(x_(i));
            T(1,2) = rot(1,2); T(1,3) = pos.y();
            T(2,0) = rot(2,0)*cos(x_(i))+rot(2,1)*sin(x_(i));
            T(2,1) = -rot(2,0)*sin(x_(i))+rot(2,1)*cos(x_(i));
            T(2,2) = rot(2,2); T(2,3) = pos.z();
            T(3,0) = 0.0; T(3,1) = 0.0; T(3,2) = 0.0; T(3,3) = 1.0;
        }

        T_BVH p;
        p.link = robot->kinematic_chain.getSegment(i).getName();
        p.T = T;
        transform_vec_bvh_.push_back(p);
    }

    // Get Endeffector FK
    for(int i=0; i< transform_vec_bvh_.size(); i++)
    {
        if(robot->base_active_)
        {
            if(i==0)
            {
                ROS_WARN("BASE IS ACTIVE");
                fk_ = mtimes(fk_base_,transform_vec_bvh_.at(i).T);
            }
            else
            {
                fk_ = mtimes(fk_,transform_vec_bvh_.at(i).T);
            }
        }
        else
        {
            if(i==0)
            {
                fk_ = transform_vec_bvh_.at(i).T;
            }
            else
            {
                fk_ = mtimes(fk_,transform_vec_bvh_.at(i).T);
            }
        }
        fk_vector_.push_back(fk_); // stacks up multiplied transformation until link n
    }
}


Eigen::MatrixXd MPC::mpc_step(const geometry_msgs::Pose pose, const KDL::JntArray& state,Robot* robot)
{

    // Bounds and initial guess for the control
#ifdef __DEBUG__
    ROS_INFO_STREAM("input_constraints_min_: " <<this->input_constraints_min_.size());
    ROS_INFO_STREAM("input_constraints_max_: " <<this->input_constraints_max_.size());
#endif
    vector<double> u_min =  this->input_constraints_min_;
    vector<double> u_max  = this->input_constraints_max_;

    ROS_INFO("Bounds and initial guess for the state");
    vector<double> x0_min;
    vector<double> x0_max;
    vector<double> x_init;
    ROS_INFO_STREAM("state rows: " <<state.rows());
    for(unsigned int i=0; i < state.rows();i++)
    {
        x0_min.push_back(state(i));
        x0_max.push_back(state(i));
        x_init.push_back(state(i));
    }

    vector<double> x_min  = this->state_path_constraints_min_;
    vector<double> x_max  = this->state_path_constraints_max_;
    vector<double> xf_min = this->state_terminal_constraints_min_;
    vector<double> xf_max = this->state_terminal_constraints_max_;

    ROS_INFO("ODE right hand side and quadrature");
    SX qdot = SX::vertcat({u_});

    ROS_INFO("Current Quaternion and Position Vector.");
    /*double kappa = 0.001; // Small regulation term for numerical stability for the NLP

        SX q_c = SX::vertcat({
        0.5 * sqrt(fk_(0,0) + fk_(1,1) + fk_(2,2) + 1.0 + kappa),
        0.5 * (sign((fk_(2,1) - fk_(1,2)))) * sqrt(fk_(0,0) - fk_(1,1) - fk_(2,2) + 1.0 + kappa),
        0.5 * (sign((fk_(0,2) - fk_(2,0)))) * sqrt(fk_(1,1) - fk_(2,2) - fk_(0,0) + 1.0 + kappa),
        0.5 * (sign((fk_(1,0) - fk_(0,1)))) * sqrt(fk_(2,2) - fk_(0,0) - fk_(1,1) + 1.0 + kappa)
    });*/

    q_c = SX::vertcat({ //current quaternion
        0.5 * sqrt(fk_(0,0) + fk_(1,1) + fk_(2,2) + 1.0),
        0.5 * ((fk_(2,1) - fk_(1,2))) * sqrt(fk_(0,0) - fk_(1,1) - fk_(2,2) + 1.0),
        0.5 * ((fk_(0,2) - fk_(2,0))) * sqrt(fk_(1,1) - fk_(2,2) - fk_(0,0) + 1.0),
        0.5 * ((fk_(1,0) - fk_(0,1))) * sqrt(fk_(2,2) - fk_(0,0) - fk_(1,1) + 1.0)
    });

    pos_c = SX::vertcat({fk_(0,3), fk_(1,3), fk_(2,3)}); //current state

    ROS_INFO("Desired Goal-pose");
    pos_target = SX::vertcat({pose.position.x, pose.position.y, pose.position.z});
    q_target = SX::vertcat({pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z});

    // Prevent collision with Base_link
    SX barrier;
    SX dist;

    std::unordered_map<std::string, std::vector<std::string> >::iterator it_scm;

    int counter = 0;
    double bv_radius;

    for( it_scm = self_collision_map_.begin(); it_scm != self_collision_map_.end(); it_scm++)
    {
        std::vector<string> scm_collision_links = it_scm->second;
        for(int i=0; i<scm_collision_links.size(); i++)
        {
            ROS_WARN_STREAM(it_scm->first);
            vector<vector<SX>> p1_mat = bvh_matrix[it_scm->first];
            vector<vector<SX>> p2_mat = bvh_matrix[scm_collision_links.at(i)];

            for(int k=0; k<p1_mat.size(); k++)
            {
                if(it_scm->first == "body")
                {
                    bv_radius = bvb_radius_.at(k);
                }
                else
                {
                    bv_radius = 0.1;
                }

                vector<SX> p1_vec = p1_mat.at(k);
                for(int m=0; m<p2_mat.size(); m++)
                {
                    vector<SX> p2_vec = p2_mat.at(m);

                    SX p1 = SX::vertcat({p1_vec.at(0)});
                    SX p2 = SX::vertcat({p2_vec.at(0)});
                    dist = dot(p1 - p2, p1 - p2);

                    if(counter == 0)
                    {
                        barrier = exp((bv_radius - sqrt(dist))/0.01);
                        counter = 1;
                    }
                    else
                    {
                        barrier += exp((bv_radius - sqrt(dist))/0.01);
                    }
                }
            }
        }
    }

    // Get orientation error
    SX q_c_inverse = SX::vertcat({q_c(0), -q_c(1), -q_c(2), -q_c(3)});
    SX e_quat= quaternion_product(q_c_inverse,q_target);
    SX error_attitute = SX::vertcat({ e_quat(1), e_quat(2), e_quat(3)});

    // L2 norm of the control signal
    SX R = SX::scalar_matrix(control_dim_,1,1);;
    SX energy = dot(sqrt(R)*u_,sqrt(R)*u_);

    // L2 norm of the states
    std::vector<int> state_convariance(state_dim_,1);
    SX S = 0.01*SX::scalar_matrix(state_dim_,1,1);
    SX motion = dot(sqrt(S)*x_,sqrt(S)*x_);

    // Objective

    SX error=pos_c-pos_target;
    ROS_INFO_STREAM("POSITION ERROR:" << double(error(0)) << double(error(1)) <<double(error(2)));
    ROS_INFO_STREAM("POSITION TARGET:" << double(pos_target(0)) << double(pos_target(1)) <<double(pos_target(2)));
    ROS_INFO_STREAM("POSITION:" << double(pos_c(0)) << double(pos_c(1)) <<double(pos_c(2)));
    SX L = 10*dot(pos_c-pos_target,pos_c-pos_target) ;//+ energy + 10 * dot(error_attitute,error_attitute) + barrier;

    // Create Euler integrator function
    Function F = create_integrator(state_dim_, control_dim_, time_horizon_, num_shooting_nodes_, qdot, x_, u_, L);

    // Declare variable vector for the NLP
    MX V = MX::sym("V",NV);

    // NLP variable bounds and initial guess
    vector<double> v_min,v_max,v_init;

    // Offset in V
    int offset=0;

    // State at each shooting node and control for each shooting interval
    vector<MX> X, U;

    for(unsigned int k=0; k<num_shooting_nodes_; ++k)
    {
        // Local state
        X.push_back( V.nz(Slice(offset,offset+state_dim_)));

        if(k==0)
        {
            v_min.insert(v_min.end(), x0_min.begin(), x0_min.end());
            v_max.insert(v_max.end(), x0_max.begin(), x0_max.end());
        }
        else
        {
            v_min.insert(v_min.end(), x_min.begin(), x_min.end());
            v_max.insert(v_max.end(), x_max.begin(), x_max.end());
        }
        v_init.insert(v_init.end(), x_init.begin(), x_init.end());
        offset += state_dim_;

        // Local control via shift initialization
        U.push_back( V.nz(Slice(offset,offset+control_dim_)));
        v_min.insert(v_min.end(), u_min.begin(), u_min.end());
        v_max.insert(v_max.end(), u_max.begin(), u_max.end());

        v_init.insert(v_init.end(), u_init_.begin(), u_init_.end());
        offset += control_dim_;
    }

    // State at end
    X.push_back(V.nz(Slice(offset,offset+state_dim_)));
    v_min.insert(v_min.end(), xf_min.begin(), xf_min.end());
    v_max.insert(v_max.end(), xf_max.begin(), xf_max.end());
    v_init.insert(v_init.end(), u_init_.begin(), u_init_.end());
    offset += state_dim_;

    // Make sure that the size of the variable vector is consistent with the number of variables that we have referenced
    casadi_assert(offset==NV);

    // Objective function
    MX J = 0;

    //Constraint function and bounds
    vector<MX> g;

    // Loop over shooting nodes
    for(unsigned int k=0; k<num_shooting_nodes_; ++k)
    {
        // Create an evaluation node
        MXDict I_out = F( MXDict{ {"x0", X[k]}, {"p", U[k]} });

        // Save continuity constraints
        g.push_back( I_out.at("xf") - X[k+1] );

        // Add objective function contribution
        J += I_out.at("qf");
    }

    // NLP
    MXDict nlp = {{"x", V}, {"f", J}, {"g", vertcat(g)}};

    // Set options
    Dict opts;

    opts["ipopt.tol"] = 1e-4;
    opts["ipopt.max_iter"] = 20;
//    opts["ipopt.hessian_approximation"] = "limited-memory";
//    opts["ipopt.hessian_constant"] = "yes";
    opts["ipopt.linear_solver"] = "ma27";
    opts["ipopt.print_level"] = 0;
    opts["print_time"] = true;
    opts["expand"] = true;  // Removes overhead

    ROS_INFO("Create an NLP solver and buffers");
    Function solver = nlpsol("nlpsol", "ipopt", nlp, opts);

    std::map<std::string, DM> arg, res;

    ROS_INFO("Bounds and initial guess");
    arg["lbx"] = v_min;
    arg["ubx"] = v_max;
    arg["lbg"] = 0;
    arg["ubg"] = 0;
    arg["x0"] = v_init;

    ROS_INFO("Solve the problem");
    res = solver(arg);

    // Optimal solution of the NLP
    vector<double> V_opt(res.at("x"));
    vector<double> J_opt(res.at("f"));

    ROS_INFO("Get the optimal control");
    Eigen::VectorXd q_dot = Eigen::VectorXd::Zero(state_dim_);
//    Eigen::VectorXd x_new = Eigen::VectorXd::Zero(mpc_ctr_->get_state_dim());
    SX sx_x_new;
    u_open_loop_.clear();
    x_open_loop_.clear();
    x_new.clear();

    for(int i=0; i<1; ++i)  // Copy only the first optimal control sequence
    {
        for(int j=0; j<control_dim_; ++j)
        {
            q_dot[j] = V_opt.at(state_dim_ + j);
            x_new.push_back(V_opt.at(j));
        }
    }
    sx_x_new = SX::vertcat({x_new});

    // Safe optimal control sequence at time t_k and take it as inital guess at t_k+1

    ROS_INFO("Plot bounding volumes");
    geometry_msgs::Point point;
    point.x = 0;
    point.y = 0;
    point.z = 0;

    /*SX result;
    for( it_scm = self_collision_map_.begin(); it_scm != self_collision_map_.end(); it_scm++)
    {
        vector<string> tmp = it_scm->second;

        for(int i=0; i<tmp.size();i++)
        {
            vector<vector<SX>> SX_vec = bvh_matrix[tmp.at(i)];
            for(int k=0; k<SX_vec.size(); k++)
            {
                SX test = SX::horzcat({SX_vec.at(k).at(0)});
                Function tesssst = Function("test", {x_}, {test});
                result = tesssst(sx_x_new).at(0);
                point.x = (double)result(0);
                point.y = (double)result(1);
                point.z = (double)result(2);

                bv_radius = 0.1;
                visualizeBVH(point, bv_radius, i+i*tmp.size());
            }
        }


        vector<vector<SX>> SX_vec = bvh_matrix[it_scm->first];
        for(int k=0; k<SX_vec.size(); k++)
        {
            SX test = SX::horzcat({SX_vec.at(k).at(0)});
            Function tesssst = Function("test", {x_}, {test});
            result = tesssst(sx_x_new).at(0);
            point.x = (double)result(0);
            point.y = (double)result(1);
            point.z = (double)result(2);

            if(it_scm->first == "body")
            {
                bv_radius = bvb_radius_.at(k);
            }
            else
            {
                bv_radius = 0.1;
            }
            visualizeBVH(point, bv_radius, k+tmp.size()+SX_vec.size());
        }
    }*/

    KDL::Frame ef_pos = forward_kinematics(state);

    return q_dot;
}

KDL::Frame MPC::forward_kinematics(const KDL::JntArray& state){

    KDL::Frame ef_pos; //POsition of the end effector

    //SX p_base = SX::vertcat({fk_base_(0,3), fk_base_(1,3), fk_base_(2,3)});

    Function fk = Function("fk_", {x_}, {pos_c});
    //Function fk_base = Function("fk_base_", {x_}, {p_base});
    vector<double> x;
    for(unsigned int i=0; i < state.rows();i++)
    {
        x.push_back(state(i));
    }
    SX test_v = fk(SX::vertcat({x})).at(0);
    //SX test_base = fk_base(SX::vertcat({x_new})).at(0);

    ef_pos.p.x((double)test_v(0));
    ef_pos.p.y((double)test_v(1));
    ef_pos.p.z((double)test_v(2));
    ROS_INFO_STREAM("Joint values:" << x);
    ROS_WARN_STREAM("Current Position: \n" << ef_pos.p.x() << " "<< ef_pos.p.y() << " "<< ef_pos.p.z() << " ");
    //ROS_WARN_STREAM("Base Position: \n" << (double)test_base(0) << " "<< (double)test_base(1) << " "<< (double)test_base(2) << " ");
    ROS_WARN_STREAM("Target Position: \n" << pos_target);
    return ef_pos;
}

Function MPC::create_integrator(const unsigned int state_dim, const unsigned int control_dim, const double T,
                                            const unsigned int N, SX ode, SX x, SX u, SX L)
{
    // Euler discretize
    double dt = T/((double)N);

    Function f = Function("f", {x, u}, {ode, L});
//
//    f.generate("f");
//
//    // Compile the C-code to a shared library
//    string compile_command = "gcc -fPIC -shared -O3 f.c -o f.so";
//    int flag = system(compile_command.c_str());
//    casadi_assert_message(flag==0, "Compilation failed");
//
//    Function f_ext = external("f");

    MX X0 = MX::sym("X0", state_dim);
    MX U_ = MX::sym("U",control_dim);
    MX X_ = X0;
    MX Q = 0;

    vector<MX> input(2);
    input[0] = X_;
    input[1] = U_;
    MX qdot_new = f(input).at(0);
    MX Q_new = f(input).at(1);

    X_= X_+ dt * qdot_new;
    Q = Q + dt * Q_new;

    Function F = Function("F", {X0, U_}, {X_, Q}, {"x0","p"}, {"xf", "qf"});
    return F.expand("F");   // Remove overhead
}

SX MPC::dual_quaternion_product(SX q1, SX q2)
{
    SX q1_real = SX::vertcat({q1(0),q1(1),q1(2),q1(3)});
    SX q1_dual = SX::vertcat({q1(4),q1(5),q1(6),q1(7)});
    SX q2_real = SX::vertcat({q2(0),q2(1),q2(2),q2(3)});
    SX q2_dual = SX::vertcat({q2(4),q2(5),q2(6),q2(7)});

    SX q1q2_real = quaternion_product(q1_real,q2_real);
    SX q1_real_q2_dual = quaternion_product(q1_real,q2_dual);
    SX q1_dual_q2_real = quaternion_product(q1_dual,q2_real);

    SX q_prod = SX::vertcat({
        q1q2_real,
        q1_real_q2_dual + q1_dual_q2_real
    });

    return q_prod;
}

SX MPC::quaternion_product(SX q1, SX q2)
{
    SX q1_v = SX::vertcat({q1(1),q1(2),q1(3)});
    SX q2_v = SX::vertcat({q2(1),q2(2),q2(3)});

    SX c = SX::cross(q1_v,q2_v);

    SX q_new = SX::vertcat({
        q1(0) * q2(0) - dot(q1_v,q2_v),
        q1(0) * q2(1) + q2(0) * q1(1) + c(0),
        q1(0) * q2(2) + q2(0) * q1(2) + c(1),
        q1(0) * q2(3) + q2(0) * q1(3) + c(2)
    });

    return q_new;
}

void MPC::visualizeBVH(const geometry_msgs::Point point, double radius, int id)
{
    visualization_msgs::Marker marker;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.lifetime = ros::Duration();
    marker.action = visualization_msgs::Marker::ADD;
    marker.ns = "preview";
    marker.header.frame_id = "odom_combined";


    marker.scale.x = 2*radius;
    marker.scale.y = 2*radius;
    marker.scale.z = 2*radius;

    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 0.1;

    marker_array_.markers.clear();

    marker.id = id;
    marker.pose.position.x = point.x;
    marker.pose.position.y = point.y;
    marker.pose.position.z = point.z;
    marker_array_.markers.push_back(marker);

    marker_pub_.publish(marker_array_);
}

void MPC::generate_bounding_volumes(Robot* robot){
    // Get bounding volume forward kinematics
    ROS_INFO("MPC::generate_bounding_volumes");
        for(int i=0; i<transform_vec_bvh_.size(); i++)
        {
            T_BVH bvh = transform_vec_bvh_.at(i);
            std::vector<SX> bvh_arm;
            if(i-1<0)
            {
                SX transform = mtimes(fk_vector_.at(i),bvh.T);
                SX tmp = SX::vertcat({transform(0,3), transform(1,3), transform(2,3)});
                bvh_arm.push_back(tmp);
                bvh_matrix[bvh.link].push_back(bvh_arm);

                if(bvh.constraint)
                {
                    bvh_arm.clear();
                    tmp.clear();
                    transform = mtimes(fk_vector_.at(i),bvh.BVH_p);
                    tmp = SX::vertcat({transform(0,3), transform(1,3), transform(2,3)});
                    bvh_arm.push_back(tmp);
                    bvh_matrix[bvh.link].push_back(bvh_arm);
                }
            }
            else
            {
                bvh_arm.clear();
                SX transform = mtimes(fk_vector_.at(i-1),bvh.T);
                SX tmp = SX::vertcat({transform(0,3), transform(1,3), transform(2,3)});
                bvh_arm.push_back(tmp);
                bvh_matrix[bvh.link].push_back(bvh_arm);
                bvh_arm.clear();

                if(bvh.constraint)
                {
                    tmp.clear();
                    transform = mtimes(fk_vector_.at(i-1),bvh.BVH_p);
                    tmp = SX::vertcat({transform(0,3), transform(1,3), transform(2,3)});
                    bvh_arm.push_back(tmp);
                    bvh_matrix[bvh.link].push_back(bvh_arm);
                }
            }
        }
        if(robot->base_active_)
        {
            for(int i=0; i<bvb_positions_.size(); i++)
            {
                std::vector<SX> base_bvh;
                SX tmp = SX::vertcat({fk_base_(0,3), fk_base_(1,3), fk_base_(2,3)+bvb_positions_.at(i)});
                base_bvh.push_back(tmp);
                bvh_matrix["body"].push_back(base_bvh);
            }

        }
}