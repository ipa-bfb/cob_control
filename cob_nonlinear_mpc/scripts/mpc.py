#!/usr/bin/env python
"""
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
 *   ROS package name: cob_nmpc_controller
 *
 * \author
 *   Author: Bruno Brito, email: Bruno.Brito@ipa.fraunhofer.de
 *
 * \date Date of creation: July, 2017
 *
 * \brief
 *
 *
"""
from casadi import *
from setuptools.command.saveopts import saveopts

import rospy
import matplotlib.pyplot as plt
from casadi import *
import thread
import numpy as np
import PyKDL
from urdf_parser_py.urdf import URDF
import PyKDL
from kdl_parser_py.urdf import treeFromUrdfModel
from urdf_parser_py.urdf import Robot
from kdl import Kinematics

class StateConstraints:
    path_constraints_min = 0.0
    path_constraints_max = 0.0
    terminal_constraints_min = 0.0
    terminal_constraints_max = 0.0
    input_constraints_min = 0.0
    input_constraints_max = 0.0

    def __init__(self):
        print("Created state constraints")


class MPC(object):

    def __init__(self, ns):
        self.joint_names = []
        self.shooting_nodes = 0
        self.time_horizon = 0.0
        self.state_dim =0
        self.control_dim = 0
        self.base_active = False
        self.state_constraints_ = StateConstraints()
        self.chain_tip_link = ""
        self.chain_base_link = ""
        self.tracking_frame = ""
        self.join_state_ = np.zeros((self.state_dim,1))
        self.rate = rospy.Rate(100)  # 10hz
        self.thread = None
        #SYBOLIC VARIABLES
        self.x = None
        self.u = None
        self.xdot = None
        self.opts = {"ipopt.linear_solver": "ma27"}
        #SYMBOLIC FUNCTIONS
        self.FK = None

        #KDL
        self.robot = None
        self.kdl_kin = None

        self.init(ns)

    def init(self, ns):

        print 'Initializing MPC...'

        if rospy.has_param(ns + '/joint_names'):
            self.joint_names = rospy.get_param(ns + '/joint_names')
        else:
            rospy.logerr('Parameter joint_names not set');
            # rospy.logwarn('Could not find parameter ~base_dir. Using default base_dir: ' + base_dir)

        if rospy.has_param(ns + '/nmpc/shooting_nodes'):
            self.shooting_nodes = rospy.get_param(ns + '/nmpc/shooting_nodes')
        else:
            rospy.logerr('Parameter shooting_nodes not set')

        if rospy.has_param(ns + '/nmpc/time_horizon'):
            self.time_horizon = rospy.get_param(ns + '/nmpc/time_horizon')
        else:
            rospy.logerr('Parameter time_horizon not set')

        if rospy.has_param(ns + '/nmpc/state_dim'):
            self.state_dim = rospy.get_param(ns + '/nmpc/state_dim')
            self.join_state_ = np.zeros((self.state_dim, 1))
        else:
            rospy.logerr('Parameter state_dim not set')

        if rospy.has_param(ns + '/nmpc/control_dim'):
            self.control_dim = rospy.get_param(ns + '/nmpc/control_dim')
        else:
            rospy.logerr('Parameter control_dim not set')

        if rospy.has_param(ns + '/nmpc/base/base_active'):
            self.base_active = rospy.get_param(ns + '/nmpc/base/base_active')
        else:
            rospy.logerr('Parameter base/base_active not set')

        if rospy.has_param(ns + '/nmpc/constraints/state/path_constraints/min'):
            self.state_constraints_.path_constraints_min = rospy.get_param(ns + '/nmpc/constraints/state/path_constraints/min')
        else:
            rospy.logerr('Parameter state/path_constraints/min not set')

        if rospy.has_param(ns + '/nmpc/constraints/state/path_constraints/max'):
            self.state_constraints_.input_constraints_max = rospy.get_param(ns + '/nmpc/constraints/state/path_constraints/max')
        else:
            rospy.logerr('Parameter state/path_constraints/max not set')

        if rospy.has_param(ns + '/nmpc/constraints/state/terminal_constraints/min'):
            self.state_constraints_.terminal_constraints_min = rospy.get_param(ns + '/nmpc/constraints/state/terminal_constraints/min')
        else:
            rospy.logerr('Parameter state/terminal_constraints/min not set')

        if rospy.has_param(ns + '/nmpc/constraints/state/terminal_constraints/max'):
            self.state_constraints_.terminal_constraints_max = rospy.get_param(ns + '/nmpc/constraints/state/terminal_constraints/max')
        else:
            rospy.logerr('Parameter state/terminal_constraints/max not set')

        if rospy.has_param(ns + '/nmpc/constraints/input/input_constraints/min'):
            self.state_constraints_.input_constraints_min = rospy.get_param(ns + '/nmpc/constraints/input/input_constraints/min')
        else:
            rospy.logerr('Parameter input/input_constraints/min not set')

        if rospy.has_param(ns + '/nmpc/constraints/input/input_constraints/max'):
            self.state_constraints_.input_constraints_max = rospy.get_param(ns + '/nmpc/constraints/input/input_constraints/max')
        else:
            rospy.logerr('Parameter input/input_constraints/max not set')

        if rospy.has_param(ns + '/chain_tip_link'):
            self.chain_tip_link = rospy.get_param(ns +'/chain_tip_link')
        else:
            rospy.logwarn('Could not find parameter chain_tip_link.')
            exit(-1)

        if rospy.has_param(ns + '/chain_base_link'):
            self.chain_base_link = rospy.get_param(ns +'/chain_base_link')
        else:
            rospy.logwarn('Could not find parameter chain_base_link.')
            exit(-1)

        if rospy.has_param(ns + '/root_frame'):
            self.root_frame = rospy.get_param(ns +'/root_frame')
        else:
            rospy.logwarn('Could not find parameter root_frame.')
            exit(-1)

        if rospy.has_param(ns + '/frame_tracker/target_frame'):
            self.tracking_frame = rospy.get_param(ns+'/frame_tracker/target_frame')
        else:
            rospy.logwarn('Could not find parameter frame_tracker/target_frame.')
            exit(-2)

        self.robot = Robot.from_parameter_server()
        print 'Initializing Kinematic Chain...'
        self.kdl_kin= Kinematics(self.robot, self.chain_base_link, self.chain_tip_link,self.root_frame)
        print 'Done initializing Kinematic Chain from ' + self.chain_base_link + ' to ' + self.chain_tip_link + ' in the ' + self.root_frame

        #print 'SYBOLIC VARIABLES'
        self.x = SX.sym('q',self.state_dim,1) #State
        self.u = SX.sym('u',self.control_dim,1)  # Control

        # Model equations
        self.xdot = self.u

        print 'Symbolic forward kinematics'
        self.fk = self.kdl_kin.symbolic_fk(self.x,base_active=self.base_active)
        self.fk = self.kdl_kin.forward2(self.join_state_, self.chain_base_link,self.chain_tip_link)
        self.J = self.kdl_kin.compute_jacobian_sym(self.x)
        print
        self.FK = Function('f', [self.x],[self.fk])
        self.Jacobian = Function('J', [self.x], [self.J])

        print (self.Jacobian(self.join_state_))
        print self.kdl_kin.compute_jacobian(self.join_state_)

        self.rate = 10
        rospy.loginfo("MPC Initialized...")

    def mpcStep(self,ref):
        print("MPC_step")
        pos_c =  self.fk[0:3,3]
        pos_ref = [ref.x,ref.y,ref.z]
        e=pos_ref-pos_c
        print e
        #print 'quadratic error'
        L = dot(e,e)
        #print 'Integrator...'
        I = self.create_integrator(L)

        #print 'Start with an empty NLP'
        w = []
        w0 = []
        lbw = []
        ubw = []
        J = 0
        g = []
        lbg = []
        ubg = []

        # Formulate the NLP
        Xk = MX(self.join_state_)

        for k in range(5):
            #print 'New NLP variable for the control'
            Uk = MX.sym('U_' + str(k))
            w += [Uk]
            lbw += [-3]
            ubw += [3]
            w0 += [0]

            #print 'Integrate till the end of the interval'
            Ik = I(x0=Xk, p=Uk)
            Xk = Ik['xf']
            J = J + Ik['qf']

            #print 'Add inequality constraint'
            g += [Xk[0]]
            lbg += [-3]
            ubg += [3]

        print vertcat(*g)
        print 'Create an NLP solver'
        prob = {'f': J, 'x': vertcat(*w), 'g': vertcat(*g)}
        solver = nlpsol('solver', 'ipopt', prob);

        print 'Pass bounds, initial guess and solve NLP'
        sol = solver(lbx=lbw,  # Lower variable bound
                     ubx=ubw,  # Upper variable bound
                     lbg=lbg,  # Lower constraint bound
                     ubg=ubg,  # Upper constraint bound
                     x0=w0)  # Initial guess
        w_opt = sol['x']

        return w_opt

    def spin(self):
        self.thread=thread.start_new_thread(name="mpc", target=self.mpcStep())
        return

    def create_integrator(self,L):
        # Formulate discrete time dynamics
        # CVODES from the SUNDIALS suite
        dae = {'x': self.x, 'p': self.u, 'ode': self.xdot, 'quad': L}
        opts = {'tf': self.time_horizon / self.shooting_nodes}
        I = integrator('F', 'cvodes', dae, opts)
        return I