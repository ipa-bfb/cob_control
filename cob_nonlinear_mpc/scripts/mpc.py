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
import threading

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
        self.init(ns)
        self.rate = rospy.Rate(10)  # 10hz
        self.thread = threading.Thread(target=self.mpc_step())
        self.thread.start()

    def init(self, ns):
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

        if rospy.has_param(ns + '/frame_tracker/target_frame'):
            self.tracking_frame = rospy.get_param(ns+'/frame_tracker/target_frame')
        else:
            rospy.logwarn('Could not find parameter frame_tracker/target_frame.')
            exit(-2)
        rospy.loginfo("MPC Initialized...")

    def mpc_step(self):
        while not rospy.is_shutdown():
            print("MPC_step")
            self.rate.sleep()