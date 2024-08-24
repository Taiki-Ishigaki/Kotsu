#!/usr/bin/env python3.9
# -*- coding: utf-8 -*-
# 2024.08.17 Created by T.Ishigaki

from .struct import *
from .kinematics import *

class BvhRobot(BvhRobotStruct):
  state : RobotState 
  
  def __init__(self, joints_, bvh_, motions_, state_):
    self.joints = joints_
    self.bvh = bvh_
    self.motions = motions_
    self.state = state_
    self.robot_init()

  @staticmethod
  def init_from_bvh_file(bvh_file):
    joints, bvh = BvhRobotStruct.read_bvh_file(bvh_file)
    robot = BvhRobotStruct(joints)
    motions = BvhRobotMotion(bvh)
    state = BvhRobotState(robot)
    return BvhRobot(joints, bvh, motions, state)
  
  def kinematics_tree(self, joint, parent, motion_vec, state_data):
    frame = BvhKinematics.kinematics(joint, parent, motion_vec, state_data)  
          
    a = mr.SE3()
    a.set_matrix(frame)

    pos = a.pos()
    rot_vec = RobotState.mat_to_vec(a.rot())
        
    state_data.update([(joint.name + "_pos" , pos.tolist())])
    state_data.update([(joint.name + "_rot" , rot_vec.tolist())])
    
    for j_id in joint.children:
      self.kinematics_tree(self.joints[j_id], joint, motion_vec, state_data)
      

  def update_kinematics(self, motion_vec):
    state_data = {}
    self.kinematics_tree(self.joints[self.root_joint_id], None, motion_vec, state_data)

    self.state.import_state(state_data)