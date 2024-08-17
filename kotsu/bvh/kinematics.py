#!/usr/bin/env python3.9
# -*- coding: utf-8 -*-
# 2024.08.14 Created by T.Ishigaki

import numpy as np

import mathrobo as mr

from .struct import *

class BvhKinematics:
  @staticmethod
  def joint_rel_frame(joint, motinos):
    rel_pos = np.array(joint.offset)
    rel_rot = np.identity(3)    
    vec =  motinos[joint.dof_index : joint.dof_index+joint.dof]
    for i in range(joint.dof):
      if joint.channels[i] == 'Xposition':
        rel_pos[0] = vec[i]
      elif joint.channels[i] == 'Yposition':
        rel_pos[1] = vec[i]
      elif joint.channels[i] == 'Zposition':
        rel_pos[2] = vec[i]
      elif joint.channels[i] == 'Xrotation':
        rel_rot = mr.euler_x(vec[i]) @ rel_rot
      elif joint.channels[i] == 'Yrotation':
        rel_rot = mr.euler_y(vec[i]) @ rel_rot
      elif joint.channels[i] == 'Zrotation':
        rel_rot = mr.euler_z(vec[i]) @ rel_rot
    
    rel_frame = np.identity(4)
    rel_frame[0:3,3] = rel_pos
    rel_frame[0:3,0:3] = rel_rot
    return rel_frame

  @staticmethod
  def kinematics(joint, parent, motinos, state):
    if parent:
      rot = RobotState.vec_to_mat(state[parent.name + "_rot"])
      h = mr.SE3(rot, state[parent.name + "_pos"])
      p_frame = h.matrix()
    else:
      p_frame = np.identity(4)
    rel_frame = BvhKinematics.joint_rel_frame(joint, motinos)
    frame = p_frame @ rel_frame
    return frame