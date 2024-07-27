#!/usr/bin/env python3.9
# -*- coding: utf-8 -*-
# 2024.07.21 Created by T.Ishigaki

import numpy as np

from kotsu.link_struct import *
from kotsu.link_df import *
from kotsu.robot_struct import *
from kotsu.kinematics import *
  
class Robot(RobotStruct):

  gen_value : RobotGenValue
  state : RobotState 

  def __init__(self, joints_, links_, gen_value_, state_):
    self.joints = joints_
    self.links = links_
    self.gen_value = gen_value_
    self.state = state_
    self.robot_init()

  @staticmethod
  def init_from_model_file(xml_file):
    joints, links = RobotStruct.read_model_file(xml_file)
    gen_value = RobotGenValue(LinkGenDF(links))
    state = RobotState(LinkStateDF(links))
    return Robot(joints, links, gen_value, state)
  
  def import_gen_vecs(self, vecs):
    self.gen_value.import_vecs(self, vecs)

  def update_kinematics(self):
    data = {}

    for l in self.links:
      frame = LinkKinematics.kinematics(l, self.gen_value, self.state)
      veloc = LinkKinematics.vel_kinematics(l, self.gen_value, self.state)
      accel = LinkKinematics.acc_kinematics(l, self.gen_value, self.state)

      a = SE3()
      a.set_adj_mat(frame)

      pos = a.pos()
      rot = a.rot()
      rot_vec =  rot[0,:]
      rot_vec = np.append(rot_vec, rot[1,:])
      rot_vec = np.append(rot_vec, rot[2,:])
      
      data.update([(l.name + "_pos" , pos.tolist())])
      data.update([(l.name + "_rot" , rot_vec.tolist())])
      data.update([(l.name + "_vel" , veloc.tolist())])
      data.update([(l.name + "_acc" , accel.tolist())])
      
    self.state.import_state(data)