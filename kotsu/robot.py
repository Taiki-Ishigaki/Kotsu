#!/usr/bin/env python3.9
# -*- coding: utf-8 -*-
# 2024.07.21 Created by T.Ishigaki

import numpy as np

from pickle import FALSE
import xml.etree.ElementTree as ET

from kotsu.basic.robot_struct import *
from kotsu.kinematics.kinematics import *

from kotsu.gen_value import *
from kotsu.state import *
  
class Robot(RobotStruct):

  gen_value : RobotGenValue
  state : RobotState 

  def __init__(self, links_, joints_, gen_value_, state_):
    self.joints = joints_
    self.links = links_
    self.gen_value = gen_value_
    self.state = state_
    self.robot_init()

  @staticmethod
  def init_from_model_file(model_file_name):
    robot_et = ET.parse(model_file_name).getroot()
    links, joints = RobotStruct.read_model_file(robot_et)
    robot = RobotStruct(links, joints)
    gen_value = RobotGenValue(robot)
    state = RobotState(robot)
    return Robot(links, joints, gen_value, state)

  def import_gen_vecs(self, vecs):
    self.gen_value.import_vecs(self, vecs)
    
  def kinematics_tree(self, link, joint, gen_value, state_data):
    for link_id in joint.connect_link:
      if LinkStruct.link_id(link) != link_id or link == None:
        l = self.links[link_id]
        frame = LinkKinematics.kinematics(l, joint, link, gen_value, state_data)  
        veloc = LinkKinematics.vel_kinematics(l, joint, link, gen_value, state_data)  
        accel = LinkKinematics.acc_kinematics(l, joint, link, gen_value, state_data) 
      
        a = SE3()
        a.set_adj_mat(frame)

        pos = a.pos()
        rot_vec = RobotState.mat_to_vec(a.rot())
        
        state_data.update([(l.name + "_pos" , pos.tolist())])
        state_data.update([(l.name + "_rot" , rot_vec.tolist())])
        state_data.update([(l.name + "_vel" , veloc.tolist())])
        state_data.update([(l.name + "_acc" , accel.tolist())])

        for joint_id in l.connect_joint:
          if joint.id != joint_id:
            j = self.joints[joint_id]
            self.kinematics_tree(l, j, gen_value, state_data)    

  def update_kinematics(self):
    gen_value = self.gen_value.to_dict(-1)
    state_data = {}
    self.kinematics_tree(None, self.joints[self.root_joint_id], gen_value, state_data)

    self.state.import_state(state_data)