#!/usr/bin/env python3.9
# -*- coding: utf-8 -*-
# 2024.07.21 Created by T.Ishigaki

import numpy as np

from kotsu.link_struct import *
from kotsu.link_df import *

class RobotStruct:
  def __init__(self, links_):
    self.links = links_
    self.link_num = len(links_)    

    self.cnct_mat = np.zeros((self.link_num, self.link_num))

    self.dof = 0
    for l in self.links:
      self.dof += l.dof
      for i in l.connection:
        self.cnct_mat[l.id,i] = 1
      
  def connectivity(self):
    return self.cnct_mat

  @staticmethod
  def read_model_file(xml_data):
    robot = ET.fromstring(xml_data)

    links = []
    dof_index = 0

    for i in range(len(robot)):
      links.append(LinkStruct())

      links[i].name = robot[i].attrib.get('name')
      links[i].joint_type = robot[i].attrib.get('joint_type')
      links[i].link_type = robot[i].attrib.get('link_type')
      cnct_list = robot[i].attrib.get('connection')
      links[i].connection = [int(cnct) for cnct in cnct_list]
      links[i].connect_pos = np.array(eval(robot[i].attrib.get('connect_pos')))
      links[i].connect_rot = np.array(eval(robot[i].attrib.get('connect_rot')))
      links[i].cog = np.array(eval(robot[i].attrib.get('cog')))
      links[i].mass = eval(robot[i].attrib.get('mass'))
      links[i].inertia_param = np.array(eval(robot[i].attrib.get('inertia_param')))
      
      links[i].id = i

      links[i].set_dof()
      links[i].set_connent_frame()
      links[i].set_connent_adj_frame()

      links[i].dof_index = dof_index
      dof_index += links[i].dof

    return RobotStruct(links)

class RobotGenValue:
  robot : RobotStruct
  df : LinkGenDF
  gen_coord : np.ndarray = np.array([])
  gen_veloc : np.ndarray = np.array([])
  gen_accel : np.ndarray = np.array([])
  gen_force : np.ndarray = np.array([])
  
  def __init__(self, robot_, link_gen_df):
    self.robot = robot_
    self.df = link_gen_df
    
  def set_gen_vec(self, name, vec):
    vec = np.zeros(self.robot.dof)
    
    for l in self.robot.links:
      vec[l.dof_index : l.dof_index + l.dof] = self.df.df[l.name + "_" + name][-1].to_numpy()
      
    return vec
    
  def set_gen_coord(self):
    return self.set_gen_vec("coord", self.gen_coord)
  
  def set_gen_veloc(self):
    return self.set_gen_vec("veloc", self.gen_veloc)
  
  def set_gen_accel(self):
    return self.set_gen_vec("accel", self.gen_accel)
  
  def set_gen_force(self):
    return self.set_gen_vec("force", self.gen_force)
  
class RobotState:
  robot : RobotStruct
  link_gen_df : LinkGenDF
  link_state_df : LinkStateDF 
  
  def __init__(self, robot_, gen_df, state_df):
    self.robot = robot_
    self.link_gen_df = gen_df
    self.link_state_df = state_df
    
  def link_state_vec(self, name, id):
    return self.link_state_df.df[self.robot.links[id].name+"_"+name][-1].to_numpy()
    
  def link_state_mat(self, name, id):
    mat_vec = self.link_state_df.df[self.robot.links[id].name+"_"+name][-1].to_numpy()
    nn = len(mat_vec)
    n = int(np.sqrt(nn))

    mat = np.zeros((n,n))
    for i in range(n):
      mat[i,0:n] = mat_vec[n*i:n*i+n]
    return mat    
  
  def all_state_vec(self, name):
    labels = []
    for l in self.robot.links:
      labels.append(l.name+"_"+name) 
    mat = [self.link_state_df.df[label][-1].to_list() for label in labels]
    return np.array(mat)
  
  def link_pos(self, id):
    return self.link_state_vec("pos", id)
  
  def all_link_pos(self):
    return self.all_state_vec("pos")
  
  def link_rot(self, id):
    return self.link_state_mat("rot", id)

  def link_vel(self, id):
    return self.link_state_vec("vel", id)

  def link_acc(self, id):
    return self.link_state_vec("acc", id)
    
  def link_frame(self, id):
    h = SE3(self.link_rot(id), self.link_pos(id))
    return h.matrix()

  def link_adj_frame(self, id):
    a = SE3(self.link_rot(id), self.link_pos(id))
    return a.adjoint()