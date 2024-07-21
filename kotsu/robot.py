#!/usr/bin/env python3.9
# -*- coding: utf-8 -*-
# 2024.07.21 Created by T.Ishigaki

import numpy as np

from kotsu.link_struct import *
from kotsu.link_df import *

class RobotStruct:
  def __init__(self, links_):
    self.links = links_
    self.dof = 0
    self.cnct_mat = np.zeros((self.dof, self.dof))
    
    for l in self.links:
      self.dof += l.dof()
      for i in l.connection:
        self.cnct_mat[i] = 1
      
  def connectivity(self):
    return self.cnct_mat

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
    vec = np.zeros(self.dof)
    
    for l in self.robot.links:
      vec[l.dof_index : l.dof_index + l.dof] = self.df.df[l.name + "_" + name][0].to_numpy()
      
    return vec
    
  def set_gen_coord(self):
    return self.set_gen_vec(self, "coord", self.gen_coord)
  
  def set_gen_veloc(self):
    return self.set_gen_vec(self, "veloc", self.gen_veloc)
  
  def set_gen_accel(self):
    return self.set_gen_vec(self, "accel", self.gen_accel)
  
  def set_gen_force(self):
    return self.set_gen_vec(self, "force", self.gen_force)
  
class RobotState:
  robot : RobotStruct
  link_gen_df : LinkGenDF
  link_state_df : LinkStateDF 
  
  def __init__(self, robot_, gen_df, state_df):
    self.robot = robot_
    self.link_gen_df = gen_df
    self.link_state_df = state_df
    
  def link_pos(self, id):
    return self.state_df[self.links[id].name+"_pos"].to_numpy()
  
  def link_rot(self, id):
    _rot = np.zeros((3,3))
    r = self.state_df[self.links[id].name+"_rot"].to_numpy()
    _rot[0,0:3] = r[0:3]
    _rot[1,0:3] = r[3:6]
    _rot[2,0:3] = r[6:12]
    return _rot

  def link_vel(self, id):
    return self.state_df[self.links[id].name+"_vel"].to_numpy()

  def link_acc(self, id):
    return self.state_df[self.links[id].name+"_acc"].to_numpy()
    
  def link_frame(self, id):
    h = SE3(self.link_rot(id), self.link_pos(id))
    return h.matrix()

  def link_adj_frame(self, id):
    a = SE3(self.link_rot(id), self.link_pos(id))
    return a.adjoint()