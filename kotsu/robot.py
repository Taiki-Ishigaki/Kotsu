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
  gen_coord : np.ndarray = np.array([])
  gen_veloc : np.ndarray = np.array([])
  gen_accel : np.ndarray = np.array([])
  gen_force : np.ndarray = np.array([])
  
  def __init__(self, robot_, link_gen_df):
    self.r = robot_
    self.df = link_gen_df
    
  def set_gen_vec(self, name, vec):
    vec = np.zeros(self.dof)
    
    for l in self.r.links:
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