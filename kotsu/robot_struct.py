#!/usr/bin/env python3.9
# -*- coding: utf-8 -*-
# 2024.07.21 Created by T.Ishigaki

import numpy as np

from kotsu.link_struct import *
from kotsu.link_df import *

class RobotStruct:
  def __init__(self, links_):
    self.links = links_

    self.robot_init()

  def robot_init(self):
    self.link_num = len(self.links)  

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
      cnct_list = eval(robot[i].attrib.get('connection'))
      links[i].connection = [int(cnct) for cnct in cnct_list]
      links[i].connect_pos = np.array(eval(robot[i].attrib.get('connect_pos')))
      links[i].connect_rot = np.array(eval(robot[i].attrib.get('connect_rot')))
      links[i].cog = np.array(eval(robot[i].attrib.get('cog')))
      links[i].mass = eval(robot[i].attrib.get('mass'))
      links[i].inertia_param = np.array(eval(robot[i].attrib.get('inertia_param')))
      
      links[i].id = i

      links[i].init()

      links[i].dof_index = dof_index
      dof_index += links[i].dof

    return links

class RobotGenValue:
  _df : LinkGenDF
  coord : np.ndarray = np.array([])
  veloc : np.ndarray = np.array([])
  accel : np.ndarray = np.array([])
  force : np.ndarray = np.array([])
  
  def __init__(self, link_gen_df):
    self._df = link_gen_df
    
  def df(self):
    return self._df.df
    
  def export_vec(self, robot, name, vec):
    vec = np.zeros(robot.dof)
    
    for l in robot.links:
      vec[l.dof_index : l.dof_index + l.dof] = self.df()[l.name + "_" + name][-1].to_numpy()
      
    return vec
  
  def link_gen_value(self, l, vec):
    if(l.dof > 0):
      return vec[l.dof_index:l.dof_index+l.dof]
    else:
      return []
  
  def import_vecs(self, robot, vecs):
    data = {}
    for l in robot.links:
      for i in range(len(vecs)):
        data.update([(l.name + "_" + self._df.aliases[i] , self.link_gen_value(l, vecs[i]))])

    self._df.add_row(data)

  def export_coord(self, robot):
    return self.export_vec(robot, "coord", self.coord)
  
  def export_veloc(self, robot):
    return self.export_vec(robot, "veloc", self.veloc)
  
  def export_accel(self, robot):
    return self.export_vec(robot, "accel", self.accel)
  
  def export_force(self, robot):
    return self.export_vec(robot, "force", self.force)
  
class RobotState:
  _df : LinkStateDF 
  
  def __init__(self, state_df):
    self._df = state_df
    
  def df(self):
    return self._df.df
    
  def link_state_vec(self, robot, name, id):
    return self.df()[robot.links[id].name+"_"+name][-1].to_numpy()
    
  def link_state_mat(self, robot, name, id):
    mat_vec = self.df()[robot.links[id].name+"_"+name][-1].to_numpy()
    nn = len(mat_vec)
    n = int(np.sqrt(nn))

    mat = np.zeros((n,n))
    for i in range(n):
      mat[i,0:n] = mat_vec[n*i:n*i+n]
    return mat    
  
  def all_state_vec(self, robot, name):
    labels = []
    for l in robot.links:
      labels.append(l.name+"_"+name) 
    mat = [self.df()[label][-1].to_list() for label in labels]
    return np.array(mat)
  
  def link_pos(self, robot, id):
    return self.link_state_vec(robot, "pos", id)
  
  def all_link_pos(self, robot):
    return self.all_state_vec(robot, "pos")
  
  def link_rot(self, robot, id):
    return self.link_state_mat(robot, "rot", id)

  def link_vel(self, robot, id):
    return self.link_state_vec(robot, "vel", id)

  def link_acc(self, robot, id):
    return self.link_state_vec(robot, "acc", id)
    
  def link_frame(self, robot, id):
    h = SE3(self.link_rot(robot, id), self.link_pos(robot, id))
    return h.matrix()

  def link_adj_frame(self, robot, id):
    a = SE3(self.link_rot(robot, id), self.link_pos(robot, id))
    return a.adjoint()