#!/usr/bin/env python3.9
# -*- coding: utf-8 -*-
# 2024.07.21 Created by T.Ishigaki

import numpy as np

from pickle import FALSE
import xml.etree.ElementTree as ET

from kotsu.joint_struct import *
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
  def read_element(link, link_list, link_id, dof_index):
    l = LinkStruct()
    
    l.name = link.attrib.get('name')
    l.joint_type = link.attrib.get('joint_type')
    l.link_type = link.attrib.get('link_type')
    cnct_list = eval(link.attrib.get('connection'))
    l.connection = [int(cnct) for cnct in cnct_list]
    l.connect_pos = np.array(eval(link.attrib.get('connect_pos')))
    l.connect_rot = np.array(eval(link.attrib.get('connect_rot')))
    l.cog = np.array(eval(link.attrib.get('cog')))
    l.mass = eval(link.attrib.get('mass'))
    l.inertia_param = np.array(eval(link.attrib.get('inertia_param')))
    
    l.id = link_id

    l.init()

    l.dof_index = dof_index
    dof_index += l.dof
    
    link_list.append(l)
    
    for child in link:
      RobotStruct.read_element(child, link_list, link_id, dof_index)
      
  @staticmethod
  def read_model_file(xml_data):
    robot = ET.fromstring(xml_data) 
    
    link_list = []
    dof_index = 0
    link_id = 0

    for child in robot:
      RobotStruct.read_element(child, link_list, link_id, dof_index)

    return link_list

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

  def export_coord(self, robot):
    return self.export_vec(robot, "coord", self.coord)
  
  def export_veloc(self, robot):
    return self.export_vec(robot, "veloc", self.veloc)
  
  def export_accel(self, robot):
    return self.export_vec(robot, "accel", self.accel)
  
  def export_force(self, robot):
    return self.export_vec(robot, "force", self.force)
  
  def _vec_to_link_gen_value(self, l, vec):
    if(l.dof > 0):
      return vec[l.dof_index:l.dof_index+l.dof]
    else:
      return []
  
  def import_vecs(self, robot, vecs):
    data = {}
    for l in robot.links:
      for i in range(len(vecs)):
        data.update([(l.name + "_" + self._df.aliases[i] , self._vec_to_link_gen_value(l, vecs[i]))])

    self._df.add_row(data)
    
  def link_gen_value(self, link, name):
    return self.df()[link.name + "_" + name][-1].to_numpy()

  def link_coord(self, link):
    return self.link_gen_value(link, "coord")
  
  def link_veloc(self, link):
    return self.link_gen_value(link, "veloc")
  
  def link_accel(self, link):
    return self.link_gen_value(link, "accel")
  
  def link_force(self, link):
    return self.link_gen_value(link, "force")
  
class RobotState:
  _df : LinkStateDF 
  
  def __init__(self, state_df):
    self._df = state_df
    
  def df(self):
    return self._df.df
    
  def link_state_vec(self, link, name):
    return self.df()[link.name+"_"+name][-1].to_numpy()
    
  def link_state_mat(self, link, name):
    mat_vec = self.df()[link.name+"_"+name][-1].to_numpy()
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
  
  def link_pos(self, link):
    return self.link_state_vec(link, "pos")
  
  def all_link_pos(self, robot):
    return self.all_state_vec(robot, "pos")
  
  def link_rot(self, link):
    return self.link_state_mat(link, "rot")

  def link_vel(self, link):
    return self.link_state_vec(link, "vel")

  def link_acc(self, link):
    return self.link_state_vec(link, "acc")
    
  def link_frame(self, link):
    h = SE3(self.link_rot(link), self.link_pos(link))
    return h.matrix()

  def link_adj_frame(self, link):
    a = SE3(self.link_rot(link), self.link_pos(link))
    return a.adjoint()

  def import_state(self, data):
    self._df.add_row(data)