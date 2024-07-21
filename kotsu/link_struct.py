#!/usr/bin/env python3.9
# -*- coding: utf-8 -*-
# 2024.07.20 Created by T.Ishigaki

import numpy as np

from dataclasses import dataclass

from pickle import FALSE
import xml.etree.ElementTree as ET

from mathrobo.basic import *
from mathrobo.so3 import *
from mathrobo.se3 import *

@dataclass
class LinkStruct_:
  name: str = 'name'
  joint_type: str = "revolution"
  link_type: str = "rigid"
  connection: np.ndarray = np.array([])
  connect_pos: np.ndarray = np.zeros(3)
  connect_rot: np.ndarray = np.identity(3)
  cog: np.ndarray = np.zeros(3)
  mass: float = 1.
  inertia_param: np.ndarray = np.zeros(6)
    
class LinkStruct(LinkStruct_):
  id : int = 0
  dof : int = 0
  joint_dof : int = 0
  link_dof : int = 0
  dof_index : int = 0
  
  connent_frame : np.ndarray = np.identity(4)
  connent_adj_frame : np.ndarray = np.identity(6)
  
  @staticmethod
  def _joint_dof(type):
    if type == "revolution":
      return 1
    elif type == "fix":
      return 0
  
  @staticmethod
  def _link_dof(type):
    if type == "rigid":
      return 0
  
  def set_dof(self):
    self.joint_dof = self._joint_dof(self.joint_type)
    self.link_dof = self._link_dof(self.link_type)
    self.dof_index = self.joint_dof + self.link_dof
    
  def dof(self):
    return self.joint_dof(self) + self.link_dof(self)
  
  def set_connent_frame(self):
    self.connent_frame = SE3(self.connect_rot, self.connect_pos).matrix()
    return self.connent_frame

  def set_connent_adj_frame(self):
    self.connent_adj_frame =  SE3(self.connect_rot, self.connect_pos).adjoint()
    return self.connent_adj_frame
    
def read_model_file(xml_data):
  robot = ET.fromstring(xml_data)

  links = []
  dof_index = 0
  
  for i in range(len(robot)):
    links.append(LinkStruct())

    links[i].name = robot[i].attrib.get('name')
    links[i].joint_type = robot[i].attrib.get('joint_type')
    links[i].link_type = robot[i].attrib.get('link_type')
    links[i].connection = robot[i].attrib.get('connection')
    links[i].connect_pos = np.array(eval(robot[i].attrib.get('connect_pos')))
    links[i].connect_rot = np.array(eval(robot[i].attrib.get('connect_rot')))
    links[i].cog = np.array(eval(robot[i].attrib.get('cog')))
    links[i].mass = eval(robot[i].attrib.get('mass'))
    links[i].inertia_param = np.array(eval(robot[i].attrib.get('inertia_param')))

    links[i].set_dof()
    links[i].set_connent_frame()
    links[i].set_connent_adj_frame()

    links[i].dof_index = dof_index
    dof_index += links[i].dof

  return links
