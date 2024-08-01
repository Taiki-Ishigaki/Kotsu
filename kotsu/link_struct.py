#!/usr/bin/env python3.9
# -*- coding: utf-8 -*-
# 2024.07.20 Created by T.Ishigaki

import numpy as np

from dataclasses import dataclass

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

  joint_select_mat : np.ndarray = np.array([])
  
  connent_frame : np.ndarray = np.identity(4)
  connent_adj_frame : np.ndarray = np.identity(6)

  def init(self):
    self.joint_select_mat = self._joint_select_mat(self.joint_type)
    self.set_dof()
    # self.set_connent_frame()
    # self.set_connent_adj_frame()

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
    
  @staticmethod
  def _joint_select_mat(joint_type):
    if joint_type == 'fix':
      return np.zeros((6,1))
    elif joint_type == 'revolution':
      mat = np.zeros((6,1))
      mat[2,0] = 1
      return mat
  
  def set_dof(self):
    self.joint_dof = self._joint_dof(self.joint_type)
    self.link_dof = self._link_dof(self.link_type)
    self.dof = self.joint_dof + self.link_dof
  
  def set_connent_frame(self):
    self.connent_frame = SE3(self.connect_rot, self.connect_pos).matrix()
    return self.connent_frame

  def set_connent_adj_frame(self):
    self.connent_adj_frame =  SE3(self.connect_rot, self.connect_pos).adjoint()
    return self.connent_adj_frame
