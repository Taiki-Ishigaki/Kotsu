#!/usr/bin/env python3.9
# -*- coding: utf-8 -*-
# 2024.07.20 Created by T.Ishigaki

import numpy as np

from dataclasses import dataclass, field

from mathrobo.basic import *
from mathrobo.so3 import *
from mathrobo.se3 import *

@dataclass
class LinkStruct_:
  name: str = 'name'
  joint_type: str = "revolution"
  link_type: str = "rigid"
  connect_joint: np.ndarray = field(default_factory=lambda: np.array([]))
  connect_pos: np.ndarray = field(default_factory=lambda: np.zeros(3))
  connect_rot: np.ndarray = field(default_factory=lambda: np.identity(3))
  cog: np.ndarray = field(default_factory=lambda: np.zeros(3))
  mass: float = 1.
  inertia_param: np.ndarray = field(default_factory=lambda: np.zeros(6))
    
class LinkStruct(LinkStruct_):
  id : int = 0
  dof : int = 0
  joint_dof : int = 0
  link_dof : int = 0
  dof_index : int = 0

  joint_select_mat : np.ndarray = np.array([])

  def init(self):
    self.joint_select_mat = self._joint_select_mat(self.joint_type)
    self.set_dof()
    self.set_connect_frames()

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
  
  def set_connect_frames(self):
    self.connect_frames = []
    self.connect_adj_frames = []
    for i in range(len(self.connect_joint)):
      h = SE3(self.connect_rot[i], self.connect_pos[i])
      self.connect_frames.append(h.matrix())
      self.connect_adj_frames.append(h.adjoint())
