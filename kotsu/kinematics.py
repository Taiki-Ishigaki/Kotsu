#!/usr/bin/env python3.9
# -*- coding: utf-8 -*-
# 2024.07.23 Created by T.Ishigaki

import numpy as np

from mathrobo.basic import *
from mathrobo.so3 import *
from mathrobo.se3 import *

from kotsu.link_struct import *
from kotsu.link_df import *

class LinkKinematics:
  @staticmethod
  def joint_trans(theta, b):
    v = b@theta
    return SE3.adj_mat(v)
  
  @staticmethod
  def link_rel_a(link, link_coord):
    if len(link_coord) != 0:
      a = LinkKinematics.joint_trans(link_coord, link.joint_select_mat)
    else:
      a = LinkKinematics.joint_trans(np.zeros(1), link.joint_select_mat)
    return link.connent_adj_frame @ a
  
  @staticmethod
  def link_kinematics(link_a, link_rel_a):
    return link_a @ link_rel_a
  
  @staticmethod
  def kinematics(link, gen_value, state):
    coord = gen_value.link_coord(link)
    link_a = state.link_adj_frame(link)
    rel_a = LinkKinematics.link_rel_a(link, coord)
    return link_a @ rel_a
  
  @staticmethod
  def link_rel_vel(link, link_veloc):
    if len(link_veloc) != 0:
      v = link.joint_select_mat @ link_veloc
    else:
      v = np.zeros(6)
    return np.linalg.inv(link.connent_adj_frame) @ v

  @staticmethod
  def vel_kinematics(link, gen_value, state):
    coord = gen_value.link_coord(link)
    veloc = gen_value.link_veloc(link)
    link_eta = state.link_vel(link)
    rel_a = LinkKinematics.link_rel_a(link, coord)
    rel_eta = LinkKinematics.link_rel_vel(link, veloc)
    eta = rel_a @ link_eta  + rel_eta
    return eta
  
  @staticmethod
  def link_rel_acc(link, accel):
    if len(accel) != 0:
      dv = link.joint_select_mat @ accel
    else:
      dv = np.zeros(6)
    return np.linalg.inv(link.connent_adj_frame) @ dv

  @staticmethod
  def acc_kinematics(link, gen_value, state):
    coord = gen_value.link_coord(link)
    veloc = gen_value.link_veloc(link)
    accel = gen_value.link_accel(link)
    link_eta = state.link_vel(link)
    link_deta = state.link_acc(link)
    rel_a = LinkKinematics.link_rel_a(link, coord)
    rel_eta = LinkKinematics.link_rel_vel(link, veloc)
    rel_deta = LinkKinematics.link_rel_acc(link, accel)
    deta = rel_a @ link_deta + SE3.adj_hat( rel_a @ rel_eta ) @ link_eta + rel_deta
    return deta