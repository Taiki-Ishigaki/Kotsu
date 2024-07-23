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

  def joint_trans(theta, b):
    v = b@theta
    return SE3.adj_mat(v)
  
  def link_rel_a(link, link_coord):
    if len(link_coord) != 0:
      a = joint_trans(link_coord, joint_select(link))
    else:
      a = joint_trans(np.zeros(1), joint_select(link))
    return link.connent_adj_frame() @ a

  def link_kinematics(link_a, link_rel_a):
    return link_a @ link_rel_a
  
  def kinematics(link, link_df, link_state_df):
    coord = link_df.df[link.name + "_coord"][0].to_numpy()
    rel_a = link_rel_a(link, coord)
    link_a = link_state_df.link_adj_frame(link)
    return link_a @ rel_a
  
  def link_rel_vel(link, link_veloc):
    if len(link_veloc) != 0:
      v = joint_select(link) @ link_veloc
    else:
      v = np.zeros(6)
    return link.connent_adj_frame() @ v

  def vel_kinematics(link, link_df, link_state_df):
    coord = link_df.df[link.name + "_coord"][0].to_numpy()
    veloc = link_df.df[link.name + "_veloc"][0].to_numpy()
    rel_a = link_rel_a(link, coord)
    rel_eta = link_rel_vel(link, veloc)
    link_eta = link_state_df.df[link.name + "_vel"][0].to_numpy()
    eta = rel_a @ link_eta  + rel_eta
    return eta
  
  def link_rel_acc(link, accel):
    if len(accel) != 0:
      dv = joint_select(link) @ accel
    else:
      dv = np.zeros(6)
    return link.connent_adj_frame() @ dv

  def acc_kinematics(link, link_df, link_state_df):
    coord = link_df.df[link.name + "_coord"][0].to_numpy()
    veloc = link_df.df[link.name + "_veloc"][0].to_numpy()
    accel = link_df.df[link.name + "_accel"][0].to_numpy()
    rel_a = link_rel_a(link, coord)
    rel_eta = link_rel_vel(link, veloc)
    rel_deta = link_rel_acc(link, accel)
    link_eta = link_state_df.df[link.name + "_vel"][0].to_numpy()
    link_deta = link_state_df.df[link.name + "_acc"][0].to_numpy()
    deta = rel_a @ link_deta + SE3.adj_hat( rel_a @ rel_eta ) @ link_eta + rel_deta
    return deta