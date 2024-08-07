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
  # @staticmethod
  # def joint_trans(theta, b):
  #   v = b@theta
  #   return SE3.adj_mat(v)
  
  # @staticmethod
  # def link_rel_a(link, link_coord):
  #   if len(link_coord) != 0:
  #     a = LinkKinematics.joint_trans(link_coord, link.joint_select_mat)
  #   else:
  #     a = LinkKinematics.joint_trans(np.zeros(1), link.joint_select_mat)
  #   return link.connent_adj_frame @ a
  
  # @staticmethod
  # def link_kinematics(link_a, link_rel_a):
  #   return link_a @ link_rel_a
  
  # @staticmethod
  # def kinematics(link, gen_value, state):
  #   coord = gen_value.link_coord(link)
  #   link_a = state.link_adj_frame(link)
  #   rel_a = LinkKinematics.link_rel_a(link, coord)
  #   return link_a @ rel_a
  
  @staticmethod
  def joint_local_frame(theta, b):
    if len(theta) != 0:
      v = b@theta
    else:
      v = b@np.zeros(1)
    frame = SE3.adj_mat(v)
    return frame

  @staticmethod
  def link_rel_frame(link, joint, joint_coord):
    joint_frame = LinkKinematics.joint_local_frame(joint_coord, joint.joint_select_mat)
    connect_frame = link.connect_adj_frames[joint.id]
    rel_frame = joint_frame @ connect_frame
    return rel_frame

  @staticmethod
  def kinematics(link, joint, parent, gen_value, state):
    joint_coord = gen_value.joint_coord(joint)
    if parent:
      p_link_frame = state.link_adj_frame(parent)
    else:
      p_link_frame = np.identity(6)
    rel_frame = LinkKinematics.link_rel_frame(link, joint, joint_coord)
    frame = p_link_frame @ rel_frame
    return frame

  # @staticmethod
  # def link_rel_vel(link, link_veloc):
  #   if len(link_veloc) != 0:
  #     v = link.joint_select_mat @ link_veloc
  #   else:
  #     v = np.zeros(6)
  #   return np.linalg.inv(link.connent_adj_frame) @ v

  # @staticmethod
  # def vel_kinematics(link, gen_value, state):
  #   coord = gen_value.link_coord(link)
  #   veloc = gen_value.link_veloc(link)
  #   link_eta = state.link_vel(link)
  #   rel_a = LinkKinematics.link_rel_a(link, coord)
  #   rel_eta = LinkKinematics.link_rel_vel(link, veloc)
  #   eta = rel_a @ link_eta  + rel_eta
  #   return eta
  
  @staticmethod
  def joint_local_vel(joint, joint_vel):
    if len(joint_vel) != 0:
      vel = joint.joint_select_mat @ joint_vel
    else:
      vel = np.zeros(6)
    return vel

  @staticmethod
  def link_rel_vel(link, joint, joint_vel):
    local_vel = joint_local_vel(joint, joint_vel)
    connect_frame = link.connect_adj_frames[joint.id]
    rel_vel = np.linalg.inv(connect_frame) @ local_vel
    return rel_vel

  @staticmethod
  def vel_kinematics(link, joint, parent, gen_value, state):
    joint_coord = gen_value.joint_coord(joint)
    joint_veloc = gen_value.link_veloc(joint)
    if parent:
      link_vel = state.link_vel(parent)
    else:
      link_vel = np.zeros(6)
    rel_frame = LinkKinematics.link_rel_frame(link, joint, joint_coord)
    rel_vel = LinkKinematics.link_rel_vel(link, joint, joint_veloc)
    vel = np.linalg.inv(rel_frame) @ link_vel  + rel_vel
    return vel
  
  # @staticmethod
  # def link_rel_acc(link, accel):
  #   if len(accel) != 0:
  #     dv = link.joint_select_mat @ accel
  #   else:
  #     dv = np.zeros(6)
  #   return np.linalg.inv(link.connent_adj_frame) @ dv

  # @staticmethod
  # def acc_kinematics(link, gen_value, state):
  #   coord = gen_value.link_coord(link)
  #   veloc = gen_value.link_veloc(link)
  #   accel = gen_value.link_accel(link)
  #   link_eta = state.link_vel(link)
  #   link_deta = state.link_acc(link)
  #   rel_a = LinkKinematics.link_rel_a(link, coord)
  #   rel_eta = LinkKinematics.link_rel_vel(link, veloc)
  #   rel_deta = LinkKinematics.link_rel_acc(link, accel)
  #   deta = rel_a @ link_deta + SE3.adj_hat( rel_a @ rel_eta ) @ link_eta + rel_deta
  #   return deta

  @staticmethod
  def joint_local_acc(joint, joint_acc):
    if len(joint_acc) != 0:
      acc = joint.joint_select_mat @ joint_acc
    else:
      acc = np.zeros(6)
    return acc

  @staticmethod
  def link_rel_acc(link, joint, joint_acc):
    local_acc = joint_local_acc(joint, joint_acc)
    connect_frame = link.connect_adj_frames[joint.id]
    rel_acc = np.linalg.inv(connect_frame) @ local_acc
    return rel_acc

  @staticmethod
  def acc_kinematics(link, joint, parent, gen_value, state):
    joint_coord = gen_value.joint_coord(joint)
    joint_veloc = gen_value.joint_veloc(joint)
    joint_accel = gen_value.joint_accel(joint)
    if parent:
      link_vel = state.link_vel(parent)
      link_acc = state.link_acc(parent)
    else:
      link_vel = np.zeros(6)
      link_acc = np.zeros(6)
    rel_frame = LinkKinematics.link_rel_a(link, coord)
    rel_vel = LinkKinematics.link_rel_vel(link, veloc)
    rel_acc = LinkKinematics.link_rel_acc(link, accel)
    acc = rel_frame @ link_acc + SE3.adj_hat( rel_frame @ rel_vel ) @ link_vel + rel_acc
    return acc