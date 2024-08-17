#!/usr/bin/env python3.9
# -*- coding: utf-8 -*-
# 2024.08.14 Created by T.Ishigaki

import bvh as Bvh

from ..robot import *

class KBvh(Bvh.Bvh):
  def get_joint_frame(self, frame_index, joint_name):
    channels = self.joint_channels(joint_name)
    frame = self.frame_joint_channels(frame_index, joint_name, channels)
    return frame
  
  def joint_direct_children(self, name):
      joint = self.get_joint(name)
      return [child for child in joint.filter('JOINT')]
  
  def get_joint_child(self, name):
    joint = self.get_joint(name)
    child_list = []
    for child in joint.children:
      for index, item in enumerate(child.value):
          if item == 'JOINT':
              if index + 1 < len(child.value):
                  child_list.append(child.value[index + 1:][0])
    return child_list  

class BvhJointStruct(JointStruct):
  channels: np.ndarray = np.array([])
  offset: np.ndarray = np.array([])
  
  def init(self):
    self.set_dof()
    
  def set_dof(self):
    self.dof = len(self.channels)
  
class BvhRobotStruct(RobotStruct):
  
  def __init__(self, joints_):
    self.joints = joints_
    
    self.robot_init()
    
  def robot_init(self):
    self.joint_num = len(self.joints)  

    self.dof = 0
    self.joint_dof = 0
    
    for j in self.joints:
      self.joint_dof += j.dof
      if j.is_root:
        self.root_joint_id = j.id
      
    self.dof = self.joint_dof
    
    self.set_joint_names()
    
  @staticmethod
  def read_bvh_file(data):
    bvh = KBvh(data)

    dof_index = 0
    joints = []
    for j_name in bvh.get_joints_names():
      joint = BvhJointStruct()
      joint.name = j_name
      joint.channels = bvh.joint_channels(j_name)
      joint.id = bvh.get_joint_index(j_name)
      joint.dof = len(joint.channels)
      joint.dof_index = dof_index
      joint.offset = bvh.joint_offset(j_name)
      
      if joint.id == 0:
        joint.is_root = True
      else:
        joint.is_root = False
      
      joints.append(joint)

      dof_index += joint.dof
    
    return joints, bvh
  
  def set_joint_names(self):
    self.joint_names = []
    for j in self.joints:
      self.joint_names.append(j.name)
  
class BvhRobotMotion:
  motion_df : RobotDF
  motion_vecs : np.ndarray = np.array([])
  
  frame_num : int = 0
  def __init__(self, bvh):
    state_names = bvh.get_joints_names()
    self.motion_df = RobotDF(state_names, ["coord"], "_")
    
    self.set_motion(bvh)
    self.set_motion_vec(bvh)
    
  def set_motion(self, bvh):
    self.frame_num = len(bvh.frames)
    for i in range(self.frame_num):
      motion_data = {}
      for name in bvh.get_joints_names():
        frame = bvh.get_joint_frame(i, name)
        motion_data.update([(name + "_coord" , frame)])   
      self.motion_df.add_row(motion_data)

  def set_motion_vec(self, bvh):
    shape  = (len(bvh.frames), len(bvh.frames[0]))
    self.motion_vecs = np.zeros(shape)
    
    for frame_index in range(len(bvh.frames)):
      for data_index in range(len(bvh.frames[0])):
        self.motion_vecs[frame_index][data_index] = float(bvh.frames[frame_index][data_index])
    
class BvhRobotState:
  df : RobotDF
  
  def __init__(self, robot):
    aliases = ["pos", "rot"]
    separator = "_"
    state_names = robot.joint_names
    self.df = RobotDF(state_names, aliases, separator)
    
  def import_state(self, data):
    self.df.add_row(data)
    
  def all_state_vec(self, robot, name):
    labels = []
    for j in robot.joints:
      labels.append(j.name+"_"+name) 
    mat = [self.df.df[label][-1].to_list() for label in labels]
    return np.array(mat)
  
  def all_link_pos(self, robot):
    return self.all_state_vec(robot, "pos")
      
# import re

# class BvhNode:

#     def __init__(self, value=[], parent=None):
#         self.value = value
#         self.children = []
#         self.parent = parent
#         if self.parent:
#             self.parent.add_child(self)

#     def add_child(self, item):
#         item.parent = self
#         self.children.append(item)

#     def filter(self, key):
#         for child in self.children:
#             if child.value[0] == key:
#                 yield child

#     def __iter__(self):
#         for child in self.children:
#             yield child

#     def __getitem__(self, key):
#         for child in self.children:
#             for index, item in enumerate(child.value):
#                 if item == key:
#                     if index + 1 >= len(child.value):
#                         return None
#                     else:
#                         return child.value[index + 1:]
#         raise IndexError('key {} not found'.format(key))

#     def __repr__(self):
#         return str(' '.join(self.value))

#     @property
#     def name(self):
#         return self.value[1]


# class Bvh:

#     def __init__(self, data):
#         self.data = data
#         self.root = BvhNode()
#         self.frames = []
#         self.tokenize()

#     def tokenize(self):
#         first_round = []
#         accumulator = ''
#         for char in self.data:
#             if char not in ('\n', '\r'):
#                 accumulator += char
#             elif accumulator:
#                     first_round.append(re.split('\\s+', accumulator.strip()))
#                     accumulator = ''
#         node_stack = [self.root]
#         frame_time_found = False
#         node = None
#         for item in first_round:
#             if frame_time_found:
#                 self.frames.append(item)
#                 continue
#             key = item[0]
#             if key == '{':
#                 node_stack.append(node)
#             elif key == '}':
#                 node_stack.pop()
#             else:
#                 node = BvhNode(item)
#                 node_stack[-1].add_child(node)
#             if item[0] == 'Frame' and item[1] == 'Time:':
#                 frame_time_found = True

#     def search(self, *items):
#         found_nodes = []

#         def check_children(node):
#             if len(node.value) >= len(items):
#                 failed = False
#                 for index, item in enumerate(items):
#                     if node.value[index] != item:
#                         failed = True
#                         break
#                 if not failed:
#                     found_nodes.append(node)
#             for child in node:
#                 check_children(child)
#         check_children(self.root)
#         return found_nodes

#     def get_joints(self):
#         joints = []

#         def iterate_joints(joint):
#             joints.append(joint)
#             for child in joint.filter('JOINT'):
#                 iterate_joints(child)
#         iterate_joints(next(self.root.filter('ROOT')))
#         return joints

#     def get_joints_names(self):
#         joints = []

#         def iterate_joints(joint):
#             joints.append(joint.value[1])
#             for child in joint.filter('JOINT'):
#                 iterate_joints(child)
#         iterate_joints(next(self.root.filter('ROOT')))
#         return joints

#     def joint_direct_children(self, name):
#         joint = self.get_joint(name)
#         return [child for child in joint.filter('JOINT')]

#     def get_joint_index(self, name):
#         return self.get_joints().index(self.get_joint(name))

#     def get_joint(self, name):
#         found = self.search('ROOT', name)
#         if not found:
#             found = self.search('JOINT', name)
#         if found:
#             return found[0]
#         raise LookupError('joint not found')

#     def joint_offset(self, name):
#         joint = self.get_joint(name)
#         offset = joint['OFFSET']
#         return (float(offset[0]), float(offset[1]), float(offset[2]))

#     def joint_channels(self, name):
#         joint = self.get_joint(name)
#         return joint['CHANNELS'][1:]

#     def get_joint_channels_index(self, joint_name):
#         index = 0
#         for joint in self.get_joints():
#             if joint.value[1] == joint_name:
#                 return index
#             index += int(joint['CHANNELS'][0])
#         raise LookupError('joint not found')

#     def get_joint_channel_index(self, joint, channel):
#         channels = self.joint_channels(joint)
#         if channel in channels:
#             channel_index = channels.index(channel)
#         else:
#             channel_index = -1
#         return channel_index
        
#     def frame_joint_channel(self, frame_index, joint, channel, value=None):
#         joint_index = self.get_joint_channels_index(joint)
#         channel_index = self.get_joint_channel_index(joint, channel)
#         if channel_index == -1 and value is not None:
#             return value
#         return float(self.frames[frame_index][joint_index + channel_index])

#     def frame_joint_channels(self, frame_index, joint, channels, value=None):
#         values = []
#         joint_index = self.get_joint_channels_index(joint)
#         for channel in channels:
#             channel_index = self.get_joint_channel_index(joint, channel)
#             if channel_index == -1 and value is not None:
#                 values.append(value)
#             else:
#                 values.append(
#                     float(
#                         self.frames[frame_index][joint_index + channel_index]
#                     )
#                 )
#         return values

#     def frames_joint_channels(self, joint, channels, value=None):
#         all_frames = []
#         joint_index = self.get_joint_channels_index(joint)
#         for frame in self.frames:
#             values = []
#             for channel in channels:
#                 channel_index = self.get_joint_channel_index(joint, channel)
#                 if channel_index == -1 and value is not None:
#                     values.append(value)
#                 else:
#                     values.append(
#                         float(frame[joint_index + channel_index]))
#             all_frames.append(values)
#         return all_frames

#     def joint_parent(self, name):
#         joint = self.get_joint(name)
#         if joint.parent == self.root:
#             return None
#         return joint.parent

#     def joint_parent_index(self, name):
#         joint = self.get_joint(name)
#         if joint.parent == self.root:
#             return -1
#         return self.get_joints().index(joint.parent)

#     @property
#     def nframes(self):
#         try:
#             return int(next(self.root.filter('Frames:')).value[1])
#         except StopIteration:
#             raise LookupError('number of frames not found')

#     @property
#     def frame_time(self):
#         try:
#             return float(next(self.root.filter('Frame')).value[2])
#         except StopIteration:
#             raise LookupError('frame time not found')