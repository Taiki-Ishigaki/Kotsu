#!/usr/bin/env python3.9
# -*- coding: utf-8 -*-
# 2024.08.14 Created by T.Ishigaki

import bvh as Bvh

from kotsu.robot import *

class KBvh(Bvh.Bvh):
  def get_joint_frame(self, frame_index, joint_name):
    channels = self.joint_channels(joint_name)
    frame = self.frame_joint_channels(frame_index, joint_name, channels)
    return frame
  
class BvhJointStruct(JointStruct):
  channels: np.ndarray = np.array([])
  
  def init(self):
    self.set_dof()
    
  def set_dof(self):
    self.dof = len(self.channels)
    
class BvhLinkStruct(LinkStruct):
  channels: np.ndarray = np.array([])
  offset: np.ndarray = np.array([])
  
class BvhRobotStruct(RobotStruct):
  
  def __init__(self, joints_):
    self.joints = joints_
    
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
  
class BvhMotionDF(RobotDF):
  def __init__(self, robot_, aliases_ = ["coord"], separator_ = "_"):
    self.separator = separator_
    self.aliases = aliases_

    self.df = pl.DataFrame()
    self.set_gen_df(robot_)
    
  def set_gen_df(self, robot_):
    for a in self.aliases:
      for j in robot_.joints:
        alias_name = j.name + self.separator + a
        self.df = self.df.with_columns([pl.Series(name=alias_name, dtype=pl.List(pl.Float64))])
    
class BvhRobotMotion:
  _df : BvhMotionDF 
  frame_num : int = 0
  def __init__(self, motion_df):
    self._df = motion_df
    
  def df(self):
    return self._df.df
    
  def set_motion(self, bvh):
    self.frame_num = len(bvh.frames)
    for i in range(self.frame_num):
      motion_data = {}
      for name in bvh.get_joints_names():
        frame = bvh.get_joint_frame(i, name)
        motion_data.update([(name + "_coord" , frame)])   
      self._df.add_row(motion_data)

class BvhRobot(BvhRobotStruct):
  state : RobotState 
  
  def __init__(self, joints_, bvh_, motions_):
    self.joints = joints_
    self.bvh = bvh_
    self.motions = motions_

  @staticmethod
  def init_from_bvh_file(bvh_file):
    joints, bvh = BvhRobotStruct.read_bvh_file(bvh_file)
    robot = BvhRobotStruct(joints)
    motions = BvhRobotMotion(BvhMotionDF(robot))
    motions.set_motion(bvh)
    return BvhRobot(joints, bvh, motions)
  
    
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