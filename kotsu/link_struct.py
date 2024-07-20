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
class LinkStruct:
  name: str = 'name'
  joint_type: str = "revolution"
  link_type: str = "rigid"
  connection: np.ndarray = np.array([])
  connect_pos: np.ndarray = np.zeros(3)
  connect_rot: np.ndarray = np.identity(3)
  cog: np.ndarray = np.zeros(3)
  mass: float = 1.
  inertia_param: np.ndarray = np.zeros(6)

  def read_link(self):
    return

  def connent_frame(self):
    return SE3(self.connect_rot, self.connect_pos).matrix()

  def connent_adj_frame(self):
    return SE3(self.connect_rot, self.connect_pos).adjoint()
  
def read_model_file(xml_data):
  robot = ET.fromstring(xml_data)

  links = []

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

  return links