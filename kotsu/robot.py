#!/usr/bin/env python3.9
# -*- coding: utf-8 -*-
# 2024.07.21 Created by T.Ishigaki

import numpy as np

from kotsu.link_struct import *
from kotsu.link_df import *
from kotsu.robot_struct import *
  
class Robot(RobotStruct):

  gen_value : RobotGenValue
  state : RobotState 

  def __init__(self, links_, gen_value_, state_):
    self.links = links_
    self.gen_value = gen_value_
    self.state = state_
    self.robot_init()

  @staticmethod
  def init_from_model_file(xml_file):
    links = RobotStruct.read_model_file(xml_file)
    gen_value = RobotGenValue(LinkGenDF(links))
    state = RobotState(LinkStateDF(links))
    return Robot(links, gen_value, state)
  
  def import_gen_vecs(self, vecs):
    self.gen_value.import_vecs(self, vecs)

