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