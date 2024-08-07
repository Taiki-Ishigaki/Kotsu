#!/usr/bin/env python3.9
# -*- coding: utf-8 -*-
# 2024.07.20 Created by T.Ishigaki

import numpy as np
import polars as pl

from mathrobo.basic import *
from mathrobo.so3 import *
from mathrobo.se3 import *

class RobotDF:
  def __init__(self, robot_, aliases_, separator_ = "_"):
    self.aliases = aliases_
    self.separator = separator_

    self.df = pl.DataFrame()

  def add_row(self, data):
    new_row = pl.DataFrame([data])
    self.df = self.df.vstack(new_row)

class RobotGenDF(RobotDF):
  def __init__(self, robot_, aliases_ = ["coord", "veloc", "accel", "force"], separator_ = "_"):
    self.separator = separator_
    self.aliases = aliases_

    self.df = pl.DataFrame()
    self.set_gen_df(robot_)
    
  def set_gen_df(self, robot_):
    for a in self.aliases:
      for j in robot_.joints:
        alias_name = j.name + self.separator + a
        self.df = self.df.with_columns([pl.Series(name=alias_name, dtype=pl.List(pl.Float64))])
      for l in robot_.links:
        alias_name = l.name + self.separator + a
        self.df = self.df.with_columns([pl.Series(name=alias_name, dtype=pl.List(pl.Float64))])

class RobotStateDF(RobotDF):
  def __init__(self, robot_, aliases_ = ["pos", "rot", "vel", "acc"], separator_ = "_"):
    self.separator = separator_
    self.aliases = aliases_

    self.df = pl.DataFrame()
    self.set_link_state_df(robot_)
    
  def set_link_state_df(self, robot_):
    for l in robot_.links:
        for a in self.aliases:
          alias_name = l.name + self.separator + a
          self.df = self.df.with_columns([pl.Series(name=alias_name, dtype=pl.List(pl.Float64))])
