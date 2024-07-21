#!/usr/bin/env python3.9
# -*- coding: utf-8 -*-
# 2024.07.20 Created by T.Ishigaki

import numpy as np
import polars as pl

from mathrobo.basic import *
from mathrobo.so3 import *
from mathrobo.se3 import *

class LinkDF:
  def __init__(self, links, aliases_, separator_ = "_"):
    self.aliases = aliases_
    self.separator = separator_

    self.df = pl.DataFrame()
    self.set_link_df(links)

  def set_link_df(self, links):
    for l in links:
      for a in self.aliases:
        alias_name = l.name + self.separator + a
        self.df = self.df.with_columns([pl.Series(name=alias_name, dtype=pl.List(pl.Float64))])

  def add_row(self, data):
    new_row = pl.DataFrame([data])
    self.df = self.df.vstack(new_row)

class LinkGenDF(LinkDF):
  def __init__(self, links, separator_ = "_"):
    self.links = links
    self.separator = separator_
    self.aliases = ["coord", "veloc", "accel", "force"]

    self.df = pl.DataFrame()
    self.set_link_df(links)

class LinkStateDF(LinkDF):
  def __init__(self, links, separator_ = "_"):
    self.separator = separator_
    self.aliases = ["pos", "rot", "vel", "acc"]

    self.df = pl.DataFrame()
    self.set_link_df(links)

  def link_frame(self, id):
    _pos = self.df[self.links[id].name+"_pos"].to_numpy()
    _rot = np.zeros((3,3))
    r = self.df[self.links[id].name+"_rot"].to_numpy()
    _rot[0,0:3] = r[0:3]
    _rot[1,0:3] = r[3:6]
    _rot[2,0:3] = r[6:12]
    return SE3(_rot, _pos).matrix()

  def link_adj_frame(self, id):
    _pos = self.df[self.links[id].name+"_pos"][0].to_numpy()
    _rot = np.zeros((3,3))
    r = self.df[self.links[id].name+"_rot"][0].to_numpy()
    _rot[0,0:3] = r[0:3]
    _rot[1,0:3] = r[3:6]
    _rot[2,0:3] = r[6:12]
    a = SE3(_rot, _pos)
    return a.adjoint()
