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
