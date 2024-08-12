#!/usr/bin/env python3.9
# -*- coding: utf-8 -*-
# 2024.08.12 Created by T.Ishigaki

import numpy as np

def search_neiborhood(robot, link_id):
  link_list = []
  link = robot.links[link_id]
  for j_id in link.connect_joint:
    joint = robot.joints[j_id]
    link_list.append(joint.connect_link)

  return link_list

'''
閉リンク構造未対応
'''
def search_root_route(robot, link_id, tree_route):
  link = robot.links[link_id]

  if(link.is_edge() and (not link.is_root())):
    return False
  else:
    link_id_list = search_neiborhood(robot, link_id)
    for l_id in link_id_list:
      if (robot.links[l_id].is_root()):
        route.append(l_id)
      else:
        route = search_root_route(robot, l_id, tree_route)
        if (route):
          route.append(link_id)
    return route



class BasicJacobian:

  @staticmethod
  def calc(self, robot, link_id):
    return None
  
class VelJacobian(BasicJacobian):
  
  @staticmethod
  def calc(robot, link_id):
    jacob = np.zeros(6, robot.dof)

    link = robot.links[link_id]

    while(not link.is_root()):

      jacob[:,robot.dof]

    return jacob
  
  @staticmethod
  def calc_all_link(self, robot):

    return None