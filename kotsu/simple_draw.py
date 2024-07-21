import matplotlib.pyplot as plt
import numpy as np

from kotsu.robot import *

def show_kotsu(robot_state):
  # 3Dプロットの作成
  fig = plt.figure()
  ax = fig.add_subplot(111, projection='3d')

  # ノードの描画
  pos = robot_state.all_link_pos()
  ax.scatter(pos[:,0], pos[:,1], pos[:,2], c='r', marker='o')

  # エッジの描画
  # for edge in edges:
  #     p1 = nodes[edge[0]]
  #     p2 = nodes[edge[1]]
  #     ax.plot([p1[0], p2[0]], [p1[1], p2[1]], [p1[2], p2[2]], 'b')

  # 軸ラベルの設定
  ax.set_xlabel('X')
  ax.set_ylabel('Y')
  ax.set_zlabel('Z')

  plt.show()