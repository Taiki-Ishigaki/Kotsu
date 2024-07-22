import matplotlib.pyplot as plt
import numpy as np

from kotsu.robot import *

def show_kotsu(robot, robot_state):
  # 3Dプロットの作成
  fig = plt.figure()
  ax = fig.add_subplot(111, projection='3d')

  # ノードの描画
  pos = robot_state.all_link_pos(robot)
  ax.scatter(pos[:,0], pos[:,1], pos[:,2], c='r', marker='o')

  # エッジの描画
  for i in range(robot_state.robot.link_num):
      for j in robot_state.robot.links[i].connection:
        ax.plot([pos[i,0], pos[j,0]], [pos[i,1], pos[j,1]], [pos[i,2], pos[j,2]], 'b')

  # 軸ラベルの設定
  ax.set_xlabel('X')
  ax.set_ylabel('Y')
  ax.set_zlabel('Z')

  plt.show()