import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation

from .struct import *
from .robot import *
from .kinematics import *

def show_bvh(robot):
  
  fig = plt.figure()
  ax = fig.add_subplot(111, projection='3d')
  
  ims = []
  for time in range(robot.motions.frame_num):
    robot.update_kinematics(robot.motions.motion_vecs[time]) 
    pos = robot.state.all_joint_pos(robot)
    
    im_list = []
    im = ax.scatter(pos[:,0], pos[:,1], pos[:,2], c='r', marker='o')
    im_list.append(im)
    
    for j in robot.joints:
      for c_j in j.children:
        im = ax.plot([pos[j.id,0], pos[c_j,0]], [pos[j.id,1], pos[c_j,1]], [pos[j.id,2], pos[c_j,2]], 'b')
        im_list.extend(im)
    
    ims.append(im_list)

  ani = animation.ArtistAnimation(fig, ims, interval = 20)
  ani.save('anim.gif', writer="pillow")
  ani.save('anim.mp4', writer="ffmpeg")
  plt.show()