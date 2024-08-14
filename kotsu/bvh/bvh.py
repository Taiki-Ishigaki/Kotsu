# import library
from bvh import Bvh
import numpy as np

# BVHファイルを読み込む
with open('MCPM_20240808_100853.BVH') as f:
    motion = Bvh(f.read())

number_frame = motion.nframes

link_name = motion.get_joints_names()
axis_name = ('Xposition','Yposition','Zposition','Xrotation','Yrotation','Zrotation')

pos = np.zeros((len(link_name),int(number_frame),int(len(axis_name)/2)))
ori = np.zeros((len(link_name),int(number_frame),int(len(axis_name)/2)))


init_pos = np.zeros((len(link_name),int(len(axis_name)/2)))

for i in range(0,int(number_frame)):
    for j in range(0,len(link_name)):
        for k in range(0,len(axis_name)):
            if k < 3:
                pos[j,i,k] = motion.frame_joint_channel(i, link_name[j], axis_name[k])
                
                if i == 0:
                    init_pos[j,k] = motion.frame_joint_channel(i, link_name[j], axis_name[k])
            else:
                ori[j,i,k-3] = motion.frame_joint_channel(i, link_name[j], axis_name[k])