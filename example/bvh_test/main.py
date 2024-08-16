import numpy as np
from kotsu.bvh.struct import *

def main():
  # with open('./test.bvh', 'r') as f:
  #   motion_data = KBvh(f.read())
  
  # print(motion_data.get_joints_names())
  
  # print('root')
  # print(motion_data.get_joint_frame(0, 'root'))
  # print(motion_data.joint_channels('root'))
  # print(motion_data.joint_parent('root'))
  # print(motion_data.get_joint_index('root'))
  # print(motion_data.joint_offset('root'))
  # print(motion_data.get_joint_channels_index('root'))
  # print('\njoint1')
  # print(motion_data.get_joint_frame(0, 'joint1'))
  # print(motion_data.joint_channels('joint1'))
  # print(motion_data.joint_parent('joint1'))
  # print(motion_data.get_joint_index('joint1'))
  # print(motion_data.joint_offset('joint1'))
  # print(motion_data.get_joint_channels_index('joint1'))

  # channels = motion_data.joint_channels('joint1')
  # frames = motion_data.frames_joint_channels('joint1', channels)
  # print(frames)
  # print(len(motion_data.frames))
  
  with open('./test.bvh', 'r') as f:
    robot = BvhRobot.init_from_bvh_file(f.read())
    
  print(robot.motions.df())

if __name__ == "__main__":
    main()