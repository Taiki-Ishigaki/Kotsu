import numpy as np
from kotsu.bvh.struct import *

def main():
  with open('./test.bvh', 'r') as f:
    motion_data = KBvh(f.read())
  
  print(motion_data.get_joints_names())

  print(motion_data.get_joint_frame(0, 'root'))
  print(motion_data.get_joint_frame(0, 'joint1'))

if __name__ == "__main__":
    main()