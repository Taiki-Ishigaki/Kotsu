import numpy as np
from kotsu.bvh.struct import *

def main():
  with open('./test.bvh', 'r') as f:
    robot = BvhRobot.init_from_bvh_file(f.read())
    
  print(robot.motions.df.df)

if __name__ == "__main__":
    main()