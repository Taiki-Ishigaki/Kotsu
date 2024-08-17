import numpy as np
import kotsu

def main():
  with open('./test.bvh', 'r') as f:
    robot = kotsu.bvh.BvhRobot.init_from_bvh_file(f.read())
    
  print(robot.motions.df.df)

if __name__ == "__main__":
    main()