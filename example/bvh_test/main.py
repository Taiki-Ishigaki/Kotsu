import numpy as np
import kotsu

def main():
  with open('./motion.bvh', 'r') as f:
  # with open('./test.bvh', 'r') as f:
    robot = kotsu.bvh.BvhRobot.init_from_bvh_file(f.read())

  print(robot.state.df.df)

  robot.update_kinematics(robot.motions.motion_vecs[1])
  
  print(robot.state.df.df)
  
  kotsu.show_kotsu(robot, robot.state)

if __name__ == "__main__":
    main()