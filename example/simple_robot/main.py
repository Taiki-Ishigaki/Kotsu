import numpy as np

from kotsu.robot import *
from kotsu.gui.simple_draw import *

def main():
  robot = Robot.init_from_model_file("simple_robot.ktm") 
  
  coord = [1., 1., 1.]
  veloc = [0., 0., 0.]
  accel = [0., 0., 0.]
  force = [0., 0., 0.]
  
  vecs = [coord, veloc, accel, force]
  vec = []
  vec.extend(coord)
  vec.extend(veloc)
  vec.extend(accel)
  vec.extend(force)
  
  robot.import_gen_vecs(vecs)
  print(robot.gen_value.df.df)
  
  robot.import_motions(vec)
  print(robot.motions.motions)
  
  robot.update_kinematics()
  
  print(robot.state.df.df)

  show_kotsu(robot, robot.state)
  
if __name__ == "__main__":
    main()