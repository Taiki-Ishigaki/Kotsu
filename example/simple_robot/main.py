import numpy as np
import polars

from kotsu.robot import *
from kotsu.simple_draw import *

links_xml = """<?xml version="1.0"?>
<robot>
    <link
      name="root"
      joint_type="fix"
      link_type="rigid"
      connection="1"
      connect_pos = "[0., 0., 0.]"
      connect_rot =
      "[
        [1., 0., 0.],
        [0., 1., 0.],
        [0., 0., 1.]
      ]"
      cog = "[0., 1., 0.]"
      mass = "100."
      inertia_param = "[1., 1., 1., 0., 0., 0.]"
    >
    </link>
    <link
      name="link1"
      joint_type="revolution"
      link_type="rigid"
      connection="1"
      connect_pos = "[0., 0., 0.]"
      connect_rot =
      "[
        [1., 0., 0.],
        [0., 1., 0.],
        [0., 0., 1.]
      ]"
      cog = "[0., 1., 0.]"
      mass = "100."
      inertia_param = "[1., 1., 1., 0., 0., 0.]"
    >
    </link>
</robot>
"""

def main():
  robot = RobotStruct.read_model_file(links_xml)
  
  link_df = LinkGenDF(robot.links)

  data = {
      robot.links[0].name + "_" + "coord" : [],
      robot.links[0].name + "_" + "veloc" : [],
      robot.links[0].name + "_" + "accel" : [],
      robot.links[0].name + "_" + "force" : [],
      robot.links[1].name + "_" + "coord" : [0.],
      robot.links[1].name + "_" + "veloc" : [1.],
      robot.links[1].name + "_" + "accel" : [2.],
      robot.links[1].name + "_" + "force" : [3.]
  }
  link_df.add_row(data)

  print(link_df.df)
  
  robot_gen_value = RobotGenValue(robot, link_df)
  
  print(robot_gen_value.set_gen_coord())
  print(robot_gen_value.set_gen_veloc())
  print(robot_gen_value.set_gen_accel())
  print(robot_gen_value.set_gen_force())

  link_state_df = LinkStateDF(robot.links)
  data = {
      robot.links[0].name + "_" + "pos" : [0., 0., 0.],
      robot.links[0].name + "_" + "rot" : [1., 0., 0., 0., 1., 0., 0., 0., 1.],
      robot.links[0].name + "_" + "vel" : [0., 0., 0., 0., 0., 0.],
      robot.links[0].name + "_" + "acc" : [0., 0., 0., 0., 0., 0.],
      robot.links[1].name + "_" + "pos" : [1., 2., 3.],
      robot.links[1].name + "_" + "rot" : [1., 0., 0., 0., 1., 0., 0., 0., 1.],
      robot.links[1].name + "_" + "vel" : [0., 0., 0., 0., 0., 0.],
      robot.links[1].name + "_" + "acc" : [0., 0., 0., 0., 0., 0.],
  }
  link_state_df.add_row(data)
  
  print(link_state_df.df)
  
  robot_state = RobotState(robot, link_df, link_state_df)

  print(robot_state.link_pos(0))
  print(robot_state.link_pos(1))
  
  print(robot_state.all_link_pos())

  print(robot_state.link_rot(0))
  print(robot_state.link_rot(1))
  
  show_kotsu(robot_state)

if __name__ == "__main__":
    main()