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
      connection="[1]"
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
      connection="[0,2]"
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
      name="link2"
      joint_type="revolution"
      link_type="rigid"
      connection="[1,3]"
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
      name="link3"
      joint_type="revolution"
      link_type="rigid"
      connection="[2]"
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
  
  print("dof" + str(robot.dof))
  
  link_df = LinkGenDF(robot.links)

  data = {
      robot.links[0].name + "_" + "coord" : [],
      robot.links[0].name + "_" + "veloc" : [],
      robot.links[0].name + "_" + "accel" : [],
      robot.links[0].name + "_" + "force" : [],
      robot.links[1].name + "_" + "coord" : [0.],
      robot.links[1].name + "_" + "veloc" : [1.],
      robot.links[1].name + "_" + "accel" : [2.],
      robot.links[1].name + "_" + "force" : [3.],
      robot.links[2].name + "_" + "coord" : [0.],
      robot.links[2].name + "_" + "veloc" : [1.],
      robot.links[2].name + "_" + "accel" : [2.],
      robot.links[2].name + "_" + "force" : [3.],
      robot.links[3].name + "_" + "coord" : [0.],
      robot.links[3].name + "_" + "veloc" : [1.],
      robot.links[3].name + "_" + "accel" : [2.],
      robot.links[3].name + "_" + "force" : [3.]
  }
  link_df.add_row(data)

  print(link_df.df)
  
  robot_gen_value = RobotGenValue(link_df)
  
  print(robot_gen_value.export_coord())
  print(robot_gen_value.export_veloc())
  print(robot_gen_value.export_accel())
  print(robot_gen_value.export_force())

  link_state_df = LinkStateDF(robot.links)
  data = {
      robot.links[0].name + "_" + "pos" : [0., 0., 0.],
      robot.links[0].name + "_" + "rot" : [1., 0., 0., 0., 1., 0., 0., 0., 1.],
      robot.links[0].name + "_" + "vel" : [0., 0., 0., 0., 0., 0.],
      robot.links[0].name + "_" + "acc" : [0., 0., 0., 0., 0., 0.],
      robot.links[1].name + "_" + "pos" : [1., 0., 0.],
      robot.links[1].name + "_" + "rot" : [1., 0., 0., 0., 1., 0., 0., 0., 1.],
      robot.links[1].name + "_" + "vel" : [0., 0., 0., 0., 0., 0.],
      robot.links[1].name + "_" + "acc" : [0., 0., 0., 0., 0., 0.],
      robot.links[2].name + "_" + "pos" : [0., 1., 0.],
      robot.links[2].name + "_" + "rot" : [1., 0., 0., 0., 1., 0., 0., 0., 1.],
      robot.links[2].name + "_" + "vel" : [0., 0., 0., 0., 0., 0.],
      robot.links[2].name + "_" + "acc" : [0., 0., 0., 0., 0., 0.],
      robot.links[3].name + "_" + "pos" : [0., 0., 1.],
      robot.links[3].name + "_" + "rot" : [1., 0., 0., 0., 1., 0., 0., 0., 1.],
      robot.links[3].name + "_" + "vel" : [0., 0., 0., 0., 0., 0.],
      robot.links[3].name + "_" + "acc" : [0., 0., 0., 0., 0., 0.],
  }
  link_state_df.add_row(data)
  
  print(link_state_df.df)
  
  robot_state = RobotState(link_state_df)

  print(robot_state.link_pos(robot, 0))
  print(robot_state.link_pos(robot, 1))
  
  print(robot_state.all_link_pos(robot))

  print(robot_state.link_rot(robot, 0))
  print(robot_state.link_rot(robot, 1))
  
  show_kotsu(robot, robot_state)

if __name__ == "__main__":
    main()