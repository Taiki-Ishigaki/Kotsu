import numpy as np
import polars

from kotsu.robot import *

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
      robot.links[1].name + "_" + "veloc" : [0.],
      robot.links[1].name + "_" + "accel" : [0.],
      robot.links[1].name + "_" + "force" : [0.]
  }
  link_df.add_row(data)

  print(link_df.df)

  link_state_df = LinkStateDF(robot.links)
  data = {
      robot.links[0].name + "_" + "pos" : [0., 0., 0.],
      robot.links[0].name + "_" + "rot" : [1., 0., 0., 0., 1., 0., 0., 0., 1.],
      robot.links[0].name + "_" + "vel" : [0., 0., 0., 0., 0., 0.],
      robot.links[0].name + "_" + "acc" : [0., 0., 0., 0., 0., 0.],
      robot.links[1].name + "_" + "pos" : [0., 0., 0.],
      robot.links[1].name + "_" + "rot" : [1., 0., 0., 0., 1., 0., 0., 0., 1.],
      robot.links[1].name + "_" + "vel" : [0., 0., 0., 0., 0., 0.],
      robot.links[1].name + "_" + "acc" : [0., 0., 0., 0., 0., 0.],
  }
  link_state_df.add_row(data)
  
  print(link_state_df.df)

if __name__ == "__main__":
    main()