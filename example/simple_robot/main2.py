import numpy as np
import polars

from kotsu.robot import *
from kotsu.simple_draw import *

links_xml = """<?xml version="1.0"?>
<robot>
    <joint
      name = "root_joint"
      joint_type = "fix"
    >
    </joint>
    <joint
      name = "joint1"
      joint_type = "revolution"
    >
    </joint>
    <joint
      name = "joint2"
      joint_type = "revolution"
    >
    </joint>
    <joint
      name = "joint3"
      joint_type = "revolution"
    >
    </joint>
    <link
      name="root_link"
      link_type="rigid"
      cog = "[0., 1., 0.]"
      mass = "100."
      inertia_param = "[1., 1., 1., 0., 0., 0.]"
    >
      <joint>
        name = "root_joint"
      </joint>
      <joint>
        name = "joint1"
        connect_pos = "[0., 0., 0.]"
        connect_rot =
        "[
          [1., 0., 0.],
          [0., 1., 0.],
          [0., 0., 1.]
        ]"
      </joint>
    </link>
    <link
      name="link1"
      joint_type="revolution"
      link_type="rigid"
      connection="[0,2]"
      connect_pos = "[0., 0., 1.]"
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
      <joint>
        name = "joint1"
      </joint>
      <joint>
        name = "joint2"
        connect_pos = "[0., 0., 0.]"
        connect_rot =
        "[
          [1., 0., 0.],
          [0., 1., 0.],
          [0., 0., 1.]
        ]"
      </joint>
    </link>
    <link
      name="link2"
      joint_type="revolution"
      link_type="rigid"
      connection="[1,3]"
      connect_pos = "[0., 0., 1.]"
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
      <joint>
        name = "joint2"
      </joint>
      <joint>
        name = "joint3"
        connect_pos = "[0., 0., 0.]"
        connect_rot =
        "[
          [1., 0., 0.],
          [0., 1., 0.],
          [0., 0., 1.]
        ]"
      </joint>
    </link>
    <link
      name="link3"
      joint_type="revolution"
      link_type="rigid"
      connection="[2]"
      connect_pos = "[0., 0., 1.]"
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
      <joint>
        name = "joint3"
      </joint>
    </link>   
</robot>
"""

def main():
  robot = Robot.init_from_model_file(links_xml) 
  
  coord = [0., 0., 0.]
  veloc = [2., 2., 2.]
  accel = [3., 3., 3.]
  force = [4., 4., 4.]
  
  vecs = [coord, veloc, accel, force]

  robot.import_gen_vecs(vecs)
  print(robot.gen_value.df())
  
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

  robot.state._df.add_row(data)
  
  print(robot.state.df())
  
  robot.update_kinematics()
  
  print(robot.state.df())

  show_kotsu(robot, robot.state)

if __name__ == "__main__":
    main()