"""
based on exts/isaacsim.robot.manipulators.examples/isaacsim/robot/manipulators/examples/franka/tasks/pick_place.py
"""

from typing import Optional

import isaacsim.core.api.tasks as tasks
import numpy as np
from isaacsim.core.utils.prims import is_prim_path_valid
from isaacsim.core.utils.string import find_unique_string_name
from rc5 import Franka


class PickPlace(tasks.PickPlace):
    """[summary]

    Args:
        name (str, optional): [description]. Defaults to "franka_pick_place".
        cube_initial_position (Optional[np.ndarray], optional): [description]. Defaults to None.
        cube_initial_orientation (Optional[np.ndarray], optional): [description]. Defaults to None.
        target_position (Optional[np.ndarray], optional): [description]. Defaults to None.
        cube_size (Optional[np.ndarray], optional): [description]. Defaults to None.
        offset (Optional[np.ndarray], optional): [description]. Defaults to None.
    """

    def __init__(
        self,
        name: str = "rc5_pick_place",
        cube_initial_position: Optional[np.ndarray] = None,
        cube_initial_orientation: Optional[np.ndarray] = None,
        target_position: Optional[np.ndarray] = None,
        cube_size: Optional[np.ndarray] = None,
        offset: Optional[np.ndarray] = None,
    ) -> None:
        tasks.PickPlace.__init__(
            self,
            name=name,
            cube_initial_position=cube_initial_position,
            cube_initial_orientation=cube_initial_orientation,
            target_position=target_position,
            cube_size=cube_size,
            offset=offset,
        )
        return

    def set_robot(self) -> Franka:
        """[summary]

        Returns:
            Franka: [description]
        """
        franka_prim_path = find_unique_string_name(
            initial_name="/World/Franka", is_unique_fn=lambda x: not is_prim_path_valid(x)
        )
        franka_robot_name = find_unique_string_name(
            initial_name="my_franka", is_unique_fn=lambda x: not self.scene.object_exists(x)
        )
        # franka_robot_name = "RC5_roboarm"
        
        robot = Franka(
            prim_path=franka_prim_path,
            name=franka_robot_name,
            usd_path="./agibot/agibot.usd",
            
            # PickPlace requires rigid body to get its position as part of obs
            # end_effector_prim_name="rc5_model/tn__BaseRC_5_i8",
            # end_effector_prim_name="raise_a2_w_t3/left_arm_link07",
            end_effector_prim_name="left_arm_link07",
            
            position=[0., -0.3, 0.],
        )
        
        # set aux USD
        from isaacsim.core.utils.stage import (add_reference_to_stage,
                                               get_stage_units)
        add_reference_to_stage(usd_path="./table_scene.usd", prim_path="/World/table_scene")
        
        return robot
