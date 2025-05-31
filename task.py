from abc import ABC, abstractmethod
from pathlib import Path
from typing import Optional

import numpy as np
from isaacsim.core.api.objects import DynamicCuboid
from isaacsim.core.api.scenes.scene import Scene
from isaacsim.core.api.tasks import BaseTask
from isaacsim.core.utils.prims import is_prim_path_valid
from isaacsim.core.utils.stage import add_reference_to_stage, get_stage_units
from isaacsim.core.utils.string import find_unique_string_name
from robot import Agibot


class AgibotTask(BaseTask):
    """
    Args:
        name (str): [description]
        offset (Optional[np.ndarray], optional): [description]. Defaults to None.
    """

    def __init__(
        self,
        name: str = 'agibot_task',
        offset: Optional[np.ndarray] = None,
    ) -> None:
        BaseTask.__init__(self, name=name, offset=offset)
        self._robot = None

    def set_up_scene(self, scene: Scene) -> None:
        """[summary]

        Args:
            scene (Scene): [description]
        """
        super().set_up_scene(scene)
        scene.add_default_ground_plane()
        self._robot = self.set_robot()
        scene.add(self._robot)
        self._task_objects[self._robot.name] = self._robot
        self._move_task_objects_to_their_frame()
        return

    def set_robot(self) -> Agibot:
        robot_prim_path = find_unique_string_name(
            initial_name="/World/Agibot", is_unique_fn=lambda x: not is_prim_path_valid(x)
        )
        robot_name = find_unique_string_name(
            initial_name="agibot", is_unique_fn=lambda x: not self.scene.object_exists(x)
        )
        
        assets_path = Path(__file__).parent / "assets"
        agibot_asset_path = (assets_path / "agibot.usd").as_posix()
        table_scene_asset_path = (assets_path / "table_scene.usd").as_posix()
        
        robot = Agibot(
            prim_path=robot_prim_path,
            name=robot_name,
            usd_path=agibot_asset_path,
            position=[0., -0.3, 0.],
        )
        
        # set aux USD
        add_reference_to_stage(usd_path=table_scene_asset_path, prim_path="/World/table_scene")
        return robot

    def get_params(self) -> dict:
        params_representation = dict()
        params_representation["robot_name"] = {"value": self._robot.name, "modifiable": False}
        return params_representation

    def get_observations(self) -> dict:
        """[summary]

        Returns:
            dict: [description]
        """
        joints_state = self._robot.get_joints_state()
        return {
            self._robot.name: {
                "joint_positions": joints_state.positions,
            },
        }

    def pre_step(self, time_step_index: int, simulation_time: float) -> None:
        """[summary]

        Args:
            time_step_index (int): [description]
            simulation_time (float): [description]
        """
        return

    def post_reset(self) -> None:
        pass

    def calculate_metrics(self) -> dict:
        """[summary]"""
        raise NotImplementedError

    def is_done(self) -> bool:
        """[summary]"""
        raise NotImplementedError
