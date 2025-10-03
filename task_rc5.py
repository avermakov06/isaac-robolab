from abc import ABC, abstractmethod
from pathlib import Path
from typing import Optional

import numpy as np
from isaacsim.core.api.objects import DynamicCuboid, VisualSphere, FixedCuboid
from isaacsim.core.api.scenes.scene import Scene
from isaacsim.core.api.tasks import BaseTask
from isaacsim.core.utils.prims import is_prim_path_valid
from isaacsim.core.utils.stage import add_reference_to_stage, get_stage_units
from isaacsim.core.utils.string import find_unique_string_name
from robot_rc5 import RC5



class RC5Task(BaseTask):
    """
    Args:
        name (str): [description]
        offset (Optional[np.ndarray], optional): [description]. Defaults to None.
    """

    def __init__(
        self,
        name: str = 'rc5_task',
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
        self.set_cube(scene)
        # self.set_visual_sphere(scene)
        # self.set_table_and_cube(scene)
        self._move_task_objects_to_their_frame()
        
        # save eef prim
        # import omni.usd
        # xform_path = "/World/Agibot/left_arm_link07/left_finger_base/left_finger_right"
        # stage = omni.usd.get_context().get_stage()
        # prim = stage.GetPrimAtPath(xform_path)
        # self._eef = prim

    def set_robot(self) -> RC5:
        robot_prim_path = find_unique_string_name(
            initial_name="/World/RC5", is_unique_fn=lambda x: not is_prim_path_valid(x)
        )
        robot_name = find_unique_string_name(
            initial_name="rc5", is_unique_fn=lambda x: not self.scene.object_exists(x)
        )
        
        # ASSET_FILE_NAME = "rc5-rigged-drives-tuned.usd"
        ASSET_FILE_NAME = "urdf_rc5/Robot/Robot.usd"
        assets_path = Path(__file__).parent / "assets"
        robot_asset_path = (assets_path / ASSET_FILE_NAME).as_posix()
        # table_scene_asset_path = (assets_path / "table_scene.usd").as_posix()
        
        # position = [0., 0., 0.]
        # position = [0., -0.3, 0.]
        # orientation = None
        
        # if MOVE_TO_TABLE:
            # position[1] -= 0.5
            # orientation = [0.707, 0., 0., 0.707]
        
        robot = RC5(
            prim_path=robot_prim_path,
            name=robot_name,
            usd_path=robot_asset_path,
            gripper_dof_names=[""]
        )
        
        # set aux USD
        # add_reference_to_stage(usd_path=table_scene_asset_path, prim_path="/World/table_scene")
        # pallet_path = (assets_path / "cad/pallet_v1.usd").as_posix()
        # add_reference_to_stage(usd_path=pallet_path, prim_path="/World/pallet")

        return robot

    def set_visual_sphere(self, scene):
        # p = [0.809, 0.094, 0.932]  # at the front, 1st one
        p = [0.075, 1.171, 1.172]  # in hand's initial pos
        # p = [0.385, 0.975, 1.172]  # a bit moved from init pos
        scale = 0.1 / 2
        name = "sphere"
        prim_path = "/World/sphere"
        self._sphere = scene.add(
            VisualSphere(
                prim_path=prim_path,
                name=name,
                translation=p,
                scale=[scale] * 3,
                color=np.array([0, 1, 0])
            )
        )
        self._task_objects[self._sphere.name] = self._sphere

    def set_cube(self, scene):
        cube_name = "cube"
        cube_prim_path = "/World/cube_to_pick"
        self._cube = scene.add(
            DynamicCuboid(
                name=cube_name,
                position=[0., 0.5 + 0.25 / 2, 0.],
                orientation=None,
                prim_path=cube_prim_path,
                scale=[0.1] * 3,
                # size=1.0,
                color=np.array([0, 0, 1]),
            )
        )
        self._task_objects[self._cube.name] = self._cube

    def set_table_and_cube(self, scene):
        self._table = scene.add(
            FixedCuboid(
                prim_path="/World/table",
                name="table",
                position=[1.12, 0., 0.4],
                scale=[0.8] * 3,
            )
        )
        self._task_objects[self._table.name] = self._table
        
        self._cube = scene.add(
            DynamicCuboid(
                prim_path="/World/cube",
                name="cube",
                position=[0.8, 0.3, 0.825],
                scale=[0.05] * 3,
                color=np.array([1., 0., 0.]),
            )
        )
        self._task_objects[self._cube.name] = self._cube
        
        self._target_pos = [0.8, -0.15, 0.825]
    
    def get_params(self) -> dict:
        params_representation = dict()
        params_representation["robot_name"] = {"value": self._robot.name, "modifiable": False}
        
        # position, orientation = self._cube.get_local_pose()
        # params_representation["cube_position"] = {"value": position, "modifiable": True}
        # params_representation["cube_orientation"] = {"value": orientation, "modifiable": True}
        # params_representation["cube_name"] = {"value": self._cube.name, "modifiable": False}
        
        # if hasattr(self, "_sphere"):
        #     position, orientation = self._sphere.get_local_pose()
        #     params_representation["sphere_position"] = {"value": position, "modifiable": True}
        #     params_representation["sphere_orientation"] = {"value": orientation, "modifiable": True}
        #     params_representation["sphere_name"] = {"value": self._sphere.name, "modifiable": False}
        return params_representation

    def get_eef_pos(self):
        import omni.usd
        from pxr import Gf
        import numpy as np

        prim = self._eef

        world_transform = omni.usd.get_world_transform_matrix(prim)
        pos_gf = world_transform.ExtractTranslation()
        pos_np = np.array([pos_gf[0], pos_gf[1], pos_gf[2]])

        return pos_np

    def get_observations(self) -> dict:
        """[summary]

        Returns:
            dict: [description]
        """
        joints_state = self._robot.get_joints_state()
        # cube_position, cube_orientation = self._cube.get_local_pose()
        # end_effector_position, _ = self._robot.end_effector.get_local_pose()
        
        # eef_pos = self.get_eef_pos()
        
        obs = {
            # self._cube.name: {
            #     "position": cube_position,
            #     "orientation": cube_orientation,
            #     # "target_position": self._target_position,
            # },
            self._robot.name: {
                "joint_positions": joints_state.positions,
                # "end_effector_position": end_effector_position,
                # "end_effector_position": eef_pos,
            },
        }
        # if hasattr(self, "_sphere"):
        #     p, q = self._sphere.get_local_pose()
        #     obs[self._sphere.name] = {
        #         "position": p,
        #         "orientation": q,
        #     }
        return obs

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
