from abc import ABC, abstractmethod
from typing import Optional

import numpy as np
from franka_with_scaling import Franka
from isaacsim.core.api.objects import DynamicCuboid
from isaacsim.core.api.scenes.scene import Scene
from isaacsim.core.api.tasks import BaseTask
from isaacsim.core.utils.prims import is_prim_path_valid
from isaacsim.core.utils.stage import add_reference_to_stage, get_stage_units
from isaacsim.core.utils.string import find_unique_string_name
from omni.isaac.core.prims import XFormPrim
from task_liquid import LiquidTask


# from exts/isaacsim.core.api/isaacsim/core/api/tasks/pick_place.py
class PickPlaceBase(ABC, BaseTask):
    """[summary]

    Args:
        name (str): [description]
        cube_initial_position (Optional[np.ndarray], optional): [description]. Defaults to None.
        cube_initial_orientation (Optional[np.ndarray], optional): [description]. Defaults to None.
        target_position (Optional[np.ndarray], optional): [description]. Defaults to None.
        cube_size (Optional[np.ndarray], optional): [description]. Defaults to None.
        offset (Optional[np.ndarray], optional): [description]. Defaults to None.
    """

    def __init__(
        self,
        name: str,
        cube_initial_position: Optional[np.ndarray] = None,
        cube_initial_orientation: Optional[np.ndarray] = None,
        target_position: Optional[np.ndarray] = None,
        cube_size: Optional[np.ndarray] = None,
        offset: Optional[np.ndarray] = None,
    ) -> None:
        BaseTask.__init__(self, name=name, offset=offset)
        self._robot = None
        self._target_cube = None
        self._cube = None
        self._cube_initial_position = cube_initial_position
        self._cube_initial_orientation = cube_initial_orientation
        self._target_position = target_position
        self._cube_size = cube_size
        if self._cube_size is None:
            self._cube_size = np.array([0.0515, 0.0515, 0.0515]) / get_stage_units()
        if self._cube_initial_position is None:
            self._cube_initial_position = np.array([0.3, 0.3, 0.3]) / get_stage_units()
        if self._cube_initial_orientation is None:
            self._cube_initial_orientation = np.array([1, 0, 0, 0])
        if self._target_position is None:
            self._target_position = np.array([-0.3, -0.3, 0]) / get_stage_units()
            self._target_position[2] = self._cube_size[2] / 2.0
        self._target_position = self._target_position + self._offset
        return

    def set_up_scene(self, scene: Scene) -> None:
        """[summary]

        Args:
            scene (Scene): [description]
        """
        super().set_up_scene(scene)
        scene.add_default_ground_plane()
        cube_prim_path = find_unique_string_name(
            initial_name="/World/Cube", is_unique_fn=lambda x: not is_prim_path_valid(x)
        )
        cube_name = find_unique_string_name(initial_name="cube", is_unique_fn=lambda x: not self.scene.object_exists(x))
        self._cube = scene.add(
            DynamicCuboid(
                name=cube_name,
                position=self._cube_initial_position,
                orientation=self._cube_initial_orientation,
                prim_path=cube_prim_path,
                scale=self._cube_size,
                size=1.0,
                color=np.array([0, 0, 1]),
            )
        )
        self._task_objects[self._cube.name] = self._cube
        self._robot = self.set_robot()
        scene.add(self._robot)
        self._task_objects[self._robot.name] = self._robot
        self._move_task_objects_to_their_frame()
        return

    @abstractmethod
    def set_robot(self) -> None:
        raise NotImplementedError

    def set_params(
        self,
        cube_position: Optional[np.ndarray] = None,
        cube_orientation: Optional[np.ndarray] = None,
        target_position: Optional[np.ndarray] = None,
    ) -> None:
        if target_position is not None:
            self._target_position = target_position
        if cube_position is not None or cube_orientation is not None:
            self._cube.set_local_pose(translation=cube_position, orientation=cube_orientation)
        return

    def get_params(self) -> dict:
        params_representation = dict()
        position, orientation = self._cube.get_local_pose()
        params_representation["cube_position"] = {"value": position, "modifiable": True}
        params_representation["cube_orientation"] = {"value": orientation, "modifiable": True}
        params_representation["target_position"] = {"value": self._target_position, "modifiable": True}
        params_representation["cube_name"] = {"value": self._cube.name, "modifiable": False}
        params_representation["robot_name"] = {"value": self._robot.name, "modifiable": False}
        return params_representation

    def get_observations(self) -> dict:
        """[summary]

        Returns:
            dict: [description]
        """
        joints_state = self._robot.get_joints_state()
        cube_position, cube_orientation = self._cube.get_local_pose()
        end_effector_position, _ = self._robot.end_effector.get_local_pose()
        return {
            self._cube.name: {
                "position": cube_position,
                "orientation": cube_orientation,
                "target_position": self._target_position,
            },
            self._robot.name: {
                "joint_positions": joints_state.positions,
                "end_effector_position": end_effector_position,
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
        from isaacsim.robot.manipulators.grippers.parallel_gripper import ParallelGripper

        if isinstance(self._robot.gripper, ParallelGripper):
            self._robot.gripper.set_joint_positions(self._robot.gripper.joint_opened_positions)
        return

    def calculate_metrics(self) -> dict:
        """[summary]"""
        raise NotImplementedError

    def is_done(self) -> bool:
        """[summary]"""
        raise NotImplementedError


class PickPlace(PickPlaceBase):
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
        name: str = "franka_pick_place",
        cube_initial_position: Optional[np.ndarray] = None,
        cube_initial_orientation: Optional[np.ndarray] = None,
        target_position: Optional[np.ndarray] = None,
        cube_size: Optional[np.ndarray] = None,
        offset: Optional[np.ndarray] = None,
    ) -> None:
        PickPlaceBase.__init__(
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
        
        # set aux USD
        # USD_SCENE_SCALE = 100.
        # scene_prim_path = "/World/table_scene"
        # add_reference_to_stage(usd_path="/isaac-sim/standalone_examples/agibot/assets/table_scene.usd", prim_path=scene_prim_path)
        # _ = XFormPrim(scene_prim_path, scale=np.array([USD_SCENE_SCALE] * 3))
        
        import omni
        self.stage = omni.usd.get_context().get_stage()
        
        self.iso_surface = True
        
        stage = self.stage
        from argparse import Namespace
        from pxr import Gf, PhysxSchema, UsdPhysics, UsdShade

        stage_properties = Namespace(
                gravity_direction=[0, 0, -1],
                gravity_magnitude=981 / 2,
            )

        physicsScenePath = "/physicsScene"
        scene = UsdPhysics.Scene.Get(stage, physicsScenePath)
        if not scene:
            scene = UsdPhysics.Scene.Define(stage, physicsScenePath)

        gravityDirection = stage_properties.gravity_direction
        gravityDirection = Gf.Vec3f(gravityDirection[0], gravityDirection[1],  gravityDirection[2])

        scene.CreateGravityDirectionAttr().Set(gravityDirection)

        gravityMagnitude = stage_properties.gravity_magnitude
        scene.CreateGravityMagnitudeAttr().Set(gravityMagnitude)
        
        # cup_position = None
        cup_position = [45., -36., 46.]
        
        cup = LiquidTask.set_cup(self, 1.0, cup_position=cup_position)
        self._cup = XFormPrim(cup.GetPath().pathString)
        self.scene.add(self._cup)
        
        return Franka(prim_path=franka_prim_path, name=franka_robot_name, scale=[100.] * 3,
                      position=[-55., -25., 46.]
        )

    def _wait_for_loading(self):
        sim = SimulationContext.instance()
        sim.render()
        while is_stage_loading():
            sim.render()

    def get_observations(self):
        obs = super().get_observations()
        
        position, orientation = self._cup.get_local_pose()
        
        obs["cup"] = {
            "position": position
        }
        return obs
        

from abc import ABC, abstractmethod
from typing import List, Optional

import isaacsim.robot.manipulators.controllers as manipulators_controllers
import numpy as np
from isaacsim.core.api.objects import DynamicCuboid
from isaacsim.core.api.scenes.scene import Scene
from isaacsim.core.api.tasks import BaseTask
from isaacsim.core.prims import SingleArticulation
from isaacsim.core.utils.prims import is_prim_path_valid
from isaacsim.core.utils.stage import get_stage_units
from isaacsim.core.utils.string import find_unique_string_name
from isaacsim.robot.manipulators.examples.franka.controllers.rmpflow_controller import \
    RMPFlowController
from isaacsim.robot.manipulators.grippers.parallel_gripper import \
    ParallelGripper
from omni.isaac.core.simulation_context import SimulationContext
from omni.isaac.core.utils.stage import (is_stage_loading, set_stage_units,
                                         set_stage_up_axis)


class PickPlaceController(manipulators_controllers.PickPlaceController):
    """[summary]

    Args:
        name (str): [description]
        gripper (ParallelGripper): [description]
        robot_articulation (SingleArticulation): [description]
        end_effector_initial_height (Optional[float], optional): [description]. Defaults to None.
        events_dt (Optional[List[float]], optional): [description]. Defaults to None.
    """

    def __init__(
        self,
        name: str,
        gripper: ParallelGripper,
        robot_articulation: SingleArticulation,
        end_effector_initial_height: Optional[float] = None,
        events_dt: Optional[List[float]] = None,
    ) -> None:
        if events_dt is None:
            events_dt = [0.008, 0.005, 1, 0.1, 0.05, 0.05, 0.0025, 1, 0.008, 0.08]
        manipulators_controllers.PickPlaceController.__init__(
            self,
            name=name,
            cspace_controller=RMPFlowController(
                name=name + "_cspace_controller", robot_articulation=robot_articulation
            ),
            gripper=gripper,
            end_effector_initial_height=end_effector_initial_height,
            events_dt=events_dt,
        )
        return
