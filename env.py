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
from robot import Agibot


# from isaaclab.envs import DirectRLEnvCfg, DirectRLEnv
# from isaaclab.scene import InteractiveSceneCfg
# from isaaclab.utils import configclass

# @configclass
# class AgibotEnvCfg(DirectRLEnvCfg):
#    ...
#    action_space = 1
#    observation_space = 4
#    state_space = 0
   
#    decimation = 1
#    episode_length_s = 100
#    scene = InteractiveSceneCfg(
#         num_envs=1,
#         env_spacing=None,
#    )


# class AgibotEnv(DirectRLEnv):
#     cfg: AgibotEnvCfg

#     def __init__(self, cfg: AgibotEnvCfg, render_mode: str | None = None, **kwargs):
#         super().__init__(cfg, render_mode, **kwargs)

#     def _setup_scene(self):
#         self.task = AgibotTask()

#     def _get_rewards(self):
#         total_reward = 0.0
#         return total_reward

#     def _get_observations(self) -> dict:
#         return self.task.get_observations()

#     def _get_dones(self):
#         a = False
#         b = False
#         return a, b

#     def _reset_idx(self, env_ids):
#         return
    
#     def _pre_physics_step(self, actions) -> None:
#         self.actions = actions


import gymnasium as gym
from gymnasium import spaces


class AgibotGymEnv(gym.Env):
    def __init__(self):
        super().__init__()
        self.observation_space = spaces.Dict({
            "cube_position": spaces.Box(-np.inf, np.inf, (3,), np.float32),
            "cube_orientation": spaces.Box(-np.inf, np.inf, (4,), np.float32),
            "cube_target_position": spaces.Box(-np.inf, np.inf, (3,), np.float32),
            "joint_positions": spaces.Box(-np.inf, np.inf, (24,), np.float32),
            "end_effector_position": spaces.Box(-np.inf, np.inf, (3,), np.float32),
        })
        self.action_space = spaces.Box(-100., 100., (3,), np.float32)
        
        self.task = AgibotTask()
        
        from isaacsim.core.api import World
        world = World(stage_units_in_meters=1.0)
        world.add_task(self.task)
        self.world = world
        world.reset()  # required to init controller
        
        self._init_controller()
    
    def _init_controller(self):
        class DummyGripper:
            def forward(self, *args, **kwargs):
                print(f"Gripper's forward got called with args: {args}, {kwargs}")

        gripper = DummyGripper()  # was robot.gripper

        task_params =self.task.get_params()
        robot = self.world.scene.get_object(task_params["robot_name"]["value"])
        self.task_params = task_params
        self.articulation_controller = robot.get_articulation_controller()
        
        from controller import PickPlaceController
        
        self.custom_controller = PickPlaceController(
            name="pick_place_controller", gripper=gripper, robot_articulation=robot,
            # end_effector_initial_height=1.172,  # otherwise hardcoded 0.3 is used
            end_effector_initial_height=0.825 + 0.2
        )
    
    def _apply_action(self, eef_action, eef_orientation, curr_joint_pos):
        # fix event for just IK
        self.custom_controller._event = 0
        actions = self.custom_controller.forward(
            picking_position=eef_action,
            placing_position=eef_action,
            current_joint_positions=curr_joint_pos,
            end_effector_offset=np.array([0., 0.025, 0.]),
            end_effector_orientation=eef_orientation,
            joint_indices=[8, 10, 12, 14, 16, 18, 20, 22],
        )
        if self.custom_controller.is_done():
            print("done picking and placing")
        else:
            self.articulation_controller.apply_action(actions)
    
    def reset(self, **kwargs):
        self.world.reset()
        self.custom_controller.reset()
        return self.world.get_observations(), {}

    def step(self, action):
        from isaacsim.core.utils.rotations import euler_angles_to_quat
        
        obs_prev = self.world.get_observations()
        curr_joint_positions = obs_prev["joint_positions"]
        joint_indices = [8, 10, 12, 14, 16, 18, 20, 22]
        curr_joint_positions = np.array([curr_joint_positions[i] for i in joint_indices])
        
        end_effector_orientation = euler_angles_to_quat(np.array([0, np.pi, 0]))
            
        self._apply_action(action, end_effector_orientation, curr_joint_positions)
        
        self.world.step(render=True)
        
        obs = self.world.get_observations()
        r = self.compute_reward(obs)
        truncated = False
        terminated = False
        
        return obs, r, truncated, terminated, {}
    
    def compute_reward(self, obs):
        cube_dist = np.linalg.norm(obs["cube_position"] - obs["cube_target_position"]).mean()
        eef_dist = np.linalg.norm(obs["end_effector_position"] - obs["cube_position"]).mean()
        r = 0.1 * cube_dist + 0.01 * eef_dist
        return r


MOVE_TO_TABLE = False


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
        # self.set_cube(scene)
        self.set_visual_sphere(scene)
        self.set_table_and_cube(scene)
        self._move_task_objects_to_their_frame()
        
        # save eef prim
        import omni.usd
        xform_path = "/World/Agibot/left_arm_link07/left_finger_base/left_finger_right"
        stage = omni.usd.get_context().get_stage()
        prim = stage.GetPrimAtPath(xform_path)
        self._eef = prim
        
        return

    def set_robot(self) -> Agibot:
        robot_prim_path = find_unique_string_name(
            initial_name="/World/Agibot", is_unique_fn=lambda x: not is_prim_path_valid(x)
        )
        robot_name = find_unique_string_name(
            initial_name="agibot", is_unique_fn=lambda x: not self.scene.object_exists(x)
        )
        
        ASSET_FILE_NAME = "agibot_tuned.usd"
        assets_path = Path(__file__).parent / "assets"
        agibot_asset_path = (assets_path / ASSET_FILE_NAME).as_posix()
        table_scene_asset_path = (assets_path / "table_scene.usd").as_posix()
        
        position = [0., 0., 0.]
        # position = [0., -0.3, 0.]
        orientation = None
        
        if MOVE_TO_TABLE:
            position[1] -= 0.5
            orientation = [0.707, 0., 0., 0.707]
        
        robot = Agibot(
            prim_path=robot_prim_path,
            name=robot_name,
            usd_path=agibot_asset_path,
            position=position,
            orientation=orientation,
            gripper_dof_names=[""]
        )
        
        # set aux USD
        add_reference_to_stage(usd_path=table_scene_asset_path, prim_path="/World/table_scene")
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

    # def set_cube(self, scene):
    #     cube_name = "cube"
    #     cube_prim_path = "/World/cube_to_pick"
    #     self._cube = scene.add(
    #         DynamicCuboid(
    #             name=cube_name,
    #             position=[0.47, -1.2, 1.21],
    #             orientation=None,
    #             prim_path=cube_prim_path,
    #             scale=[0.025] * 3,
    #             # size=1.0,
    #             color=np.array([0, 0, 1]),
    #         )
    #     )
    #     self._task_objects[self._cube.name] = self._cube

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
        
        position, orientation = self._cube.get_local_pose()
        params_representation["cube_position"] = {"value": position, "modifiable": True}
        params_representation["cube_orientation"] = {"value": orientation, "modifiable": True}
        params_representation["cube_name"] = {"value": self._cube.name, "modifiable": False}
        
        if hasattr(self, "_sphere"):
            position, orientation = self._sphere.get_local_pose()
            params_representation["sphere_position"] = {"value": position, "modifiable": True}
            params_representation["sphere_orientation"] = {"value": orientation, "modifiable": True}
            params_representation["sphere_name"] = {"value": self._sphere.name, "modifiable": False}
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
        cube_position, cube_orientation = self._cube.get_local_pose()
        # end_effector_position, _ = self._robot.end_effector.get_local_pose()
        
        eef_pos = self.get_eef_pos()
        
        # obs = {
        #     self._cube.name: {
        #         "position": cube_position,
        #         "orientation": cube_orientation,
        #         # "target_position": self._target_position,
        #     },
        #     self._robot.name: {
        #         "joint_positions": joints_state.positions,
        #         # "end_effector_position": end_effector_position,
        #         "end_effector_position": eef_pos,
        #     },
        # }
        # if hasattr(self, "_sphere"):
        #     p, q = self._sphere.get_local_pose()
        #     obs[self._sphere.name] = {
        #         "position": p,
        #         "orientation": q,
        #     }
        
        # sb3 does not support nested spaces
        obs = {
            "cube_position": cube_position,
            "cube_orientation": cube_orientation,
            "cube_target_position": self._target_pos,
            "joint_positions": joints_state.positions,
            # "end_effector_position": end_effector_position,
            "end_effector_position": eef_pos,
        }
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
