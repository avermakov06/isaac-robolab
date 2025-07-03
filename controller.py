from typing import List, Optional

import isaacsim.robot.manipulators.controllers as manipulators_controllers
from isaacsim.core.prims import SingleArticulation
from isaacsim.robot.manipulators.grippers.parallel_gripper import ParallelGripper


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


import isaacsim.robot_motion.motion_generation as mg
from isaacsim.robot_motion.motion_generation.interface_config_loader import _process_policy_config
from isaacsim.core.prims import SingleArticulation

# from isaacsim.robot.manipulators.examples.franka.controllers.rmpflow_controller import RMPFlowController
class RMPFlowController(mg.MotionPolicyController):
    """[summary]

    Args:
        name (str): [description]
        robot_articulation (SingleArticulation): [description]
        physics_dt (float, optional): [description]. Defaults to 1.0/60.0.
    """

    def __init__(self, name: str, robot_articulation: SingleArticulation, physics_dt: float = 1.0 / 60.0) -> None:
        # self.rmp_flow_config = mg.interface_config_loader.load_supported_motion_policy_config("Franka", "RMPflow")
        
        self.rmp_flow_config = _process_policy_config("standalone_examples/isaac-robolab/controller_configs/rmpflow/config.json")
        
        self.rmp_flow = mg.lula.motion_policies.RmpFlow(**self.rmp_flow_config)

        self.articulation_rmp = mg.ArticulationMotionPolicy(robot_articulation, self.rmp_flow, physics_dt)

        mg.MotionPolicyController.__init__(self, name=name, articulation_motion_policy=self.articulation_rmp)
        (
            self._default_position,
            self._default_orientation,
        ) = self._articulation_motion_policy._robot_articulation.get_world_pose()
        self._motion_policy.set_robot_base_pose(
            robot_position=self._default_position, robot_orientation=self._default_orientation
        )
        return

    def reset(self):
        mg.MotionPolicyController.reset(self)
        self._motion_policy.set_robot_base_pose(
            robot_position=self._default_position, robot_orientation=self._default_orientation
        )
