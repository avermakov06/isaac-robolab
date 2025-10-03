from isaacsim import SimulationApp

simulation_app = SimulationApp({
    "headless": False,
})

import numpy as np
from isaacsim.core.api import World
from isaacsim.core.utils.types import ArticulationAction
from task import AgibotTask
# from isaacsim.robot.manipulators.examples.franka.controllers.pick_place_controller import PickPlaceController
from controller import PickPlaceController


class DummyGripper:
    def forward(self, *args, **kwargs):
        print(f"Gripper's forward got called with args: {args}, {kwargs}")

gripper = DummyGripper()  # was robot.gripper


world = World(stage_units_in_meters=1.0)
task = AgibotTask()
world.add_task(task)
world.reset()
task_params = task.get_params()
robot = world.scene.get_object(task_params["robot_name"]["value"])
articulation_controller = robot.get_articulation_controller()
my_controller = PickPlaceController(
    name="pick_place_controller", gripper=gripper, robot_articulation=robot,
    # end_effector_initial_height=1.172,  # otherwise hardcoded 0.3 is used
    end_effector_initial_height=0.825 + 0.2
)
from isaacsim.core.utils.rotations import euler_angles_to_quat

# from isaacsim.core.utils.rotations import euler_angles_to_quat
# quat = euler_angles_to_quat([0.0, 0.0, -90], degrees=True)
# robot.set_world_pose(orientation=quat)


i = 0
reset_needed = False

n_dofs = 25
curr_ref = np.array([0.] * n_dofs)
step = 0.2

# world.stop()

while simulation_app.is_running():
    world.step(render=True)
    if world.is_stopped() and not reset_needed:
        reset_needed = True
    if world.is_playing():
        if reset_needed:
            world.reset()
            my_controller.reset()
            reset_needed = False
        observations = world.get_observations()

        if i % 500 == 0:
            step *= -1
        
        # curr_ref += step * 3.1415 / 180
        
        actions = ArticulationAction(
            joint_positions=curr_ref
        )
        
        # actions from controller
        curr_joint_positions = observations[task_params["robot_name"]["value"]]["joint_positions"]
        # joint_indices = [4, 9, 11, 13, 15, 17, 19, 21, 23]  # with lift joint
        joint_indices = [8, 10, 12, 14, 16, 18, 20, 22]
        curr_joint_positions = np.array([curr_joint_positions[i] for i in joint_indices])
        
        end_effector_orientation = euler_angles_to_quat(np.array([0, np.pi, 0]))
        # end_effector_orientation = None
        
        actions = my_controller.forward(
            picking_position=observations[task_params["cube_name"]["value"]]["position"],
            placing_position=observations[task_params["cube_name"]["value"]]["position"],
            current_joint_positions=curr_joint_positions,
            end_effector_offset=np.array([0., 0.025, 0.]),
            end_effector_orientation=end_effector_orientation,
            joint_indices=[8, 10, 12, 14, 16, 18, 20, 22],
        )
        
        print("current eef pos:", observations[task_params["robot_name"]["value"]]["end_effector_position"])
        
        if my_controller.is_done():
            print("done picking and placing")
        else:
            articulation_controller.apply_action(actions)
        i += 1
simulation_app.close()
