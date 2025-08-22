from isaacsim import SimulationApp

simulation_app = SimulationApp({
    "headless": False,
})

import numpy as np
from isaacsim.core.api import World
from isaacsim.core.utils.types import ArticulationAction
from task_rc5 import RC5Task
# from isaacsim.robot.manipulators.examples.franka.controllers.pick_place_controller import PickPlaceController
from controller import PickPlaceController


def main():
    class DummyGripper:
        def forward(self, *args, **kwargs):
            print(f"Gripper's forward got called with args: {args}, {kwargs}")

    gripper = DummyGripper()  # was robot.gripper


    world = World(stage_units_in_meters=1.0)
    task = RC5Task()
    world.add_task(task)
    world.reset()
    task_params = task.get_params()
    robot = world.scene.get_object(task_params["robot_name"]["value"])
    articulation_controller = robot.get_articulation_controller()
    # my_controller = PickPlaceController(
    #     name="pick_place_controller", gripper=gripper, robot_articulation=robot,
    #     # end_effector_initial_height=1.172,  # otherwise hardcoded 0.3 is used
    #     end_effector_initial_height=0.825 + 0.2
    # )
    from isaacsim.core.utils.rotations import euler_angles_to_quat

    # from isaacsim.core.utils.rotations import euler_angles_to_quat
    # quat = euler_angles_to_quat([0.0, 0.0, -90], degrees=True)
    # robot.set_world_pose(orientation=quat)


    i = 0
    reset_needed = False

    n_dofs = 6
    curr_ref = np.array([0.] * n_dofs)
    curr_ref[1] = 0.1
    step = 0.2

    ik, aik = create_IK_solver(task._robot)

    # world.stop()

    while simulation_app.is_running():
        world.step(render=True)
        if world.is_stopped() and not reset_needed:
            reset_needed = True
        if world.is_playing():
            if reset_needed:
                world.reset()
                # my_controller.reset()
                reset_needed = False
            observations = world.get_observations()

            if i % 500 == 0:
                step *= -1
            
            curr_ref += step * 3.1415 / 180
            
            # actions = ArticulationAction(
            #     joint_positions=curr_ref
            # )
            
            # IK
            robot_base_translation, robot_base_orientation = robot.get_world_pose()
            ik.set_robot_base_pose(robot_base_translation, robot_base_orientation)
            target_position, target_orientation = task._cube.get_world_pose()
            # rot = np.array([0.7071, 0.7071, 0, 0])
            rot = np.array([0, -1, 0, 0])
            target_position[-1] = 0.3
            action, success = aik.compute_inverse_kinematics(target_position, rot)
            
            print(success)
            print(action)
            actions = action
            
            # actions from controller
            curr_joint_positions = observations[task_params["robot_name"]["value"]]["joint_positions"]
            # joint_indices = [4, 9, 11, 13, 15, 17, 19, 21, 23]  # with lift joint
            # joint_indices = [8, 10, 12, 14, 16, 18, 20, 22]
            # curr_joint_positions = np.array([curr_joint_positions[i] for i in joint_indices])
            
            end_effector_orientation = euler_angles_to_quat(np.array([0, np.pi, 0]))
            # end_effector_orientation = None
            
            # actions = my_controller.forward(
            #     picking_position=observations[task_params["cube_name"]["value"]]["position"],
            #     placing_position=observations[task_params["cube_name"]["value"]]["position"],
            #     current_joint_positions=curr_joint_positions,
            #     end_effector_offset=np.array([0., 0.025, 0.]),
            #     end_effector_orientation=end_effector_orientation,
            #     joint_indices=[8, 10, 12, 14, 16, 18, 20, 22],
            # )
            
            # print("current eef pos:", observations[task_params["robot_name"]["value"]]["end_effector_position"])
            
            # if my_controller.is_done():
            #     print("done picking and placing")
            # else:
                # articulation_controller.apply_action(actions)
            
            
            articulation_controller.apply_action(actions)
            
            i += 1
    simulation_app.close()


from isaacsim.core.utils.extensions import get_extension_path_from_name
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.core.prims import Articulation
from isaacsim.core.utils.nucleus import get_assets_root_path
from isaacsim.core.prims import XFormPrim
from isaacsim.core.utils.numpy.rotations import euler_angles_to_quats

from isaacsim.robot_motion.motion_generation import ArticulationKinematicsSolver, LulaKinematicsSolver
from isaacsim.robot_motion.motion_generation import interface_config_loader


def create_IK_solver(robot):
    kinematics_solver = LulaKinematicsSolver(
        robot_description_path="controller_configs/rc5_official_urdf.yaml",
        urdf_path="assets/urdf_rc5/Robot.urdf",
    )
    # print("Valid frame names at which to compute kinematics:", kinematics_solver.get_all_frame_names())
    end_effector_name = "body6"
    articulation_kinematics_solver = ArticulationKinematicsSolver(robot, kinematics_solver, end_effector_name)
    return kinematics_solver, articulation_kinematics_solver
    

if __name__ == "__main__":
    main()
