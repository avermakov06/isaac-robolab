from isaacsim import SimulationApp

simulation_app = SimulationApp({
    "headless": False,
})

import numpy as np

from isaacsim.core.api import World
from isaacsim.core.utils.types import ArticulationAction
from isaacsim.core.utils.rotations import euler_angles_to_quat

from task_rc5 import RC5Task
from gripper import GripperDemo


def main():
    PICK_PHASE = 0
    
    world = World(stage_units_in_meters=1.0)
    task = RC5Task()
    world.add_task(task)
    world.reset()

    task_params = task.get_params()
    robot = world.scene.get_object(task_params["robot_name"]["value"])
    articulation_controller = robot.get_articulation_controller()
    
    # get primary's pos
    import omni
    from pxr import Usd
    stage = omni.usd.get_context().get_stage()
    g_path = "/World/RC5/frame6/body6"
    prim = stage.GetPrimAtPath(g_path)

    m = omni.usd.get_world_transform_matrix(prim, time_code=Usd.TimeCode.Default())
    pos_gf = m.ExtractTranslation()
    pos = np.array([pos_gf[0], pos_gf[1], pos_gf[2]], dtype=float)
    print(pos)
    
    gripper_demo = GripperDemo(pos)
    gripper_demo.surface_gripper.open()

    i = 0
    reset_needed = False

    n_dofs = 6
    curr_ref = np.array([0.] * n_dofs)
    curr_ref[1] = 0.1
    step = 0.2

    ik, aik = create_IK_solver(task._robot)

    world.stop()

    while simulation_app.is_running():
        world.step(render=True)
        if world.is_stopped() and not reset_needed:
            reset_needed = True
        if world.is_playing():
            if reset_needed:
                world.reset()
                reset_needed = False
            observations = world.get_observations()

            if i % 500 == 0:
                step *= -1
            
            curr_ref += step * 3.1415 / 180
            
            # IK
            robot_base_translation, robot_base_orientation = robot.get_world_pose()
            ik.set_robot_base_pose(robot_base_translation, robot_base_orientation)
            
            actions = None
            
            if PICK_PHASE == 0:
                # get prim's pos
                import omni
                from pxr import Usd
                g_path = "/Box"
                prim = stage.GetPrimAtPath(g_path)
                m = omni.usd.get_world_transform_matrix(prim, time_code=Usd.TimeCode.Default())
                pos_gf = m.ExtractTranslation()
                pos = np.array([pos_gf[0], pos_gf[1], pos_gf[2]], dtype=float)
                target_position = pos
                
                rot = np.array([0., -0.707, 0., -0.707])
                
                if i < 50:
                    # teleport
                    tjp = [ 0.4212939, -1.5068517, -1.3259246,  2.8326983, -1.1495025,  1.5707899]
                    task._robot.set_joint_positions(np.array(tjp))
                else:
                    # target for mesh
                    rot = np.array([0., -0.707, 0., -0.707])
                    target_position[-1] = 0.55
                    action, success = aik.compute_inverse_kinematics(target_position, rot)
                    actions = action

                if i >= 250:
                    PICK_PHASE += 1
            elif PICK_PHASE == 1:
                print(f"Entering phase {PICK_PHASE}...")
                if target_position[-1] >= 0.52:
                    target_position[-1] -= 0.0005
                action, success = aik.compute_inverse_kinematics(target_position, rot)
                actions = action
                if i >= 400:
                    PICK_PHASE += 1
            elif PICK_PHASE == 2:
                print(f"Entering phase {PICK_PHASE}...")
                if i >= 450:
                    PICK_PHASE += 1
            elif PICK_PHASE == 3:
                print(f"Entering phase {PICK_PHASE}...")
                print("CLOSING GRIPPER")
                gripper_demo.surface_gripper.close()  # should close only here!
                if target_position[-1] < 0.55:
                    target_position[-1] += 0.0005
                else:
                    PICK_PHASE += 1
                action, success = aik.compute_inverse_kinematics(target_position, rot)
                actions = action
                k = 0
            elif PICK_PHASE == 4:
                k += 1
                if k > 100:
                    PICK_PHASE += 1
            else:
                print("Opening gripper...")
                gripper_demo.surface_gripper.open()
            
            if actions is not None:
                articulation_controller.apply_action(actions)
            
            i += 1
    simulation_app.close()


from isaacsim.robot_motion.motion_generation import ArticulationKinematicsSolver, LulaKinematicsSolver


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

