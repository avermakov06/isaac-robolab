from isaacsim import SimulationApp

simulation_app = SimulationApp({
    "headless": False,
})

import numpy as np
from isaacsim.core.api import World
from isaacsim.core.utils.types import ArticulationAction
from task_rc5 import RC5Task
from isaacsim.robot_motion.motion_generation import ArticulationKinematicsSolver, LulaKinematicsSolver
from controller import PickPlaceController
from isaacsim.core.utils.rotations import euler_angles_to_quat


def main():
    class DummyGripper:
        def forward(self, *args, **kwargs):
            print(f"Gripper's forward got called with args: {args}, {kwargs}")

    gripper = DummyGripper()

    world = World(stage_units_in_meters=1.0)
    task = RC5Task()
    world.add_task(task)
    world.reset()
    task_params = task.get_params()
    robot = world.scene.get_object(task_params["robot_name"]["value"])
    articulation_controller = robot.get_articulation_controller()

    i = 0
    reset_needed = False

    ik, aik = create_IK_solver(robot)

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
                print(f"Step {i}")

            # IK - исправленная часть
            target_position = np.array([0.374, 0.485, 0.168])  # позиция
            euler_angles = np.array([-180., 0., -45.])  # углы Эйлера в градусах
            
            # Преобразуем углы Эйлера в кватернион
            target_orientation = euler_angles_to_quat(euler_angles, degrees=True)
            
            robot_base_translation, robot_base_orientation = robot.get_world_pose()
            ik.set_robot_base_pose(robot_base_translation, robot_base_orientation)
            
            # Теперь передаем кватернион вместо углов Эйлера
            action, success = aik.compute_inverse_kinematics(target_position, target_orientation)
            
            print(f"Success: {success}")
            print(f"Action: {action}")
            
            if success:
                articulation_controller.apply_action(action)
            else:
                print("IK failed!")
            
            i += 1
    simulation_app.close()


def create_IK_solver(robot):
    kinematics_solver = LulaKinematicsSolver(
        robot_description_path="controller_configs/rc5_official_urdf.yaml",
        urdf_path="assets/urdf_rc5/Robot.urdf",
    )
    end_effector_name = "body6"
    articulation_kinematics_solver = ArticulationKinematicsSolver(robot, kinematics_solver, end_effector_name)
    return kinematics_solver, articulation_kinematics_solver
    

if __name__ == "__main__":
    main()