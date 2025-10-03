from isaacsim import SimulationApp

simulation_app = SimulationApp({
    "headless": False,
})

import numpy as np

from isaacsim.core.utils.types import ArticulationAction
from isaacsim.core.utils.rotations import euler_angles_to_quat

from env import AgibotGymEnv


def main():
    env = AgibotGymEnv()
    world = env.world
    task_params = env.task_params
    
    i = 0
    reset_needed = False
    
    observations = env.reset()
    
    while simulation_app.is_running():
        # world.step(render=True)
        if world.is_stopped() and not reset_needed:
            reset_needed = True
        if world.is_playing():
            if reset_needed:
                env.reset()
                reset_needed = False
            

            print("current eef pos:", observations[task_params["robot_name"]["value"]]["end_effector_position"])

            eef_action = observations[task_params["cube_name"]["value"]]["position"]
            observations, r, done, info = env.step(eef_action)
            
            i += 1
    simulation_app.close()


if __name__ == "__main__":
    main()
