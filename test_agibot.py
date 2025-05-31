from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

import numpy as np
from isaacsim.core.api import World
from isaacsim.core.utils.types import ArticulationAction
from task import AgibotTask

world = World(stage_units_in_meters=1.0)
task = AgibotTask()
world.add_task(task)
world.reset()
task_params = task.get_params()
robot = world.scene.get_object(task_params["robot_name"]["value"])
articulation_controller = robot.get_articulation_controller()

i = 0
reset_needed = False

n_dofs = 25
curr_ref = np.array([0.] * n_dofs)
step = 0.2


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
        
        actions = ArticulationAction(
            joint_positions=curr_ref
        )
        articulation_controller.apply_action(actions)
        i += 1
simulation_app.close()
