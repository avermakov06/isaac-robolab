from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False, 'active_gpu': 1})

import numpy as np
from isaacsim.core.api import World
from isaacsim.core.utils.types import ArticulationAction, JointsState
from task_liquid import LiquidTask

my_world = World(stage_units_in_meters=1.0)
my_task = LiquidTask()
my_world.add_task(my_task)

my_world.get_physics_context().enable_gpu_dynamics(True)
my_world.get_physics_context().set_broadphase_type("GPU")

# from omni.isaac.core.utils.stage import set_stage_units, set_stage_up_axis, is_stage_loading
# set_stage_up_axis("y")
# set_stage_units(0.01)  # cm

my_world.reset()
task_params = my_task.get_params()
# robot = my_world.scene.get_object(task_params["robot_name"]["value"])
# my_controller = PickPlaceController(
#     name="pick_place_controller", gripper=my_franka.gripper, robot_articulation=my_franka
# )

i = 0
reset_needed = False

n_dofs = 25
curr_ref = np.array([0.] * n_dofs)
step = 0.2


my_world.stop()

while simulation_app.is_running():
    my_world.step(render=True)
    if my_world.is_stopped() and not reset_needed:
        reset_needed = True
    if my_world.is_playing():
        if reset_needed:
            my_world.reset()
            # my_controller.reset()
            reset_needed = False
        observations = my_world.get_observations()

        if i % 500 == 0:
            step *= -1
        
        curr_ref += step * 3.1415 / 180
        
        i += 1
simulation_app.close()
