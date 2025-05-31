from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

import numpy as np
from isaacsim.core.api import World
from isaacsim.core.utils.stage import get_stage_units
from task_spill_liquid import PickPlace, PickPlaceController

# used only here
# FRANKA_POS = np.array([0., -25., 46.])
FRANKA_POS = np.array([0., 0., 0.])
CUBE_SIZE = np.array([0.0515, 0.0515, 0.0515]) / get_stage_units() * 100.
CUBE_INIT_POS = FRANKA_POS + np.array([0.3, 0., 0.]) / get_stage_units() * 100.
TARGET_POSITION = FRANKA_POS + np.array([0.3, -0.3, 0]) / get_stage_units() * 100
TARGET_POSITION[2] = CUBE_SIZE[2] / 2.0

world = World(stage_units_in_meters=1.0)

world.get_physics_context().enable_gpu_dynamics(True)
world.get_physics_context().set_broadphase_type("GPU")


task = PickPlace(
    cube_initial_position=CUBE_INIT_POS,
    target_position=TARGET_POSITION,
    cube_size=CUBE_SIZE,
)
world.add_task(task)
world.reset()
    
task_params = task.get_params()
my_franka = world.scene.get_object(task_params["robot_name"]["value"])
my_controller = PickPlaceController(
    name="pick_place_controller", gripper=my_franka.gripper, robot_articulation=my_franka
)
articulation_controller = my_franka.get_articulation_controller()

world.stop()

started = False
i = 0
reset_needed = False
while simulation_app.is_running():
    world.step(render=True)
    if world.is_stopped() and not reset_needed:
        reset_needed = True
    if world.is_playing():
        if reset_needed:
            world.reset()
            # my_controller.reset()
            reset_needed = False
        
        if i > 100:
            if not started:
                print("Started to move")
                started = True
            observations = world.get_observations()
        i += 1
simulation_app.close()
