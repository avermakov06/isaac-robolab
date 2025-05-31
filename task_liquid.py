from abc import ABC, abstractmethod
from argparse import Namespace
from typing import Optional
from pathlib import Path

import numpy as np
from isaacsim.core.api.objects import DynamicCuboid
from isaacsim.core.api.scenes.scene import Scene
from isaacsim.core.api.tasks import BaseTask
from isaacsim.core.utils.prims import is_prim_path_valid
from isaacsim.core.utils.stage import add_reference_to_stage, get_stage_units
from isaacsim.core.utils.string import find_unique_string_name
from omni.isaac.core.prims import XFormPrim
from omni.isaac.core.simulation_context import SimulationContext
from omni.isaac.core.utils.prims import (delete_prim,
                                         get_all_matching_child_prims,
                                         get_prim_at_path, is_prim_path_valid)
from omni.isaac.core.utils.stage import (is_stage_loading, set_stage_units,
                                         set_stage_up_axis)
from robot import Agibot


class LiquidTask(BaseTask):
    """
    Args:
        name (str): [description]
        offset (Optional[np.ndarray], optional): [description]. Defaults to None.
    """

    def __init__(
        self,
        name: str = 'liquid_task',
        offset: Optional[np.ndarray] = None,
    ) -> None:
        BaseTask.__init__(self, name=name, offset=offset)
        self._cup = None
        self.iso_surface = False

    def set_up_scene(self, scene: Scene) -> None:
        """[summary]

        Args:
            scene (Scene): [description]
        """
        super().set_up_scene(scene)
        scene.add_default_ground_plane()
        
        import omni
        self.stage = omni.usd.get_context().get_stage()
        
        # after stage and before loading objects
        
        set_up_physics_secne(self.stage)
        
        CUP_AND_PARTICLES_SCALE = 1.0  # 2022.1.1 units conversion
        cup = self.set_cup(CUP_AND_PARTICLES_SCALE)
        
        self._cup = XFormPrim(cup.GetPath().pathString)
        scene.add(self._cup)
        
        self._move_task_objects_to_their_frame()
        return

    def set_cup(self, SCALE, cup_position=None):
        if cup_position is None:
            cup_position = [0., 16., 0.]
        
        from environment.constants import PREDICATE
        from pxr import Gf, Sdf, UsdGeom
        
        def cup_shape_predicate(prim_path: str):
            prim = get_prim_at_path(prim_path)
            if "cupShape" in prim.GetPath().pathString and prim.GetTypeName() == "Mesh":
                return True
                                    
            return False
        
        cup_path = Path(__file__).parent / "assets" / "cup.usd"
        param = Namespace(
            usd_path=cup_path.as_posix(),  # from arnold container
            object_position=cup_position,
            # orientation_quat=[1., 0., 0.0, 0.0],
            orientation_quat=[0.70182, 0.70182, 0.0, 0.0],  # should correspont to rotation of particles
            scale=np.array([1., 1., 1.]) * SCALE,
            object_physics_properties=Namespace(
                properties={'collision': 'convexDecomposition', 'is_rigid_body': True}),
            part_physics_properties={
                'cup_shape': Namespace(
                    properties={
                    'mass': 0.01, 'static_friction': 1.0, 'dynamic_friction': 1.0, 'restitution': 0.0, 'has_physics_material': True,
                    'collision': 'convexDecomposition', 'is_rigid_body': False, PREDICATE: cup_shape_predicate
                })
            },
            fluid_properties=Namespace(
                properties={
                    "fluid_sphere_diameter": 0.72 * SCALE,
                    "particle_system_schema_parameters": {
                        'particle_contact_offset': 0.75 * SCALE,
                        #
                        'solid_rest_offset': 0.75 * SCALE * 0.9,
                        'fluid_rest_offset': 0.45 * SCALE,
                        #
                        'contact_offset': 0.9 * SCALE,
                        'rest_offset': 0.75 * SCALE,
                        #
                        'solver_position_iterations': 10,
                        'wind': Gf.Vec3f(0.0, 0.0, 0.0),
                        'max_velocity': 40 * SCALE,
                    },
                    'particle_mass': 3e-07 * (SCALE ** 3),
                    # 'particle_color': np.array([0.2, 0.7, 0.9]),  # blue
                    'particle_color': np.array([0.03, 0.95, 0.35]),  # green
                }
            )
        )
        
        object_prim_path = find_unique_string_name(
            initial_name="/World/cup", is_unique_fn=lambda x: not is_prim_path_valid(x)
        )
        
        position = param.object_position
        rotation = param.orientation_quat
        object_prim = add_reference_to_stage(param.usd_path, object_prim_path)
        _ = XFormPrim(object_prim_path, translation= position, orientation = rotation, scale = np.array(param.scale))
        
        # particles
        from environment.fluid_utils import set_particle_system_for_cup

        particle_system_path = '/World_0/Fluid'
        particle_instance_str = "/World_0/Particles"
        
        volume_mesh_path = object_prim.GetPath().AppendPath(f"cup_volume").pathString
        set_particle_system_for_cup(
            self.stage, Gf.Vec3f(param.object_position[0], param.object_position[1], param.object_position[2]),
            volume_mesh_path, particle_system_path, particle_instance_str, param.fluid_properties,
            asset_root="/assets", enable_iso_surface=self.iso_surface,
            mug_init_rotation=90,  # in degrees for USD prop
            scale=SCALE,
        )
        
        self._wait_for_loading()
        
        from environment.physics_utils import set_physics_properties

        if param.object_physics_properties:
            set_physics_properties(self.stage, object_prim, param.object_physics_properties)
        
        if param.part_physics_properties:
            for keyword, properties in param.part_physics_properties.items():
                
                prim_list = get_all_matching_child_prims(object_prim_path, properties.properties[PREDICATE])
                for sub_prim_path in prim_list:
                    try:
                        sub_prim = get_prim_at_path(sub_prim_path)
                    except:
                        # since 2022.1.1 get_prim_at_path returns a prim instead of a path
                        sub_prim = get_prim_at_path(sub_prim_path.GetPath().pathString)
                    set_physics_properties(self.stage, sub_prim, properties)
                    # add_update_semantics(sub_prim, keyword)
        
        # ---------------------------
        cup = object_prim
        
        # set aux USD
        USD_SCENE_SCALE = 100.
        scene_prim_path = "/World/table_scene"
        usd_path = Path(__file__).parent / "assets/table_scene.usd"
        add_reference_to_stage(usd_path=usd_path.as_posix(), prim_path=scene_prim_path)
        _ = XFormPrim(scene_prim_path, scale=np.array([USD_SCENE_SCALE] * 3))
        return cup

    def _wait_for_loading(self):
        sim = SimulationContext.instance()
        sim.render()
        while is_stage_loading():
            sim.render()
    
    def get_params(self) -> dict:
        params_representation = dict()
        return params_representation

    def get_observations(self) -> dict:
        """[summary]

        Returns:
            dict: [description]
        """
        return {}

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


from pxr import Gf, PhysxSchema, UsdPhysics, UsdShade


def set_up_physics_secne(stage):
    stage_properties = Namespace(
        gravity_direction=[0, 0, -1],
        gravity_magnitude=9.81,
    )
    
    # reference : https://docs.omniverse.nvidia.com/app_isaacsim/app_isaacsim/tutorial_gym_transfer_policy.html
    physicsScenePath = "/physicsScene"
    scene = UsdPhysics.Scene.Get(stage, physicsScenePath)
    if not scene:
        scene = UsdPhysics.Scene.Define(stage, physicsScenePath)
    
    gravityDirection = stage_properties.gravity_direction
    gravityDirection = Gf.Vec3f(gravityDirection[0], gravityDirection[1],  gravityDirection[2])

    scene.CreateGravityDirectionAttr().Set(gravityDirection)

    gravityMagnitude = stage_properties.gravity_magnitude
    scene.CreateGravityMagnitudeAttr().Set(gravityMagnitude)
    
    physxSceneAPI = PhysxSchema.PhysxSceneAPI.Apply(scene.GetPrim())
    physxSceneAPI.CreateEnableCCDAttr().Set(True)
    physxSceneAPI.GetTimeStepsPerSecondAttr().Set(120)
    physxSceneAPI.CreateEnableGPUDynamicsAttr().Set(True)
    physxSceneAPI.CreateEnableEnhancedDeterminismAttr().Set(True)
    physxSceneAPI.CreateEnableStabilizationAttr().Set(True)

    physxSceneAPI.GetGpuMaxRigidContactCountAttr().Set(524288)
    physxSceneAPI.GetGpuMaxRigidPatchCountAttr().Set(81920)
    physxSceneAPI.GetGpuFoundLostPairsCapacityAttr().Set(8192)
    physxSceneAPI.GetGpuFoundLostAggregatePairsCapacityAttr().Set(262144)
    physxSceneAPI.GetGpuTotalAggregatePairsCapacityAttr().Set(8192)
    physxSceneAPI.GetGpuMaxSoftBodyContactsAttr().Set(1048576)
    physxSceneAPI.GetGpuMaxParticleContactsAttr().Set(1048576)
    # physxSceneAPI.GetGpuHeapCapacityAttr().Set(67108864)