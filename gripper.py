import numpy as np
import omni
import omni.physics.tensors as physics
from isaacsim.robot.surface_gripper._surface_gripper import Surface_Gripper, Surface_Gripper_Properties
from pxr import Gf, Sdf, UsdGeom, UsdLux, UsdPhysics
from pxr import Usd, UsdGeom, UsdPhysics, Gf, Sdf

STATIC_CONE = 0
MOUNTED_CONE = 1
MOUNTED_MESH = 2


SCALE = 1.
GRIPPER_TYPE = MOUNTED_MESH


if GRIPPER_TYPE == MOUNTED_CONE:
    GRIPPER_PRIM_PATH = "/World/RC5/GripperCone"
elif GRIPPER_TYPE == STATIC_CONE:
    GRIPPER_PRIM_PATH = "/GripperCone"
elif GRIPPER_TYPE == MOUNTED_MESH:
    GRIPPER_PRIM_PATH = "/World/RC5/GripperMesh"
else:
    raise NotImplementedError

class GripperDemo:
    def __init__(self, cone_init_pos):
        self.cone_init_pos = cone_init_pos
        self._timeline = omni.timeline.get_timeline_interface()
        self.create_surface_gripper()
    
    def create_surface_gripper(self):
        self._usd_context = omni.usd.get_context()
        self._stage = self._usd_context.get_stage()
        self._stage_id = self._usd_context.get_stage_id()
            
        self.color_closed = Gf.Vec3f(1.0, 0.2, 0.2)
        self.color_open = Gf.Vec3f(0.2, 1.0, 0.2)
        
        if GRIPPER_TYPE == MOUNTED_CONE:
            self.gripper_start_pose = physics.Transform(self.cone_init_pos, [0.70710678, -0.70710678, 0.0, 0.0])
        elif GRIPPER_TYPE == STATIC_CONE:
            self.gripper_start_pose = physics.Transform([1/2, 0, 0.301], [1, 0, 0, 0])
        elif GRIPPER_TYPE == MOUNTED_MESH:
            self.cone_init_pos[1] += 0.03
            self.gripper_start_pose = physics.Transform(self.cone_init_pos, [0.70710678, 0., 0.0, -0.70710678])
        
        if GRIPPER_TYPE in (STATIC_CONE, MOUNTED_CONE):
            self.coneGeom = self.createRigidBody(
                UsdGeom.Cone,
                GRIPPER_PRIM_PATH,
                0.100,
                [0.10 * SCALE, 0.10 * SCALE, 0.10 * SCALE],
                self.gripper_start_pose.p,
                self.gripper_start_pose.r,
                self.color_open,
            )
        elif GRIPPER_TYPE == MOUNTED_MESH:
            ASSET_FILE_NAME = "cad/end_eff_v1.usd"
            MESH_SCALE = 1.5
            from pathlib import Path
            assets_path = Path(__file__).parent / "assets"
            asset_path = (assets_path / ASSET_FILE_NAME).as_posix()
            self.coneGeom = self.createRigidBodyFromMesh(
                asset_path,
                GRIPPER_PRIM_PATH,
                0.100,
                [MESH_SCALE * SCALE] * 3,
                self.gripper_start_pose.p,
                self.gripper_start_pose.r,
                self.color_open,
            )
        
        if GRIPPER_TYPE in (MOUNTED_CONE, MOUNTED_MESH):
            from omni.physx.scripts import utils
            a = self._stage.GetPrimAtPath("/World/RC5/frame6/body6")
            b = self._stage.GetPrimAtPath(GRIPPER_PRIM_PATH)
            utils.createJoint(self._stage, "Fixed", a, b)

        # Box to be picked
        self.box_start_pose = physics.Transform([1/2, 0, 0.10], [1, 0, 0, 0])
        BOX_SCALE = 0.1
        Z_SCALE = 2.05 - 0.15 + 0.05
        self.boxGeom = self.createRigidBody(
            UsdGeom.Cube, "/Box", 0.10, [BOX_SCALE * SCALE, BOX_SCALE * SCALE, BOX_SCALE * SCALE * Z_SCALE], self.box_start_pose.p, self.box_start_pose.r, [0.2, 0.2, 1]
        )
        
        # mesh
        self.sgp = Surface_Gripper_Properties()
        self.sgp.d6JointPath = f"{GRIPPER_PRIM_PATH}/SurfaceGripper"
        self.sgp.parentPath = GRIPPER_PRIM_PATH
        self.sgp.offset = physics.Transform()
        self.sgp.offset.p.x = 0.05878
        self.sgp.offset.p.y = -0.08547 - 0.01
        self.sgp.offset.p.z = -0.0201
        self.sgp.offset.r = [0, 0, -0.7071, 0.7071]  # Rotate to point gripper in Z direction
        # while [0.7071, 0, 0, -0.7071] works for vis
        self.sgp.gripThreshold = 0.04
        self.sgp.forceLimit = 1.0e9
        self.sgp.torqueLimit = 1.0e9
        self.sgp.bendAngle = np.pi / 4
        self.sgp.stiffness = 1.0e4
        self.sgp.damping = 1.0e3
        
        self.surface_gripper = Surface_Gripper()
        self.surface_gripper.initialize(self.sgp)
        
        self.create_child_prim(GRIPPER_PRIM_PATH, "point_vis", self.sgp.offset)

    def toggle_gripper(self):
        if self._timeline.is_playing():
            if self.surface_gripper.is_closed():
                self.surface_gripper.open()
            else:
                self.surface_gripper.close()

    def createRigidBody(self, bodyType, boxActorPath, mass, scale, position, rotation, color):
        p = Gf.Vec3f(position[0], position[1], position[2])
        orientation = Gf.Quatf(rotation[0], rotation[1], rotation[2], rotation[3])
        scale = Gf.Vec3f(scale[0], scale[1], scale[2])

        bodyGeom = bodyType.Define(self._stage, boxActorPath)
        bodyPrim = self._stage.GetPrimAtPath(boxActorPath)
        bodyGeom.AddTranslateOp().Set(p)
        bodyGeom.AddOrientOp().Set(orientation)
        bodyGeom.AddScaleOp().Set(scale)
        bodyGeom.CreateDisplayColorAttr().Set([color])

        UsdPhysics.CollisionAPI.Apply(bodyPrim)
        if mass > 0:
            massAPI = UsdPhysics.MassAPI.Apply(bodyPrim)
            massAPI.CreateMassAttr(mass)
        UsdPhysics.RigidBodyAPI.Apply(bodyPrim)
        UsdPhysics.CollisionAPI(bodyPrim)
        print(bodyPrim.GetPath().pathString)
        return bodyGeom

    from pxr import Usd, UsdGeom, UsdPhysics, Gf

    def createRigidBodyFromMesh(self, meshPath, actorPath, mass, scale, position, rotation, color):
        """
        Create a rigid body in the stage from a mesh asset (USD file).
        
        Parameters:
            meshPath (str): Path to the USD mesh file (e.g. "/path/to/mesh.usd").
            actorPath (str): Prim path in the stage where the mesh should be created (e.g. "/World/MyMesh").
            mass (float): Mass of the rigid body (>0 makes it dynamic).
            scale (tuple): Scale factors (x, y, z).
            position (tuple): Translation (x, y, z).
            rotation (tuple): Quaternion (w, x, y, z).
            color (tuple): RGB values in [0,1].
        """
        # Convert inputs
        p = Gf.Vec3f(*position)
        orientation = Gf.Quatf(rotation[0], rotation[1], rotation[2], rotation[3])  # w, x, y, z
        s = Gf.Vec3f(*scale)

        # Create an Xform prim at actorPath
        xformPrim = self._stage.DefinePrim(actorPath, "Xform")

        # Reference the mesh asset onto this prim
        xformPrim.GetReferences().AddReference(meshPath)

        # Wrap as Xformable
        xform = UsdGeom.Xformable(xformPrim)

        # Safely set transforms:
        # - Use GetOrderedXformOps() to check if ops already exist
        # - If found, reuse them
        # - Otherwise, create them
        def set_or_add(opName, createFn, value):
            for op in xform.GetOrderedXformOps():
                if op.GetOpName() == opName:
                    op.Set(value)
                    return
            createFn().Set(value)

        set_or_add("xformOp:translate", xform.AddTranslateOp, p)
        set_or_add("xformOp:orient", xform.AddOrientOp, orientation)
        set_or_add("xformOp:scale", xform.AddScaleOp, s)

        # Optionally override display color (if the mesh is a Mesh prim)
        meshGeom = UsdGeom.Mesh.Get(self._stage, actorPath)
        if meshGeom and color:
            meshGeom.CreateDisplayColorAttr().Set([color])

        # Apply physics APIs
        UsdPhysics.CollisionAPI.Apply(xformPrim)
        if mass > 0:
            massAPI = UsdPhysics.MassAPI.Apply(xformPrim)
            massAPI.CreateMassAttr(mass)
        UsdPhysics.RigidBodyAPI.Apply(xformPrim)

        print(xformPrim.GetPath().pathString)
        return xformPrim

    def create_child_prim(self, parent_path: str, child_name: str, relative_transform):
        """
        Creates a child prim under the given parent path and sets its relative position and orientation.

        Args:
            parent_path (str): The Sdf path of the parent prim.
            child_name (str): The name of the child prim to create.
            relative_transform (omni.physics.Transform): Transform relative to the parent.
        Returns:
            UsdPrim: The created child prim.
        """
        stage = omni.usd.get_context().get_stage()
        if not stage:
            raise RuntimeError("USD stage not available!")

        # Build the full path for the child
        child_path = f"{parent_path}/{child_name}"

        # Create the child prim as an Xform (transformable prim)
        child_prim = stage.DefinePrim(child_path, "Xform")
        if not child_prim:
            raise RuntimeError(f"Failed to create child prim at {child_path}")

        # Set relative translation
        translation = relative_transform.p  # Should be a Gf.Vec3f or list [x, y, z]
        rotation = relative_transform.r   # Should be a Gf.Quatf or list [w, x, y, z]

        xform = UsdGeom.Xformable(child_prim)
        xform.AddTranslateOp().Set(Gf.Vec3f(*translation))
        xform.AddOrientOp().Set(Gf.Quatf(rotation[0], rotation[1], rotation[2], rotation[3]))

        return child_prim