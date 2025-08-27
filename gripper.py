import numpy as np
import omni
import omni.physics.tensors as physics
from isaacsim.robot.surface_gripper._surface_gripper import Surface_Gripper, Surface_Gripper_Properties
from pxr import Gf, Sdf, UsdGeom, UsdLux, UsdPhysics


SCALE = 1.
MOUNT_GRIPPER = True

if MOUNT_GRIPPER:
    GRIPPER_PRIM_PATH = "/World/RC5/GripperCone"
else:
    GRIPPER_PRIM_PATH = "/GripperCone"

class GripperDemo:
    def __init__(self, cone_init_pos):
        self.cone_init_pos = cone_init_pos
        self._timeline = omni.timeline.get_timeline_interface()
        self.create_surface_gripper()
    
    def create_surface_gripper(self):
        self._usd_context = omni.usd.get_context()
        self._stage = self._usd_context.get_stage()
        self._stage_id = self._usd_context.get_stage_id()
            
        # Colors to represent when gripper is open or closed
        self.color_closed = Gf.Vec3f(1.0, 0.2, 0.2)
        self.color_open = Gf.Vec3f(0.2, 1.0, 0.2)
        
        # Cone that will represent the gripper
        if MOUNT_GRIPPER:
            # self.gripper_start_pose = physics.Transform(self.cone_init_pos, [0, -1, 0, 0])
            self.gripper_start_pose = physics.Transform(self.cone_init_pos, [0.70710678, -0.70710678, 0.0, 0.0])
        else:
            self.gripper_start_pose = physics.Transform([1/2, 0, 0.301], [1, 0, 0, 0])
        
        self.coneGeom = self.createRigidBody(
            UsdGeom.Cone,
            GRIPPER_PRIM_PATH,
            0.100,
            [0.10 * SCALE, 0.10 * SCALE, 0.10 * SCALE],
            self.gripper_start_pose.p,
            self.gripper_start_pose.r,
            self.color_open,
        )
        
        if MOUNT_GRIPPER:
            # import omni.physx.scripts.utils as physx_utils
            from omni.physx.scripts import utils
            a = self._stage.GetPrimAtPath("/World/RC5/frame6/body6")
            b = self._stage.GetPrimAtPath(GRIPPER_PRIM_PATH)
            joint_prim = utils.createJoint(self._stage, "Fixed", a, b)

        # Box to be picked
        self.box_start_pose = physics.Transform([1/3, 0, 0.10], [1, 0, 0, 0])
        self.boxGeom = self.createRigidBody(
            UsdGeom.Cube, "/Box", 0.10, [0.1 * SCALE, 0.1 * SCALE, 0.1 * SCALE], self.box_start_pose.p, self.box_start_pose.r, [0.2, 0.2, 1]
        )
        
        # Reordering the quaternion to follow DC convention for later use.
        self.gripper_start_pose = physics.Transform([0, 0, 0.301], [0, 0, 0, 1])
        self.box_start_pose = physics.Transform([0, 0, 0.10], [0, 0, 0, 1])

        # Gripper properties
        self.sgp = Surface_Gripper_Properties()
        self.sgp.d6JointPath = f"{GRIPPER_PRIM_PATH}/SurfaceGripper"
        self.sgp.parentPath = GRIPPER_PRIM_PATH
        self.sgp.offset = physics.Transform()
        self.sgp.offset.p.x = 0
        self.sgp.offset.p.z = -0.1001
        self.sgp.offset.r = [0, 0.7071, 0, 0.7071]  # Rotate to point gripper in Z direction
        self.sgp.gripThreshold = 0.02
        self.sgp.forceLimit = 1.0e2
        self.sgp.torqueLimit = 1.0e3
        self.sgp.bendAngle = np.pi / 4
        self.sgp.stiffness = 1.0e4
        self.sgp.damping = 1.0e3

        self.surface_gripper = Surface_Gripper()
        self.surface_gripper.initialize(self.sgp)

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
