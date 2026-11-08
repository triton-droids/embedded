"""Optional MuJoCo scene: D435 revision K visuals and a physical football.

CAD is in mm with +Y toward the lenses. This robot's toes face +Y.
The pitch axis starts 30 degrees down; both axes use simulated servos.
Masses, optical centre, field of view and mounting height are simulation assumptions.
"""
import json
import math
from pathlib import Path

import mujoco
from humanoid_sim import model_builder as mb


def assets_directory():
    local = Path(__file__).resolve().parents[1] / 'assets'
    if local.is_dir():
        return local
    from ament_index_python.packages import get_package_share_directory
    return Path(get_package_share_directory('humanoid_ball_chase')) / 'assets'


def build_scene(spec, options):
    if not options.add_head:
        raise ValueError('Ball-chase scene requires enable_head=true')
    spec.add_material(name='football_pitch', rgba=[0.12,0.38,0.10,1])
    spec.geom('floor').material = 'football_pitch'
    if spec.body(mb.TARGET_BODY) is not None:
        spec.delete(spec.body(mb.TARGET_BODY))
    head = spec.body('head')
    for name in ('head_neck', 'head_shell', 'head_lens'):
        spec.delete(spec.geom(name))
    spec.delete(spec.camera(mb.HEAD_CAMERA))
    fixed = spec.body('torso').add_body(name='d435_mount_base', pos=spec.site('top').pos)
    # Adapter to the robot's pre-existing top attachment site; assumed geometry.
    fixed.add_geom(name='d435_sim_adapter', type=mujoco.mjtGeom.mjGEOM_BOX,
                   pos=[0,0,-0.035], size=[0.025,0.022,0.035],
                   rgba=[0.15,0.18,0.2,1], mass=0.03, group=2, contype=0, conaffinity=0)
    pitch_angle = math.radians(30.0)
    pitch = head.add_body(name='d435_pitch', pos=[-0.02, 0, 0.088],
                         quat=[math.cos(pitch_angle/2), -math.sin(pitch_angle/2), 0, 0])
    pitch.add_joint(name='head_pitch_joint', type=mujoco.mjtJoint.mjJNT_HINGE,
                    axis=[-1,0,0], range=[-0.6,0.6], damping=0.05,
                    frictionloss=0.03, armature=0.001)
    spec.add_actuator(name='head_pitch_servo', target='head_pitch_joint',
                      trntype=mujoco.mjtTrn.mjTRN_JOINT,
                      gaintype=mujoco.mjtGain.mjGAIN_FIXED,
                      biastype=mujoco.mjtBias.mjBIAS_AFFINE,
                      forcelimited=mujoco.mjtLimited.mjLIMITED_TRUE,
                      forcerange=[-2.5,2.5]).gainprm[0] = 1.0
    for part in json.loads((assets_directory() / 'mount.json').read_text()):
        name = part['name']
        mesh_name = 'd435_' + name
        spec.add_mesh(name=mesh_name, file=str(assets_directory() / (name + '.stl')),
                      scale=[0.001]*3)
        group = part['group']
        body = fixed if group == 'fixed' else pitch if group == 'pitch' else head
        # CAD +Y lenses and robot +Y toes point in the same direction.
        pos = [-0.02, 0, 0] if group != 'pitch' else [0, 0, -0.088]
        mass = (0.072 if name == 'REF_D435_envelope' else
                0.060 if name.startswith('REF_DS3225') else
                0.025 if name[:2] in ('01', '02', '03') else 0.0)
        body.add_geom(name=mesh_name, type=mujoco.mjtGeom.mjGEOM_MESH,
                      meshname=mesh_name, pos=pos, quat=[1,0,0,0],
                      rgba=[*part['color'],1], mass=mass, group=2,
                      contype=0, conaffinity=0)
    pitch.add_camera(name=mb.HEAD_CAMERA, pos=[0,0.013,0],
                     quat=mb._quat_from_axes([1,0,0],[0,0,1]),
                     fovy=options.camera_fovy_deg, resolution=list(options.camera_resolution))
    spec.delete(spec.site('head_imu'))
    pitch.add_site(name='head_imu')
    ball = spec.worldbody.add_body(name='chase_ball', pos=[-0.25,1.8,0.11])
    ball.add_freejoint(name='chase_ball_free')
    ball.add_geom(name='chase_ball', type=mujoco.mjtGeom.mjGEOM_SPHERE,
                  size=[0.10,0,0], rgba=[0,0,0,0], mass=0.20,
                  friction=[0.6,0.005,0.001], condim=6, contype=1, conaffinity=1)
    for color in ('white','black'):
        name = 'football_' + color
        spec.add_mesh(name=name, file=str(assets_directory()/(name+'.stl')))
        value = 0.95 if color == 'white' else 0.025
        ball.add_geom(name=name, type=mujoco.mjtGeom.mjGEOM_MESH, meshname=name,
                      rgba=[value,value,value,1], mass=0, contype=0, conaffinity=0)
