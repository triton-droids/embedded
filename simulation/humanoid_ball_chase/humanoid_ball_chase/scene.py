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


# ---------------------------------------------------------------------------
# Harder perception scene (launch: scene:=hard). Everything added below is
# visual only (contype/conaffinity 0, no mass): physics, contacts and the
# walking task are identical to build_scene. The layout copies what the
# HSLVision images show the detectors miss: a ball in front of a row of robot
# legs, people's legs and white shoes beside it, white lines through it, goal
# posts, advertising boards and hard shadows.

def _visual(parent, name, rgba, **kw):
    return parent.add_geom(name=name, rgba=rgba, contype=0, conaffinity=0, mass=0, group=0, **kw)


def _box(parent, name, pos, half, rgba, yaw=0.0):
    quat = [math.cos(yaw/2), 0, 0, math.sin(yaw/2)]
    return _visual(parent, name, rgba, type=mujoco.mjtGeom.mjGEOM_BOX, pos=pos, size=half, quat=quat)


def _capsule(parent, name, a, b, radius, rgba):
    return _visual(parent, name, rgba, type=mujoco.mjtGeom.mjGEOM_CAPSULE, fromto=[*a, *b], size=[radius, 0, 0])


WHITE_LINE = [0.93, 0.93, 0.9, 1]


def _line(world, name, a, b, width=0.05):
    (x0, y0), (x1, y1) = a, b
    length = math.hypot(x1 - x0, y1 - y0)
    _box(world, name, [(x0+x1)/2, (y0+y1)/2, 0.0012], [length/2, width/2, 0.001], WHITE_LINE,
         yaw=math.atan2(y1 - y0, x1 - x0))


def _field_robot(world, prefix, x, y, jersey):
    """About 0.9 m humanoid facing -Y (toward the chasing robot's camera)."""
    metal, dark = [0.28, 0.28, 0.3, 1], [0.08, 0.08, 0.09, 1]
    for side, dx in (('l', -0.075), ('r', 0.075)):
        _box(world, f'{prefix}_{side}_foot', [x+dx, y-0.02, 0.016], [0.045, 0.075, 0.016], dark)
        _capsule(world, f'{prefix}_{side}_shin', [x+dx, y, 0.05], [x+dx, y, 0.28], 0.032, metal)
        _box(world, f'{prefix}_{side}_shin_plate', [x+dx, y-0.035, 0.17], [0.028, 0.006, 0.07], jersey)
        _capsule(world, f'{prefix}_{side}_thigh', [x+dx, y, 0.30], [x+dx*0.8, y, 0.48], 0.045, jersey)
        _capsule(world, f'{prefix}_{side}_arm', [x+dx*2.1, y, 0.73], [x+dx*2.3, y-0.03, 0.47], 0.03, metal)
    _box(world, f'{prefix}_pelvis', [x, y, 0.51], [0.11, 0.07, 0.05], dark)
    _box(world, f'{prefix}_torso', [x, y, 0.65], [0.13, 0.08, 0.11], jersey)
    _box(world, f'{prefix}_number', [x, y-0.0815, 0.67], [0.05, 0.001, 0.05], [0.95, 0.95, 0.95, 1])
    _box(world, f'{prefix}_head', [x, y, 0.83], [0.065, 0.065, 0.065], metal)
    _box(world, f'{prefix}_visor', [x, y-0.066, 0.84], [0.05, 0.001, 0.025], dark)


def _person(world, prefix, x, y, trousers, shoe=(0.96, 0.96, 0.96, 1)):
    for side, dx in (('l', -0.11), ('r', 0.11)):
        _box(world, f'{prefix}_{side}_shoe', [x+dx, y-0.04, 0.045], [0.05, 0.13, 0.045], list(shoe))
        _capsule(world, f'{prefix}_{side}_leg', [x+dx, y, 0.12], [x+dx*0.8, y, 0.92], 0.065, trousers)
    _box(world, f'{prefix}_torso', [x, y, 1.25], [0.2, 0.12, 0.3], [0.15, 0.3, 0.55, 1])


def build_scene_hard(spec, options):
    build_scene(spec, options)
    world = spec.worldbody
    # Mown turf: two close greens in long stripes plus speckle, instead of one flat colour.
    spec.add_texture(name='turf_hard', type=mujoco.mjtTexture.mjTEXTURE_2D,
                     builtin=mujoco.mjtBuiltin.mjBUILTIN_CHECKER,
                     rgb1=[0.12, 0.37, 0.11], rgb2=[0.105, 0.33, 0.095],
                     mark=mujoco.mjtMark.mjMARK_RANDOM, random=0.25,
                     markrgb=[0.08, 0.27, 0.07], width=512, height=512)
    pitch = spec.add_material(name='football_pitch_hard', texrepeat=[2, 16], reflectance=0.0)
    pitch.textures[mujoco.mjtTextureRole.mjTEXROLE_RGB] = 'turf_hard'
    spec.geom('floor').material = 'football_pitch_hard'
    # Field lines through and around the ball (ball rests at -0.25, 1.8).
    _line(world, 'line_goal', (-3.0, 2.0), (3.0, 2.0))
    _line(world, 'line_box_l', (-0.6, 2.0), (-0.6, 3.2))
    _line(world, 'line_box_r', (1.4, 2.0), (1.4, 3.2))
    _line(world, 'line_side', (0.9, -0.5), (0.9, 2.0))
    _line(world, 'line_mark_a', (0.25, 2.6), (0.35, 2.6), width=0.04)
    _line(world, 'line_mark_b', (0.3, 2.55), (0.3, 2.65), width=0.04)
    for i in range(10):  # centre-circle arc
        a0, a1 = math.pi * (0.55 + 0.09 * i), math.pi * (0.55 + 0.09 * (i + 1))
        _line(world, f'line_arc_{i}', (-1.6 + 0.75*math.cos(a0), 0.9 + 0.75*math.sin(a0)),
              (-1.6 + 0.75*math.cos(a1), 0.9 + 0.75*math.sin(a1)))
    # A row of robots right behind the ball, as in the ice-ribbon images.
    red, blue = [0.75, 0.08, 0.1, 1], [0.1, 0.2, 0.7, 1]
    for i, (x, y, jersey) in enumerate([(-0.58, 2.18, red), (-0.24, 2.14, red), (0.1, 2.2, red),
                                        (1.2, 3.0, blue), (-1.5, 3.4, blue)]):
        _field_robot(world, f'distractor_robot_{i}', x, y, jersey)
    # People: one standing next to the ball with white shoes, two further away.
    _person(world, 'person_0', 0.32, 1.98, [0.12, 0.15, 0.25, 1])
    _person(world, 'person_1', -1.25, 2.6, [0.05, 0.05, 0.06, 1])
    _person(world, 'person_2', 1.9, 3.6, [0.45, 0.4, 0.33, 1], shoe=(0.1, 0.1, 0.1, 1))
    # Goal and boards behind it.
    for x in (-1.3, 1.3):
        _capsule(world, f'goal_post_{x:+.1f}', [x, 4.2, 0], [x, 4.2, 1.1], 0.05, [0.97, 0.97, 0.97, 1])
    _capsule(world, 'goal_bar', [-1.3, 4.2, 1.1], [1.3, 4.2, 1.1], 0.05, [0.97, 0.97, 0.97, 1])
    colors = [[0.1, 0.2, 0.6, 1], [0.95, 0.95, 0.95, 1], [0.85, 0.15, 0.1, 1], [0.95, 0.8, 0.1, 1]]
    for i in range(12):
        _box(world, f'board_back_{i}', [-3.3 + 0.6*i, 4.9, 0.25], [0.29, 0.02, 0.25], colors[i % 4])
        _box(world, f'board_back_stripe_{i}', [-3.3 + 0.6*i, 4.875, 0.3], [0.2, 0.003, 0.04], colors[(i+1) % 4])
    for i in range(8):
        for x in (-3.0, 3.0):
            _box(world, f'board_side_{x:+.0f}_{i}', [x, -0.5 + 0.7*i, 0.25], [0.02, 0.34, 0.25], colors[(i+2) % 4])
    # Low, hard sunlight: long shadows of legs across the ball and lines.
    world.add_light(name='low_sun', pos=[-2, -1, 3], dir=[0.45, 0.55, -0.7],
                    type=mujoco.mjtLightType.mjLIGHT_DIRECTIONAL, castshadow=True,
                    diffuse=[0.75, 0.72, 0.65], specular=[0.1, 0.1, 0.1])
