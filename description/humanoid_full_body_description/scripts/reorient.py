"""Turn the full-body model to face +y and re-derive joint signs and limits from their motion.

Input: the xacro and detailed URDF of commit 77cf784c (original CAD feet, before the 180 deg foot turn and the
policy zero match). The CAD robot faces -y (its toes point -y, the hip flexes far towards -y). We rotate the torso
link frame 180 deg about z (torso geometry, inertial and its three child joints), so the robot faces +y like the
controllers expect and left_* ends up on -x (the robot's left). Then every actuated joint's axis is negated if its
positive direction does not match the convention below (checked by forward kinematics), and limits are set.

Conventions (robot frame: +y forward, -x left, +z up):
  hip1 + flexes (foot forward); hip2: left - / right + abducts; thigh + rotates the toe outward;
  knee - flexes (foot back); ankle + lifts the toe; shoulder_pitch + swings the arm forward;
  shoulder_roll + abducts; elbow + flexes (hand forward).
"""
import re, subprocess, sys, tempfile, numpy as np
import xml.etree.ElementTree as ET
from scipy.spatial.transform import Rotation as R
import pinocchio as pin

D = sys.argv[1]  # work dir with base.xacro, base_detailed.urdf
PI = np.pi
RZ = R.from_euler('z', PI)
LIMITS = {  # (lower, upper) rad; hip1 lower is the extension stop of the current hardware (see README)
    'left_hip1_joint': (-0.305, 2.094), 'right_hip1_joint': (-0.262, 2.094),
    'left_hip2_joint': (-1.57, 0.436332), 'right_hip2_joint': (-0.436332, 1.57),
    'left_thigh_joint': (-0.785398, 0.785398), 'right_thigh_joint': (-0.785398, 0.785398),
    'left_knee_joint': (-2.0944, 0.0), 'right_knee_joint': (-2.0944, 0.0),
    'left_ankle_joint': (-2.094, 1.686), 'right_ankle_joint': (-2.094, 1.686),  # measured: toe down 120, toe up 96.6 deg
    'left_elbow_joint': (0.0, 2.269), 'right_elbow_joint': (0.0, 2.269),
}


def fmt(v):
    return ' '.join(f'{x:.12g}' for x in v)


def rot_joint_origin(xyz, rpy):
    xyz = RZ.apply(np.array(xyz, float))
    rpy = (RZ * R.from_euler('xyz', rpy)).as_euler('xyz')
    return xyz, rpy


def rot_inertia(com, I6):
    ixx, ixy, ixz, iyy, iyz, izz = I6
    M = RZ.as_matrix(); I = np.array([[ixx, ixy, ixz], [ixy, iyy, iyz], [ixz, iyz, izz]])
    J = M @ I @ M.T
    return RZ.apply(np.array(com, float)), [J[0, 0], J[0, 1], J[0, 2], J[1, 1], J[1, 2], J[2, 2]]


# ---------------------------------------------------------------- xacro (text edits, keeps the layout)
x = open(f'{D}/base.xacro').read()
m = re.search(r'<xacro:cad_link name="torso_link" material="(\w+)"\s+mass="([^"]+)" com="([^"]+)"\s+inertia="([^"]+)"/>', x)
com, I6 = rot_inertia([float(v) for v in m.group(3).split()], [float(v) for v in m.group(4).split()])
torso = f'''<!-- The CAD robot faces -y; this link frame is the CAD torso frame turned 180 deg about z, so the robot
       faces +y (left_* on -x). Mesh, inertial and the three child joints carry the turn. -->
  <link name="torso_link">
    <inertial>
      <origin xyz="{fmt(com)}" rpy="0 0 0"/>
      <mass value="{m.group(2)}"/>
      <inertia ixx="{I6[0]:.6g}" ixy="{I6[1]:.6g}" ixz="{I6[2]:.6g}" iyy="{I6[3]:.6g}" iyz="{I6[4]:.6g}" izz="{I6[5]:.6g}"/>
    </inertial>
    <visual>
      <origin xyz="0 0 0" rpy="0 0 ${{pi}}"/>
      <geometry><mesh filename="${{mesh_dir}}/visual/torso_link.stl"/></geometry>
      <material name="{m.group(1)}"/>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="0 0 ${{pi}}"/>
      <geometry><mesh filename="${{mesh_dir}}/collision/torso_link.stl"/></geometry>
    </collision>
  </link>'''
x = x[:m.start()] + torso + x[m.end():]


def fix_parent_torso(text):
    def repl(mm):
        s = mm.group(0)
        if 'parent="torso_link"' not in s:
            return s
        xyz = [float(v) for v in re.search(r'xyz="([^"]+)"', s).group(1).split()]
        rpy = [float(v) for v in re.search(r'rpy="([^"]+)"', s).group(1).split()]
        nx, nr = rot_joint_origin(xyz, rpy)
        s = re.sub(r'xyz="[^"]+"', f'xyz="{fmt(nx)}"', s, count=1)
        return re.sub(r'rpy="[^"]+"', f'rpy="{fmt(nr)}"', s, count=1)
    return re.sub(r'<xacro:(?:fixed|continuous|revolute)_joint name="[^"]+"[^/]*/>', repl, text)


x = fix_parent_torso(x)

# ---------------------------------------------------------------- detailed URDF (XML)
raw = open(f'{D}/base_detailed.urdf', 'rb').read().decode()
root = ET.fromstring(raw)
for l in root.findall('link'):
    if l.get('name') != 'torso_link':
        continue
    for tag in ('visual', 'collision'):
        for e in l.findall(tag):
            o = e.find('origin')
            if o is None:
                o = ET.Element('origin', xyz='0 0 0', rpy='0 0 0'); e.insert(0, o)
            xyz, rpy = rot_joint_origin([float(v) for v in o.get('xyz', '0 0 0').split()],
                                        [float(v) for v in o.get('rpy', '0 0 0').split()])
            o.set('xyz', fmt(xyz)); o.set('rpy', fmt(rpy))
    ine = l.find('inertial'); o = ine.find('origin'); I = ine.find('inertia')
    c, I6 = rot_inertia([float(v) for v in o.get('xyz').split()], [float(I.get(k)) for k in ('ixx', 'ixy', 'ixz', 'iyy', 'iyz', 'izz')])
    o.set('xyz', fmt(c))
    for k, v in zip(('ixx', 'ixy', 'ixz', 'iyy', 'iyz', 'izz'), I6):
        I.set(k, f'{v:.12g}')
for j in root.findall('joint'):
    if j.find('parent').get('link') == 'torso_link':
        o = j.find('origin')
        xyz, rpy = rot_joint_origin([float(v) for v in o.get('xyz').split()], [float(v) for v in o.get('rpy').split()])
        o.set('xyz', fmt(xyz)); o.set('rpy', fmt(rpy))


# ---------------------------------------------------------------- signs by forward kinematics
def expand(xacro_text):
    with tempfile.NamedTemporaryFile('w', suffix='.xacro', delete=False) as f:
        f.write(xacro_text)
    return subprocess.run(['xacro', f.name], capture_output=True, text=True, check=True).stdout


def model_of(urdf_text):
    return pin.buildModelFromXML(urdf_text)


def point(model, data, q, frame, local):
    pin.framesForwardKinematics(model, data, q)
    return data.oMf[model.getFrameId(frame)].act(local)


def check(urdf_text):
    """For each convention, True if the joint moves as intended."""
    model = model_of(urdf_text); data = model.createData(); q0 = pin.neutral(model)
    res = {}
    def qa(name, a):
        jid = model.getJointId(name); i = model.idx_qs[jid]; q = q0.copy()
        if model.nqs[jid] == 2:  # continuous joint: (cos, sin)
            q[i], q[i + 1] = np.cos(a), np.sin(a)
        else:
            q[i] = a
        return q
    for side, out in (('left', -1.0), ('right', 1.0)):
        foot = f'{side}_foot_link'
        pin.framesForwardKinematics(model, data, q0)
        F = data.oMf[model.getFrameId(foot)]
        toe_world = F.translation + np.array([0, 0.10, 0]); toe = F.actInv(toe_world)  # point 10 cm ahead
        heel_local = F.actInv(F.translation)
        d = lambda j, a, loc=heel_local, fr=foot: point(model, data, qa(j, a), fr, loc) - point(model, data, q0, fr, loc)
        res[f'{side}_hip1_joint'] = d(f'{side}_hip1_joint', 0.3)[1] > 0
        res[f'{side}_knee_joint'] = d(f'{side}_knee_joint', -0.3)[1] < 0
        res[f'{side}_hip2_joint'] = d(f'{side}_hip2_joint', 0.3 * out)[0] * out > 0
        res[f'{side}_thigh_joint'] = d(f'{side}_thigh_joint', 0.3, toe)[0] * out > 0
        res[f'{side}_ankle_joint'] = d(f'{side}_ankle_joint', 0.3, toe)[2] > 0
        hand = f'{side}_link_004'
        pin.framesForwardKinematics(model, data, q0)
        H = data.oMf[model.getFrameId(hand)]
        tip = H.actInv(H.translation + np.array([0, 0, -0.15]))  # 15 cm below the forearm frame
        dh = lambda j, a: point(model, data, qa(j, a), hand, tip) - point(model, data, q0, hand, tip)
        res[f'{side}_shoulder_pitch_joint'] = dh(f'{side}_shoulder_pitch_joint', 0.3)[1] > 0
        res[f'{side}_shoulder_roll_joint'] = dh(f'{side}_shoulder_roll_joint', 0.3)[0] * out > 0
        res[f'{side}_elbow_joint'] = dh(f'{side}_elbow_joint', 0.3)[1] > 0
    return res


def flip_axis_xacro(text, joint):
    def repl(mm):
        s = mm.group(0)
        if f'name="{joint}"' not in s:
            return s
        ax = [-float(v) for v in re.search(r'axis="([^"]+)"', s).group(1).split()]
        return re.sub(r'axis="[^"]+"', f'axis="{fmt(ax)}"', s, count=1)
    return re.sub(r'<xacro:(?:continuous|revolute)_joint name="[^"]+"[^/]*/>', repl, text)


res = check(expand(x))
flips = [j for j, ok in res.items() if not ok]
for j in flips:
    x = flip_axis_xacro(x, j)
    jj = next(e for e in root.findall('joint') if e.get('name') == j)
    a = jj.find('axis'); a.set('xyz', fmt([-float(v) for v in a.get('xyz').split()]))
after = check(expand(x))
det_check = check(ET.tostring(root, encoding='unicode'))
assert all(after.values()) and all(det_check.values()), (after, det_check)


# ---------------------------------------------------------------- limits
def set_limit_xacro(text, joint, lo, hi):
    def repl(mm):
        s = mm.group(0)
        if f'name="{joint}"' not in s:
            return s
        s = s.replace('<xacro:continuous_joint', '<xacro:revolute_joint')
        if 'lower=' in s:
            s = re.sub(r'lower="[^"]+"', f'lower="{lo}"', s); s = re.sub(r'upper="[^"]+"', f'upper="{hi}"', s)
        else:
            s = s.replace('/>', f' lower="{lo}" upper="{hi}"/>')
        return s
    return re.sub(r'<xacro:(?:continuous|revolute)_joint name="[^"]+"[^/]*/>', repl, text)


for j, (lo, hi) in LIMITS.items():
    x = set_limit_xacro(x, j, lo, hi)
    jj = next(e for e in root.findall('joint') if e.get('name') == j)
    jj.set('type', 'revolute')
    lim = jj.find('limit')
    if lim is None:
        lim = ET.SubElement(jj, 'limit', effort='0', velocity='0')
    lim.set('lower', str(lo)); lim.set('upper', str(hi))

open(f'{D}/full_body.urdf.xacro', 'w').write(x)
det = ET.tostring(root, encoding='unicode')
if '\r\n' in raw:
    det = det.replace('\n', '\r\n')
open(f'{D}/full_body_parts.urdf', 'w', newline='').write('<?xml version="1.0" ?>' + ('\r\n' if '\r\n' in raw else '\n') + det)
print('flipped axes:', flips)
print('xacro conventions:', all(after.values()), ' detailed conventions:', all(det_check.values()))
