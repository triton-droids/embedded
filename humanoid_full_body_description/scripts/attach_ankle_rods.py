#!/usr/bin/env python3
"""Move the ankle pushrods from the shank link to the foot link.

The CAD export put each ankle pushrod (left link_005_001, right link_009_046) in the shank link, so it stayed put
while the foot rotated and visibly came off the foot at large ankle angles. Here each rod becomes rigid with the
foot: its <visual>/<collision> move to <side>_foot_link (origin re-expressed in the foot frame at q = 0), and its
mass moves with it. The rod's mass is its CAD volume times the shank's implied density (shank mass / shank part
volume); its inertia comes from the mesh volume integrals. Shank and foot inertials are updated with the
parallel-axis theorem, in both cad_source/full_body_parts.urdf and urdf/full_body.urdf.xacro.

The crank and the rod-end hardware at the crank stay on the shank. A rod rigid with the foot pivots about the
ankle axis, so at large ankle angles its upper end moves away from the crank instead (the real four-bar is a
closed loop, which a URDF tree cannot express).

Afterwards rebuild the per-link meshes: scripts/simplify_meshes.py --links left_shank_link left_foot_link
right_shank_link right_foot_link, then scripts/make_sim_mjcf.py.
"""
import re
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import trimesh
from scipy.spatial.transform import Rotation as R

PKG = Path(__file__).resolve().parent.parent
RODS = {'left': 'link_005_001_1_01', 'right': 'link_009_046_1_01'}
KEYS = ('ixx', 'ixy', 'ixz', 'iyy', 'iyz', 'izz')


def pose(o):
    T = np.eye(4)
    if o is not None:
        T[:3, :3] = R.from_euler('xyz', [float(v) for v in o.get('rpy', '0 0 0').split()]).as_matrix()
        T[:3, 3] = [float(v) for v in o.get('xyz', '0 0 0').split()]
    return T


def fmt(v):
    return ' '.join(f'{x:.12g}' for x in v)


def mesh_of(e):
    uri = e.find('geometry/mesh').get('filename').removeprefix('package://humanoid_full_body_description/')
    return trimesh.load(PKG / uri, force='mesh')


def inertial_of(link):
    ine = link.find('inertial')
    m = float(ine.find('mass').get('value'))
    c = np.array([float(v) for v in ine.find('origin').get('xyz').split()])
    i = [float(ine.find('inertia').get(k)) for k in KEYS]
    I = np.array([[i[0], i[1], i[2]], [i[1], i[3], i[4]], [i[2], i[4], i[5]]])
    return m, c, I


def set_inertial(link, m, c, I):
    ine = link.find('inertial')
    ine.find('mass').set('value', f'{m:.12g}')
    ine.find('origin').set('xyz', fmt(c))
    for k, v in zip(KEYS, (I[0, 0], I[0, 1], I[0, 2], I[1, 1], I[1, 2], I[2, 2])):
        ine.find('inertia').set(k, f'{v:.12g}')


def shift(m, c, I_c):
    """Inertia about the link origin from inertia about the COM."""
    return I_c + m * (np.dot(c, c) * np.eye(3) - np.outer(c, c))


def main():
    path = PKG / 'cad_source/full_body_parts.urdf'
    raw = path.read_bytes().decode()
    root = ET.fromstring(raw)
    links = {l.get('name'): l for l in root.findall('link')}
    joints = {j.get('name'): j for j in root.findall('joint')}
    new_inertials = {}
    for side, rod in RODS.items():
        shank, foot = links[f'{side}_shank_link'], links[f'{side}_foot_link']
        elems = [e for e in list(shank) if e.tag in ('visual', 'collision')
                 and e.find('geometry/mesh').get('filename').endswith(f'/{rod}.stl')]
        if not elems:
            sys.exit(f'{rod} is not in {side}_shank_link (already moved?)')
        T_sf = pose(joints[f'{side}_ankle_joint'].find('origin'))  # foot frame in the shank frame, q = 0
        # rod mass properties in the shank frame
        rod_mesh = mesh_of(elems[0]); rod_mesh.apply_transform(pose(elems[0].find('origin')))
        shank_volume = 0.0
        for v in shank.findall('collision'):
            shank_volume += abs(mesh_of(v).volume)
        ms, cs, Is = inertial_of(shank)
        rho = ms / shank_volume
        mr = abs(rod_mesh.volume) * rho
        cr = rod_mesh.center_mass
        Ir = rod_mesh.moment_inertia * (mr / rod_mesh.mass)  # about cr, shank axes
        # shank minus rod
        ms2 = ms - mr
        cs2 = (ms * cs - mr * cr) / ms2
        Io = shift(ms, cs, Is) - shift(mr, cr, Ir)
        Is2 = Io - ms2 * (np.dot(cs2, cs2) * np.eye(3) - np.outer(cs2, cs2))
        # rod into the foot frame, foot plus rod
        Rfs = T_sf[:3, :3].T
        cr_f = Rfs @ (cr - T_sf[:3, 3]); Ir_f = Rfs @ Ir @ Rfs.T
        mf, cf, If = inertial_of(foot)
        mf2 = mf + mr
        cf2 = (mf * cf + mr * cr_f) / mf2
        Io = shift(mf, cf, If) + shift(mr, cr_f, Ir_f)
        If2 = Io - mf2 * (np.dot(cf2, cf2) * np.eye(3) - np.outer(cf2, cf2))
        set_inertial(shank, ms2, cs2, Is2); set_inertial(foot, mf2, cf2, If2)
        new_inertials[shank.get('name')] = (ms2, cs2, Is2); new_inertials[foot.get('name')] = (mf2, cf2, If2)
        for e in elems:
            T = np.linalg.inv(T_sf) @ pose(e.find('origin'))
            o = e.find('origin')
            if o is None:
                o = ET.Element('origin'); e.insert(0, o)
            o.set('xyz', fmt(T[:3, 3])); o.set('rpy', fmt(R.from_matrix(T[:3, :3]).as_euler('xyz')))
            shank.remove(e); foot.insert(list(foot).index(foot.find('inertial')), e) if foot.find('inertial') is not None else foot.append(e)
        print(f'{side}: rod {mr * 1000:.1f} g (density {rho / 1000:.2f} g/cm3); shank {ms:.4f} -> {ms2:.4f} kg, '
              f'foot {mf:.4f} -> {mf2:.4f} kg')
    det = ET.tostring(root, encoding='unicode')
    nl = '\r\n' if '\r\n' in raw else '\n'
    if nl == '\r\n':
        det = det.replace('\r\n', '\n').replace('\n', '\r\n')
    path.write_text('<?xml version="1.0" ?>' + nl + det, newline='')

    xpath = PKG / 'urdf/full_body.urdf.xacro'
    x = xpath.read_text()
    for name, (m, c, I) in new_inertials.items():
        pat = re.compile(rf'(<xacro:cad_link name="{name}" material="\w+"\s+)mass="[^"]+" com="[^"]+"(\s+)inertia="[^"]+"')
        i6 = (I[0, 0], I[0, 1], I[0, 2], I[1, 1], I[1, 2], I[2, 2])
        x, n = pat.subn(lambda mm: f'{mm.group(1)}mass="{m:.6g}" com="{" ".join(f"{v:.6g}" for v in c)}"'
                        f'{mm.group(2)}inertia="{" ".join(f"{v:.6g}" for v in i6)}"', x)
        assert n == 1, name
    xpath.write_text(x)


if __name__ == '__main__':
    main()
