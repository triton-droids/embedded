#!/usr/bin/env python3
"""Give each ankle four-bar its own links: a crank on the ankle motor and the pushrod hung on the crank.

The CAD export put the crank and the pushrod in the shank link, so they stayed put while the foot rotated and the
rod came off the foot. The real linkage is a closed loop (motor axis, crank, rod, heel lever, ankle axis), which a
URDF tree cannot express. The trick here: two extra links follow the ankle angle.

  <side>_ankle_crank_link   child of the shank, revolute about the motor axis: crank, crank hub and pin hardware
  <side>_ankle_rod_link     child of the crank, revolute about the crank pin: the pushrod
  <side>_ankle_rod_end      fixed frame at the rod's lower eye (on the heel lever pin when the loop is closed)

Both joints are URDF <mimic> joints of <side>_ankle_joint (robot_state_publisher, MoveIt and RViz move them),
with the best linear fit over the driven ankle range. That fit is within about 1.6 deg; scripts/make_sim_mjcf.py
closes the loop exactly in MuJoCo instead (equality connect of the rod end to the foot).

Pin positions are measured from the CAD parts (axes of the crank hub, crank pin and rod eyes, from the cylinder
faces), so the loop closes at q = 0. Masses: CAD volume times the shank's implied density; shank inertial reduced
by the same amount (parallel-axis theorem). Edits cad_source/full_body_parts.urdf and urdf/full_body.urdf.xacro;
run once on the export (it refuses to run twice). Afterwards:
  scripts/simplify_meshes.py --links <side>_shank_link <side>_ankle_crank_link <side>_ankle_rod_link (both sides)
  scripts/make_sim_mjcf.py

Prints the exact relation (four-bar solved with the CAD pin positions) of ankle angle, crank (= motor) angle and
rod angle, and compares the motor angle with motor_control_hybrid's ankle_mapping.
"""
import math
import re
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import trimesh

PKG = Path(__file__).resolve().parent.parent
PARTS = {  # CAD part names in <side>_shank_link
    'left': dict(rod='link_005_001_1_01', crank='link_005_009_1_01', motor='link_005_005_1_01'),
    'right': dict(rod='link_009_046_1_01', crank='link_009_030_1_01', motor='link_009_012_1_01'),
}
DRIVE_DEG = (-65.0, 75.0)  # ankle range the four-bar drives (ankle_mapping); the mimic fit covers this range
KEYS = ('ixx', 'ixy', 'ixz', 'iyy', 'iyz', 'izz')


# ------------------------------------------------------------------ small geometry helpers
def euler(r):
    cx, cy, cz = np.cos(r); sx, sy, sz = np.sin(r)
    return (np.array([[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]]) @ np.array([[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]])
            @ np.array([[1, 0, 0], [0, cx, -sx], [0, sx, cx]]))


def pose(o):
    T = np.eye(4)
    if o is not None:
        T[:3, :3] = euler([float(v) for v in o.get('rpy', '0 0 0').split()])
        T[:3, 3] = [float(v) for v in o.get('xyz', '0 0 0').split()]
    return T


def rotation(ax, t):
    K = np.array([[0, -ax[2], ax[1]], [ax[2], 0, -ax[0]], [-ax[1], ax[0], 0]])
    return np.eye(3) + math.sin(t) * K + (1 - math.cos(t)) * K @ K


def signed_angle(ax, a, b):
    a = a - (a @ ax) * ax; b = b - (b @ ax) * ax
    return math.atan2(np.cross(a, b) @ ax, a @ b)


def radial(p, q, ax):
    d = p - q
    return float(np.linalg.norm(d - (d @ ax) * ax))


def hole_axis(mesh, ax, seed, radius):
    """Point on the axis of the cylindrical faces (normal perpendicular to ax) within radius of seed: the
    weighted least-squares intersection of their normal lines."""
    n, p, w = mesh.face_normals, mesh.triangles_center, mesh.area_faces
    sel = (np.abs(n @ ax) < 0.05) & (np.array([radial(x, seed, ax) for x in p]) < radius)
    P = np.eye(3) - np.outer(ax, ax)
    A = 1e-9 * np.outer(ax, ax); b = 1e-9 * np.outer(ax, ax) @ seed
    for pi, ni, wi in zip(p[sel], n[sel], w[sel]):
        Q = P - np.outer(ni, ni); A += wi * Q; b += wi * Q @ pi
    return np.linalg.solve(A, b)


def mass_properties(meshes, rho):
    """mass, COM and inertia about the COM of meshes (link frame) at density rho."""
    m = trimesh.util.concatenate(meshes)
    m.density = rho
    return m.mass, m.center_mass, m.moment_inertia


def shift(m, c, I_c):  # inertia about the frame origin from inertia about the COM
    return I_c + m * (np.dot(c, c) * np.eye(3) - np.outer(c, c))


def unshift(m, c, I_o):
    return I_o - m * (np.dot(c, c) * np.eye(3) - np.outer(c, c))


def fmt(v, p=12):
    return ' '.join(f'{x:.{p}g}' for x in v)


# ------------------------------------------------------------------ four-bar
class FourBar:
    """Planar four-bar in the shank frame: ankle axis (A), motor axis (M), crank pin (C), heel lever pin (F)."""

    def __init__(self, A, ax, M, C, F):
        self.A, self.ax, self.M, self.C0, self.F0 = A, ax, M, C, F
        self.crank = radial(C, M, ax); self.rod = radial(C, F, ax)

    def solve(self, ankle, crank_guess=0.0):
        """Crank angle (about ax, from its q = 0 position) and rod angle relative to the crank for an ankle angle.
        The branch nearest crank_guess."""
        ax, M = self.ax, self.M
        F = self.A + rotation(ax, ankle) @ (self.F0 - self.A)
        d = F - M; d = d - (d @ ax) * ax; D = np.linalg.norm(d); e = d / D; f = np.cross(ax, e)
        x = (D * D + self.crank ** 2 - self.rod ** 2) / (2 * D); h2 = self.crank ** 2 - x * x
        if h2 < 0:
            return None
        best = None
        for s in (1, -1):
            C = M + x * e + s * math.sqrt(h2) * f
            th = crank_guess + math.remainder(signed_angle(ax, self.C0 - M, C - M) - crank_guess, 2 * math.pi)
            if best is None or abs(th - crank_guess) < abs(best[0] - crank_guess):
                best = (th, C)
        th, C = best
        psi = signed_angle(ax, self.C0 - self.F0, C - F)
        return th, psi - th

    def sweep(self, ankles):
        """(crank, rod) for each ankle angle, continued from q = 0 outwards so the branch stays the same."""
        ankles = np.asarray(ankles, float)
        out = np.full((len(ankles), 2), np.nan)
        i0 = int(np.argmin(np.abs(ankles)))
        for order in (range(i0, len(ankles)), range(i0, -1, -1)):
            th = 0.0
            for i in order:
                r = self.solve(ankles[i], th)
                if r is not None:
                    out[i] = r; th = r[0]
        return out


# ------------------------------------------------------------------ main
def main():
    path = PKG / 'cad_source/full_body_parts.urdf'
    raw = path.read_bytes().decode()
    root = ET.fromstring(raw)
    if any(l.get('name').endswith('_ankle_crank_link') for l in root.findall('link')):
        sys.exit('ankle linkage links already present')
    links = {l.get('name'): l for l in root.findall('link')}
    joints = {j.get('name'): j for j in root.findall('joint')}
    xpath = PKG / 'urdf/full_body.urdf.xacro'
    x = xpath.read_text()
    report = []

    def mesh_of(e):
        uri = e.find('geometry/mesh').get('filename').removeprefix('package://humanoid_full_body_description/')
        m = trimesh.load(PKG / uri, force='mesh'); m.apply_transform(pose(e.find('origin')))
        return m

    for side, names in PARTS.items():
        shank_name = f'{side}_shank_link'
        shank = links[shank_name]
        ankle = joints[f'{side}_ankle_joint']
        T_sf = pose(ankle.find('origin'))
        ax = T_sf[:3, :3] @ np.array([float(v) for v in ankle.find('axis').get('xyz').split()])
        ax /= np.linalg.norm(ax)
        A = T_sf[:3, 3]
        parts = {}
        for e in shank.findall('collision'):
            parts[e.find('geometry/mesh').get('filename').rsplit('/', 1)[1][:-4]] = mesh_of(e)
        rod, crank, motor = parts[names['rod']], parts[names['crank']], parts[names['motor']]
        # pins: rod eyes (both ends), crank hub on the motor axis
        c = rod.vertices.mean(0); u = np.linalg.svd(rod.vertices - c)[2][0]; t = (rod.vertices - c) @ u
        ends = [hole_axis(rod, ax, rod.vertices[t.argmax()] - 0.008 * u, 0.015),
                hole_axis(rod, ax, rod.vertices[t.argmin()] + 0.008 * u, 0.015)]
        C, F = sorted(ends, key=lambda p: -radial(p, A, ax))  # upper eye (crank pin) is far from the ankle
        M = hole_axis(crank, ax, motor.center_mass, 0.012)
        M = M + ((C - M) @ ax) * ax  # hub point in the rod's mid-plane
        bar = FourBar(A, ax, M, C, F)
        # parts that move: crank group = crank, its hub parts and the crank pin hardware
        crank_parts = [names['crank']] + [
            n for n, m in parts.items() if n not in names.values() and abs(m.volume) < 2e-6
            and (radial(m.centroid, C, ax) < 0.001 or radial(m.centroid, M, ax) < 0.013)]
        rod_parts = [names['rod']]
        # mass: shank implied density
        ms = float(shank.find('inertial/mass').get('value'))
        rho = ms / sum(abs(m.volume) for m in parts.values())
        ine = shank.find('inertial')
        cs = np.array([float(v) for v in ine.find('origin').get('xyz').split()])
        i6 = [float(ine.find('inertia').get(k)) for k in KEYS]
        Is = np.array([[i6[0], i6[1], i6[2]], [i6[1], i6[3], i6[4]], [i6[2], i6[4], i6[5]]])
        Io = shift(ms, cs, Is)
        new_links = {}
        for lname, group, origin in ((f'{side}_ankle_crank_link', crank_parts, M),
                                     (f'{side}_ankle_rod_link', rod_parts, C)):
            m, com, I = mass_properties([parts[n] for n in group], rho)
            Io = Io - shift(m, com, I); ms -= m
            new_links[lname] = (group, origin, m, com - origin, I)
        # shank COM: total first moment minus the moved parts
        first = float(shank.find('inertial/mass').get('value')) * np.array(
            [float(v) for v in ine.find('origin').get('xyz').split()])
        for _, (_, origin, m, com_local, _) in new_links.items():
            first = first - m * (com_local + origin)
        cs = first / ms
        Is = unshift(ms, cs, Io)
        set_inertial(shank, ms, cs, Is)
        x = set_xacro_inertial(x, shank_name, ms, cs, Is)

        # mimic fits over the driven range
        grid = np.radians(np.arange(DRIVE_DEG[0], DRIVE_DEG[1] + 1e-9, 0.5))
        rel = bar.sweep(grid)
        fit_crank = np.polyfit(grid, rel[:, 0], 1); fit_rod = np.polyfit(grid, rel[:, 1], 1)
        err = [np.degrees(np.abs(np.polyval(f, grid) - rel[:, k])).max() for k, f in enumerate((fit_crank, fit_rod))]
        lim_lo, lim_hi = (float(ankle.find('limit').get(k)) for k in ('lower', 'upper'))
        full = bar.sweep(np.linspace(lim_lo, lim_hi, 400))
        limits = []
        for k, f in enumerate((fit_crank, fit_rod)):
            vals = np.concatenate([full[:, k][~np.isnan(full[:, k])], np.polyval(f, [lim_lo, lim_hi])])
            limits.append((round(vals.min() - 0.05, 4), round(vals.max() + 0.05, 4)))

        # detailed URDF: new links and joints
        foot_index = list(root).index(links[f'{side}_foot_link'])
        ankle_index = list(root).index(ankle)
        for k, (lname, (group, origin, m, com_local, I)) in enumerate(new_links.items()):
            link = ET.Element('link', name=lname)
            e_in = ET.SubElement(link, 'inertial')
            ET.SubElement(e_in, 'origin', xyz=fmt(com_local), rpy='0 0 0')
            ET.SubElement(e_in, 'mass', value=f'{m:.12g}')
            ET.SubElement(e_in, 'inertia', **{kk: f'{v:.12g}' for kk, v in zip(KEYS, (
                I[0, 0], I[0, 1], I[0, 2], I[1, 1], I[1, 2], I[2, 2]))})
            for e in [e for e in list(shank) if e.tag in ('visual', 'collision')
                      and e.find('geometry/mesh').get('filename').rsplit('/', 1)[1][:-4] in group]:
                o = e.find('origin')
                if o is None:
                    o = ET.Element('origin', xyz='0 0 0', rpy='0 0 0'); e.insert(0, o)
                T = pose(o); o.set('xyz', fmt(T[:3, 3] - origin)); o.set('rpy', o.get('rpy', '0 0 0'))
                shank.remove(e); link.append(e)
            root.insert(foot_index + 1 + k, link)
        root.insert(foot_index + 3, ET.Element('link', name=f'{side}_ankle_rod_end'))
        jdefs = [(f'{side}_ankle_crank_joint', shank_name, f'{side}_ankle_crank_link', M, fit_crank, limits[0]),
                 (f'{side}_ankle_rod_joint', f'{side}_ankle_crank_link', f'{side}_ankle_rod_link', C - M, fit_rod,
                  limits[1])]
        ankle_index = list(root).index(ankle)
        xj = ''
        for k, (jname, parent, child, xyz, fit, (lo, hi)) in enumerate(jdefs):
            j = ET.Element('joint', name=jname, type='revolute')
            ET.SubElement(j, 'origin', xyz=fmt(xyz), rpy='0 0 0')
            ET.SubElement(j, 'parent', link=parent); ET.SubElement(j, 'child', link=child)
            ET.SubElement(j, 'axis', xyz=fmt(ax))
            ET.SubElement(j, 'limit', lower=str(lo), upper=str(hi), effort='0', velocity='0')
            ET.SubElement(j, 'mimic', joint=f'{side}_ankle_joint', multiplier=f'{fit[0]:.6f}', offset=f'{fit[1]:.6f}')
            root.insert(ankle_index + 1 + k, j)
            xj += (f'  <joint name="{jname}" type="revolute">\n    <parent link="{parent}"/>\n'
                   f'    <child link="{child}"/>\n    <origin xyz="{fmt(xyz, 9)}" rpy="0 0 0"/>\n'
                   f'    <axis xyz="{fmt(ax, 9)}"/>\n    <limit lower="{lo}" upper="{hi}" effort="0" velocity="0"/>\n'
                   f'    <mimic joint="{side}_ankle_joint" multiplier="{fit[0]:.6f}" offset="{fit[1]:.6f}"/>\n'
                   f'  </joint>\n')
        j = ET.Element('joint', name=f'{side}_ankle_rod_end_joint', type='fixed')
        ET.SubElement(j, 'origin', xyz=fmt(F - C), rpy='0 0 0')
        ET.SubElement(j, 'parent', link=f'{side}_ankle_rod_link'); ET.SubElement(j, 'child', link=f'{side}_ankle_rod_end')
        root.insert(ankle_index + 3, j)
        xj += (f'  <xacro:fixed_joint name="{side}_ankle_rod_end_joint" parent="{side}_ankle_rod_link" '
               f'child="{side}_ankle_rod_end"\n      xyz="{fmt(F - C, 9)}" rpy="0 0 0"/>\n')

        # xacro: links after the foot link, joints after the ankle joint
        xl = ''
        for lname, (group, origin, m, com_local, I) in new_links.items():
            xl += (f'  <xacro:cad_link name="{lname}" material="dark_grey"\n'
                   f'                  mass="{m:.6g}" com="{fmt(com_local, 6)}"\n'
                   f'                  inertia="{fmt((I[0, 0], I[0, 1], I[0, 2], I[1, 1], I[1, 2], I[2, 2]), 6)}"/>\n')
        xl += f'  <link name="{side}_ankle_rod_end"/>\n'
        mm = re.search(rf'  <xacro:cad_link name="{side}_foot_link"[^/]*/>\n', x)
        x = x[:mm.end()] + xl + x[mm.end():]
        mm = re.search(rf'  <xacro:revolute_joint name="{side}_ankle_joint"[^/]*/>\n', x)
        note = (f'  <!-- {side} ankle four-bar (scripts/ankle_linkage.py): crank on the ankle motor, rod on the crank pin.\n'
                f'       Mimic joints: linear fit over the driven ankle range (max error crank {err[0]:.2f} deg, rod '
                f'{err[1]:.2f} deg);\n       the MuJoCo model closes the loop exactly. -->\n')
        x = x[:mm.end()] + note + xj + x[mm.end():]

        report.append((side, bar, fit_crank, fit_rod, err, rho, new_links))

    det = ET.tostring(root, encoding='unicode')
    nl = '\r\n' if '\r\n' in raw else '\n'
    if nl == '\r\n':
        det = det.replace('\r\n', '\n').replace('\n', '\r\n')
    path.write_text('<?xml version="1.0" ?>' + nl + det, newline='')
    xpath.write_text(x)
    print_report(report)


def set_inertial(link, m, c, I):
    ine = link.find('inertial')
    ine.find('mass').set('value', f'{m:.12g}')
    ine.find('origin').set('xyz', fmt(c))
    for k, v in zip(KEYS, (I[0, 0], I[0, 1], I[0, 2], I[1, 1], I[1, 2], I[2, 2])):
        ine.find('inertia').set(k, f'{v:.12g}')


def set_xacro_inertial(x, name, m, c, I):
    pat = re.compile(rf'(<xacro:cad_link name="{name}" material="\w+"\s+)mass="[^"]+" com="[^"]+"(\s+)inertia="[^"]+"')
    i6 = (I[0, 0], I[0, 1], I[0, 2], I[1, 1], I[1, 2], I[2, 2])
    x, n = pat.subn(lambda mm: f'{mm.group(1)}mass="{m:.6g}" com="{fmt(c, 6)}"{mm.group(2)}inertia="{fmt(i6, 6)}"', x)
    assert n == 1, name
    return x


def print_report(report):
    try:
        sys.path.insert(0, str(PKG.parents[1] / 'control/motor_control_hybrid'))
        from motor_control_hybrid.ankle_mapping import AnkleMapper
        import yaml
        cfg = yaml.safe_load((PKG.parents[1] / 'control/motor_control_hybrid/config/motors.yaml').read_text())
        mapper = AnkleMapper(cfg['motor_control_node']['ros__parameters']['motors']['left_ankle_joint']['ankle_mapping'])
    except Exception as exc:  # noqa: BLE001
        mapper = None
        print('ankle_mapping not available:', exc)
    for side, bar, fc, fr, err, rho, new_links in report:
        L1 = radial(bar.M, bar.A, bar.ax); L4 = bar.crank; L3 = bar.rod; L2 = radial(bar.F0, bar.A, bar.ax)
        print(f'== {side}: ground {L1 * 1000:.2f} mm, crank {L4 * 1000:.2f}, rod {L3 * 1000:.2f}, heel lever '
              f'{L2 * 1000:.2f} (ankle_mapping L1..L4: ground 166.69, crank 47.63, rod 165.10, lever 45.47)')
        print(f'   density {rho / 1000:.2f} g/cm3; ' + ', '.join(
            f'{n}: {v[2] * 1000:.1f} g ({len(v[0])} parts)' for n, v in new_links.items()))
        print(f'   mimic crank = {fc[0]:.4f} * ankle {fc[1]:+.4f} (max err {err[0]:.2f} deg), '
              f'rod = {fr[0]:.4f} * ankle {fr[1]:+.4f} (max err {err[1]:.2f} deg)')
        print('   ankle  crank(motor)  rod/crank   ankle_mapping motor')
        g = 0.0
        for a in (-120, -90, -65, -45, -20, 0, 20, 45, 75, 90, 96):
            r = bar.sweep(np.radians(np.linspace(0, a, 200)))[-1]
            mtxt = ''
            if mapper is not None and DRIVE_DEG[0] <= a <= DRIVE_DEG[1]:
                g = mapper.ankle_rad_to_motor_logical_rad(math.radians(a), 0.0)
                mtxt = f'{math.degrees(g):+7.1f}'
            print(f'   {a:+5d}   {math.degrees(r[0]):+8.1f}   {math.degrees(r[1]):+8.1f}     {mtxt}')


if __name__ == '__main__':
    main()
