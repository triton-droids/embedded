#!/usr/bin/env python3
"""Import the D435 CAD archive from a td_perception_camera checkout.

Uses assembly tessellation directly; no CAD toolkit or physical device needed.
All mesh coordinates remain in mm and scene.py applies the metre conversion.
"""
import argparse
import hashlib
import json
from pathlib import Path
import struct
import subprocess
import zipfile


def git_value(root, *args):
    return subprocess.check_output(['git', '-C', str(root), *args], text=True).strip()


def import_assets(source_root, output):
    archive = source_root / 'ds3225_d435_pantilt_rev_k_slotted_horn.zip'
    member = 'ds3225_d435_pantilt_rev_k_slotted_horn/meshes.json'
    with zipfile.ZipFile(archive) as zipped:
        data = zipped.read(member)
    output.mkdir(parents=True, exist_ok=True)
    records = []
    for part in json.loads(data):
        name = part['name']
        if name.startswith(('KEEP_', 'HW_', 'REF_SERVO_', 'REF_camera_')):
            continue
        if Path(name).name != name or '/' in name or '\\' in name:
            raise ValueError('Invalid CAD part name')
        triangles = part['triangles']
        with (output / (name + '.stl')).open('wb') as stream:
            stream.write(b'CAD revision K reference visual; units mm'.ljust(80, b'\0'))
            stream.write(struct.pack('<I', len(triangles)))
            for triangle in triangles:
                points = [part['vertices'][i] for i in triangle]
                stream.write(struct.pack('<12fH', 0, 0, 0, *sum(points, []), 0))
        records.append({key: part[key] for key in ('name', 'group', 'color', 'note')})
    (output / 'mount.json').write_text(json.dumps(records, indent=2) + '\n')
    provenance = {
        'source': str(archive), 'archive_member': member,
        'source_sha256': hashlib.sha256(data).hexdigest(),
        'archive_sha256': hashlib.sha256(archive.read_bytes()).hexdigest(),
        'repository': git_value(source_root, 'remote', 'get-url', 'origin'),
        'revision_commit': git_value(source_root, 'rev-parse', 'HEAD'),
        'revision_branch': git_value(source_root, 'branch', '--show-current'),
        'revision': 'K USB_LEFT', 'units': 'mm',
        'status': 'Imported from user-provided td_perception_camera checkout. Reference CAD, not vendor D435 production geometry. Mass/inertia are simulation assumptions.',
        'pitch_axis_z_mm': 88, 'yaw_axis_x_mm': 20,
    }
    (output / 'SOURCE.json').write_text(json.dumps(provenance, indent=2) + '\n')
    return provenance


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('source_root', type=Path)
    parser.add_argument('--output', type=Path, default=Path(__file__).resolve().parents[1] / 'assets')
    args = parser.parse_args()
    print(json.dumps(import_assets(args.source_root.resolve(), args.output), indent=2))
