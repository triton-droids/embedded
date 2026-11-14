#!/usr/bin/env python3
"""Import the revision L D435 CAD directory from embedded/mechanical.

Uses assembly tessellation directly; no CAD toolkit or physical device needed.
All mesh coordinates remain in mm and scene.py applies the metre conversion.
"""
import argparse
import hashlib
import json
from pathlib import Path
import struct
import subprocess


def git_value(root, *args):
    return subprocess.check_output(['git', '-C', str(root), *args], text=True).strip()


def import_assets(source_root, output):
    mesh_file = source_root / 'meshes.json'
    data = mesh_file.read_bytes()
    parameters = json.loads((source_root / 'parameters.json').read_text())
    if parameters['revision'] != 'L' or parameters['units'] != 'mm':
        raise ValueError('Expected revision L CAD in millimetres')
    output.mkdir(parents=True, exist_ok=True)
    records = []
    for part in json.loads(data):
        name = part['name']
        if name.startswith(('KEEP_', 'HW_', 'REF_SERVO_', 'REF_camera_', 'VIS_IMU_axis_')):
            continue
        if Path(name).name != name or '/' in name or '\\' in name:
            raise ValueError('Invalid CAD part name')
        triangles = part['triangles']
        with (output / (name + '.stl')).open('wb') as stream:
            stream.write(b'CAD revision L reference visual; units mm'.ljust(80, b'\0'))
            stream.write(struct.pack('<I', len(triangles)))
            for triangle in triangles:
                points = [part['vertices'][i] for i in triangle]
                stream.write(struct.pack('<12fH', 0, 0, 0, *sum(points, []), 0))
        records.append({key: part[key] for key in ('name', 'group', 'color', 'note')})
    (output / 'mount.json').write_text(json.dumps(records, indent=2) + '\n')
    (output / 'mount_parameters.json').write_text(json.dumps(parameters, indent=2) + '\n')
    provenance = {
        'source': str(mesh_file),
        'source_sha256': hashlib.sha256(data).hexdigest(),
        'parameters_sha256': hashlib.sha256((source_root / 'parameters.json').read_bytes()).hexdigest(),
        'repository': git_value(source_root, 'remote', 'get-url', 'origin'),
        'revision_commit': git_value(source_root, 'rev-parse', 'HEAD'),
        'revision_branch': git_value(source_root, 'branch', '--show-current'),
        'source_files_tracked': bool(git_value(source_root, 'ls-files', '--', 'meshes.json', 'parameters.json')),
        'revision': 'L USB_LEFT', 'units': 'mm',
        'status': 'Imported from local embedded/mechanical revision L exports. Repository commit is context; source files may be untracked, so SHA256 identifies the exports. Reference CAD, not vendor D435 production geometry. Mass/inertia are simulation assumptions.',
        'pitch_axis_z_mm': parameters['pitch_axis_z'],
        'yaw_axis_x_mm': parameters['yaw_axis_x'],
        'imu_mount': parameters['imu_mount'],
    }
    (output / 'SOURCE.json').write_text(json.dumps(provenance, indent=2) + '\n')
    return provenance


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('source_root', type=Path)
    parser.add_argument('--output', type=Path, default=Path(__file__).resolve().parents[1] / 'assets')
    args = parser.parse_args()
    print(json.dumps(import_assets(args.source_root.resolve(), args.output), indent=2))
