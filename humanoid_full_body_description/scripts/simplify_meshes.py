#!/usr/bin/env python3
"""Build one visual and one collision mesh per link from the per-part CAD export.

Input:  cad_source/full_body_parts.urdf, which has one <visual>/<collision> per CAD part
        (1,086 parts, about 3M triangles), and the STLs in cad_source/meshes/.
Output: meshes/visual/<link>.stl     the link's parts merged, then decimated to a triangle budget
        meshes/collision/<link>.stl  the convex hull of the link's parts

All meshes stay in the link frame, so joint origins and inertials need no change.

Offline tool, not a runtime dependency:
  python3 -m venv /tmp/meshenv && /tmp/meshenv/bin/pip install trimesh==4.4.9 fast-simplification==0.1.9 scipy
  /tmp/meshenv/bin/python scripts/simplify_meshes.py            # run from the package directory
"""

import argparse
import re
import xml.etree.ElementTree as ET
from pathlib import Path

import fast_simplification
import numpy as np
import trimesh

PACKAGE = "humanoid_full_body_description"
# Visual triangle budget per link. The torso frame is a third of the robot's parts, so it gets more.
VISUAL_BUDGET = {"torso_link": 40000}
DEFAULT_VISUAL_BUDGET = 12000


def load_link_parts(link, package_dir):
    """All collision parts of a link, concatenated in the link frame."""
    parts = []
    for collision in link.findall("collision"):
        uri = collision.find("geometry/mesh").get("filename")
        part = trimesh.load(package_dir / uri.removeprefix(f"package://{PACKAGE}/"), force="mesh")
        origin = collision.find("origin")
        if origin is not None:
            xyz = [float(v) for v in origin.get("xyz", "0 0 0").split()]
            rpy = [float(v) for v in origin.get("rpy", "0 0 0").split()]
            transform = trimesh.transformations.euler_matrix(*rpy, axes="sxyz")
            transform[:3, 3] = xyz
            part.apply_transform(transform)
        parts.append(part)
    merged = trimesh.util.concatenate(parts)
    merged.merge_vertices()
    return merged, len(parts)


def decimate(mesh, budget):
    if len(mesh.faces) <= budget:
        return mesh
    points, faces = fast_simplification.simplify(
        mesh.vertices.astype(np.float32), mesh.faces.astype(np.int32),
        target_reduction=1.0 - budget / len(mesh.faces))
    return trimesh.Trimesh(points, faces, process=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--package-dir", type=Path, default=Path(__file__).resolve().parent.parent)
    args = parser.parse_args()
    package_dir = args.package_dir
    robot = ET.parse(package_dir / "cad_source/full_body_parts.urdf").getroot()
    (package_dir / "meshes/visual").mkdir(parents=True, exist_ok=True)
    (package_dir / "meshes/collision").mkdir(parents=True, exist_ok=True)

    totals = np.zeros(3, dtype=int)
    for link in robot.findall("link"):
        name = link.get("name")
        merged, n_parts = load_link_parts(link, package_dir)
        visual = decimate(merged, VISUAL_BUDGET.get(name, DEFAULT_VISUAL_BUDGET))
        hull = merged.convex_hull
        visual.export(package_dir / f"meshes/visual/{name}.stl")
        hull.export(package_dir / f"meshes/collision/{name}.stl")
        totals += [len(merged.faces), len(visual.faces), len(hull.faces)]
        print(f"{name:34s} {n_parts:4d} parts  {len(merged.faces):8d} -> visual {len(visual.faces):6d}, "
              f"collision hull {len(hull.faces):5d} triangles")
    print(f"{'total':34s}       {totals[0]:8d} -> visual {totals[1]:6d}, collision hull {totals[2]:5d}")


if __name__ == "__main__":
    main()
