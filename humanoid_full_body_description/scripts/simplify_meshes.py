#!/usr/bin/env python3
"""Build one visual and one collision mesh per link from the per-part CAD export.

Input:  cad_source/full_body_parts.urdf, which has one <visual>/<collision> per CAD part
        (1,086 parts, about 3M triangles), and the STLs in cad_source/meshes/.
Output: meshes/visual/<link>.stl     the link's parts, each decimated on its own, then concatenated
        meshes/collision/<link>.stl  the convex hull of the link's parts

Parts are decimated one at a time, never as one merged mesh. Merging first lets the decimator spend the
budget on detailed parts (screws) and collapse simple large ones: the two torso battery blocks ended up
64 mm off. Every part is checked against its original: the 95th percentile of the distance from the
original surface to the simplified part must stay under --tolerance, or that part keeps more triangles.
If a link still exceeds MAX_LINK_FACES (MuJoCo's STL limit is 200,000), that link is redone with the
tolerance doubled until it fits.

All meshes stay in the link frame, so joint origins and inertials need no change.

Offline tool, not a runtime dependency:
  python3 -m venv /tmp/meshenv
  /tmp/meshenv/bin/pip install trimesh==4.4.9 fast-simplification==0.1.9 rtree==1.3.0 scipy
  /tmp/meshenv/bin/python scripts/simplify_meshes.py            # from the package directory
"""

import argparse
import xml.etree.ElementTree as ET
from pathlib import Path

import fast_simplification
import numpy as np
import trimesh

PACKAGE = "humanoid_full_body_description"
KEEP_BELOW = 300        # parts with at most this many triangles are kept as they are
SAMPLES = 400           # surface samples per part for the deviation check
MAX_LINK_FACES = 150000 # MuJoCo rejects STL files with more than 200,000 faces


def load_parts(link, package_dir):
    """The link's collision parts as separate meshes, in the link frame."""
    parts = []
    for collision in link.findall("collision"):
        uri = collision.find("geometry/mesh").get("filename")
        path = package_dir / uri.removeprefix(f"package://{PACKAGE}/")
        part = trimesh.load(path, force="mesh")
        origin = collision.find("origin")
        if origin is not None:
            transform = trimesh.transformations.euler_matrix(
                *[float(v) for v in origin.get("rpy", "0 0 0").split()], axes="sxyz")
            transform[:3, 3] = [float(v) for v in origin.get("xyz", "0 0 0").split()]
            part.apply_transform(transform)
        part.merge_vertices()
        parts.append((path.name, part))
    return parts


def deviation(original, simplified, rng):
    """95th percentile distance (m) from the original surface to the simplified one."""
    points, _ = trimesh.sample.sample_surface(original, SAMPLES, seed=rng)
    _, distance, _ = trimesh.proximity.closest_point(simplified, points)
    return float(np.percentile(distance, 95))


def simplify_part(part, tolerance, rng):
    """Fewest triangles (doubling from KEEP_BELOW) whose deviation is within tolerance."""
    n = len(part.faces)
    target = KEEP_BELOW
    while target < n:
        points, faces = fast_simplification.simplify(
            part.vertices.astype(np.float32), part.faces.astype(np.int32),
            target_reduction=1.0 - target / n)
        candidate = trimesh.Trimesh(points, faces, process=True)
        if len(candidate.faces) and deviation(part, candidate, rng) <= tolerance:
            return candidate
        target *= 2
    return part


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--package-dir", type=Path, default=Path(__file__).resolve().parent.parent)
    parser.add_argument("--tolerance", type=float, default=0.001, help="max p95 deviation per part, m")
    args = parser.parse_args()
    package_dir = args.package_dir
    robot = ET.parse(package_dir / "cad_source/full_body_parts.urdf").getroot()
    (package_dir / "meshes/visual").mkdir(parents=True, exist_ok=True)
    (package_dir / "meshes/collision").mkdir(parents=True, exist_ok=True)
    rng = np.random.default_rng(0)

    totals = np.zeros(3, dtype=int)
    worst_overall = 0.0
    for link in robot.findall("link"):
        name = link.get("name")
        parts = load_parts(link, package_dir)
        tolerance = args.tolerance
        while True:
            simplified = [simplify_part(part, tolerance, rng) for _, part in parts]
            if sum(len(p.faces) for p in simplified) <= MAX_LINK_FACES:
                break
            tolerance *= 2
        worst, worst_part = max((deviation(p, s, rng), n) for (n, p), s in zip(parts, simplified))
        worst_overall = max(worst_overall, worst)
        visual = trimesh.util.concatenate(simplified)
        original = trimesh.util.concatenate([p for _, p in parts])
        hull = original.convex_hull
        visual.export(package_dir / f"meshes/visual/{name}.stl")
        hull.export(package_dir / f"meshes/collision/{name}.stl")
        totals += [len(original.faces), len(visual.faces), len(hull.faces)]
        print(f"{name:32s} {len(parts):4d} parts {len(original.faces):8d} -> visual {len(visual.faces):6d} "
              f"(tolerance {tolerance * 1000:.0f} mm, worst part p95 {worst * 1000:.2f} mm: {worst_part}), "
              f"hull {len(hull.faces):5d}")
    print(f"{'total':32s}      {totals[0]:13d} -> visual {totals[1]:6d}, hull {totals[2]:5d}; "
          f"worst part p95 deviation {worst_overall * 1000:.2f} mm")


if __name__ == "__main__":
    main()
