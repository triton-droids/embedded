#!/usr/bin/env python3
"""Build one visual and one collision mesh per link from the per-part CAD export.

Input:  cad_source/full_body_parts.urdf, which has one <visual>/<collision> per CAD part
        (1,086 parts, about 3M triangles), and the STLs in cad_source/meshes/.
Output: meshes/visual/<link>.stl     the link's parts, each decimated on its own, then concatenated
        meshes/collision/<link>.stl  one convex hull per CAD part, concatenated; strongly concave parts
                                     are split into several hulls with CoACD first

Parts are decimated one at a time, never as one merged mesh. Merging first lets the decimator spend the
budget on detailed parts (screws) and collapse simple large ones: the two torso battery blocks ended up
64 mm off. Every part is checked against its original: the 95th percentile of the distance from the
original surface to the simplified part must stay under --tolerance, or that part keeps more triangles.
If a link still exceeds MAX_LINK_FACES (MuJoCo's STL limit is 200,000), that link is redone with the
tolerance doubled until it fits.

Collision follows the SO-101 (lerobot_test) approach: a true convex hull per part, not per link, so the
hull always contains the part. A single hull per link filled the gaps between parts and rejected 2-4% of
collision-free poses. A part whose hull adds more than CONCAVE_EMPTY_VOLUME of empty space (U-brackets,
frames: the pelvis bracket fills 4% of its own hull) is decomposed with CoACD. CoACD is approximate, so
the script reports how far each decomposed part's surface lies outside its hulls. Parts under
SMALL_PART_VOLUME (screws, nuts, washers) get an oriented bounding box instead of a hull, which keeps
every collision STL under MuJoCo's 200,000-face limit.
MoveIt checks the concatenated hulls as a triangle mesh. MuJoCo convexifies each collision STL as a
whole, so in MuJoCo every link is still one hull.

All meshes stay in the link frame, so joint origins and inertials need no change.

Offline tool, not a runtime dependency:
  python3 -m venv /tmp/meshenv
  /tmp/meshenv/bin/pip install trimesh==4.4.9 fast-simplification==0.1.9 rtree==1.3.0 coacd==1.0.14 scipy
  OMP_NUM_THREADS=1 /tmp/meshenv/bin/python scripts/simplify_meshes.py   # from the package directory
  (--only visual|collision regenerates one kind; collision takes about 6 min, visual about 1.5 min)
"""

import argparse
import subprocess
import sys
import tempfile
from concurrent.futures import ThreadPoolExecutor
import xml.etree.ElementTree as ET
from pathlib import Path

import coacd
import fast_simplification
import numpy as np
import trimesh

PACKAGE = "humanoid_full_body_description"
KEEP_BELOW = 300        # parts with at most this many triangles are kept as they are
SAMPLES = 400           # surface samples per part for the deviation check
MAX_LINK_FACES = 150000 # MuJoCo rejects STL files with more than 200,000 faces
CONCAVE_EMPTY_VOLUME = 50e-6  # m^3: decompose a part whose hull adds more empty space than this
COACD = dict(threshold=0.05, max_convex_hull=24, preprocess_mode="auto", seed=0,
             decimate=True, max_ch_vertex=64)
SMALL_PART_VOLUME = 5e-6  # m^3: screws, nuts, washers get an oriented bounding box (12 triangles)
WORKERS = 8


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


def collision_hulls(part):
    """Convex pieces that cover the part: a box for tiny parts, its hull, or CoACD pieces if the hull is
    mostly empty."""
    if abs(part.volume) < SMALL_PART_VOLUME:
        # A 0.2 cm3 screw has a 952-triangle hull; small parts made up 96% of the torso's hull triangles.
        return [part.bounding_box_oriented.to_mesh()], None
    hull = part.convex_hull
    if hull.volume - abs(part.volume) <= CONCAVE_EMPTY_VOLUME:
        return [hull], None
    pieces = run_coacd_isolated(part)
    if pieces is None:  # CoACD crashed on every seed: fall back to the plain hull (always covers the part)
        return [hull], (hull.volume, hull.volume, 0.0, 0.0, "CoACD failed, plain hull")
    hulls = [trimesh.Trimesh(v, f).convex_hull for v, f in pieces]
    # How far the original surface lies outside every piece (0 = fully covered).
    points, _ = trimesh.sample.sample_surface(part, 2000, seed=0)
    outside = np.min([np.maximum(-trimesh.proximity.signed_distance(h, points), 0) for h in hulls], axis=0)
    return hulls, (hull.volume, sum(h.volume for h in hulls), float(np.percentile(outside, 99)),
                   float(outside.max()), "")


def run_coacd_isolated(part):
    """CoACD in a child process: its C++ asserts abort the whole process on some meshes."""
    with tempfile.TemporaryDirectory() as tmp:
        np.savez(f"{tmp}/in.npz", v=part.vertices, f=part.faces)
        for seed in (0, 1, 2):
            run = subprocess.run([sys.executable, __file__, "--coacd-one", tmp, str(seed)],
                                 capture_output=True)
            if run.returncode == 0:
                data = np.load(f"{tmp}/out.npz")
                return [(data[f"v{i}"], data[f"f{i}"]) for i in range(int(data["n"]))]
    return None


def coacd_one(tmp, seed):
    data = np.load(f"{tmp}/in.npz")
    coacd.set_log_level("error")
    pieces = coacd.run_coacd(coacd.Mesh(data["v"], data["f"]), **{**COACD, "seed": seed})
    arrays = {f"v{i}": v for i, (v, _) in enumerate(pieces)} | {f"f{i}": f for i, (_, f) in enumerate(pieces)}
    np.savez(f"{tmp}/out.npz", n=len(pieces), **arrays)


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--package-dir", type=Path, default=Path(__file__).resolve().parent.parent)
    parser.add_argument("--tolerance", type=float, default=0.001, help="max p95 deviation per part, m")
    parser.add_argument("--only", choices=["visual", "collision"], help="regenerate only one kind")
    parser.add_argument("--links", nargs="*", help="regenerate only these links")
    parser.add_argument("--coacd-one", nargs=2, metavar=("DIR", "SEED"), help=argparse.SUPPRESS)
    args = parser.parse_args()
    if args.coacd_one:
        coacd_one(args.coacd_one[0], int(args.coacd_one[1]))
        return
    package_dir = args.package_dir
    robot = ET.parse(package_dir / "cad_source/full_body_parts.urdf").getroot()
    (package_dir / "meshes/visual").mkdir(parents=True, exist_ok=True)
    (package_dir / "meshes/collision").mkdir(parents=True, exist_ok=True)
    rng = np.random.default_rng(0)

    coacd.set_log_level("error")
    worst_overall, worst_outside = 0.0, 0.0
    for link in robot.findall("link"):
        name = link.get("name")
        if args.links and name not in args.links:
            continue
        parts = load_parts(link, package_dir)
        line = f"{name:32s} {len(parts):4d} parts {sum(len(p.faces) for _, p in parts):8d} triangles"
        if args.only != "collision":
            tolerance = args.tolerance
            while True:
                simplified = [simplify_part(part, tolerance, rng) for _, part in parts]
                if sum(len(p.faces) for p in simplified) <= MAX_LINK_FACES:
                    break
                tolerance *= 2
            worst = max(deviation(p, s, rng) for (_, p), s in zip(parts, simplified))
            worst_overall = max(worst_overall, worst)
            visual = trimesh.util.concatenate(simplified)
            visual.export(package_dir / f"meshes/visual/{name}.stl")
            line += f" | visual {len(visual.faces):6d} (tol {tolerance * 1000:.0f} mm, worst p95 {worst * 1000:.2f} mm)"
        if args.only != "visual":
            hulls, decomposed = [], []
            with ThreadPoolExecutor(WORKERS) as pool:
                results = list(pool.map(collision_hulls, [part for _, part in parts]))
            for (part_name, _), (pieces, report) in zip(parts, results):
                hulls += pieces
                if report:
                    decomposed.append((part_name, report))
                    worst_outside = max(worst_outside, report[3])
            collision = trimesh.util.concatenate(hulls)
            if len(collision.faces) > MAX_LINK_FACES:
                raise SystemExit(f"{name}: collision mesh has {len(collision.faces)} faces, over {MAX_LINK_FACES}")
            collision.export(package_dir / f"meshes/collision/{name}.stl")
            line += f" | collision {len(hulls):4d} hulls, {len(collision.faces):6d} triangles"
            for part_name, (single, pieces_volume, p99, maximum, note) in decomposed:
                line += (f"\n      CoACD {part_name}: hull {single * 1e6:.0f} -> {pieces_volume * 1e6:.0f} cm3, "
                         f"surface outside p99 {p99 * 1000:.2f} mm, max {maximum * 1000:.2f} mm {note}")
        print(line, flush=True)
    print(f"worst visual p95 deviation {worst_overall * 1000:.2f} mm, "
          f"worst CoACD surface outside {worst_outside * 1000:.2f} mm")

if __name__ == "__main__":
    main()
