"""Known-radius ball center from a partially visible RGB-D surface; no scene truth."""
import numpy as np


def point_from_sphere(u, v, width, height, depth, k, radius):
    """Return pixel and optical CENTER, or None when sphere support is insufficient.

    Bound work to 600 depth points and 1024 deterministic hypotheses. Planar
    foreground/background is rejected; no median-depth fallback on fit failure.
    """
    k = np.asarray(k, float).reshape(3, 3)
    if (not np.isfinite([u, v, width, height, radius]).all() or radius <= 0
            or min(width, height) <= 0 or not np.isfinite(k).all()
            or min(k[0, 0], k[1, 1]) <= 0):
        return None
    yy, xx = np.mgrid[max(0, int(v-height*.5)):min(depth.shape[0], int(v+height*.5)+1):2,
                      max(0, int(u-width*.5)):min(depth.shape[1], int(u+width*.5)+1):2]
    z = depth[yy, xx].ravel()
    points = np.column_stack(((xx.ravel()-k[0, 2])*z/k[0, 0],
                              (yy.ravel()-k[1, 2])*z/k[1, 1], z))
    points = points[np.isfinite(points).all(axis=1) & (z > .1) & (z < 10)]
    if len(points) < 30:
        return None
    if len(points) > 600:
        points = points[np.linspace(0, len(points)-1, 600).astype(int)]
    samples = points[np.random.default_rng(13).integers(0, len(points), (1024, 4))]
    a = 2*(samples[:, 1:] - samples[:, :1])
    b = (samples[:, 1:]**2).sum(axis=2) - (samples[:, :1]**2).sum(axis=2)
    valid = abs(np.linalg.det(a)) > 1e-9
    if not valid.any():
        return None
    try:
        centers = np.linalg.solve(a[valid], b[valid])
        radii = np.linalg.norm(centers-samples[valid, 0], axis=1)
        centers = centers[(radii > .75*radius) & (radii < 1.25*radius)]
        if not len(centers):
            return None
        residual = abs(np.linalg.norm(points[None, :, :]-centers[:, None, :], axis=2)-radius)
        tolerance = max(.003, .05*radius)
        counts = (residual < tolerance).sum(axis=1)
        best = counts.argmax()
        if counts[best] < max(30, .15*len(points)):
            return None
        center = centers[best].copy()
        inliers = points[residual[best] < tolerance]
        for _ in range(5):
            diff = center-inliers
            dist = np.linalg.norm(diff, axis=1)
            jac = diff / np.maximum(dist[:, None], 1e-6)
            delta = np.linalg.lstsq(jac, radius-dist, rcond=None)[0]
            if np.linalg.norm(delta) > .3*radius:
                return None
            center += delta
    except np.linalg.LinAlgError:
        return None
    if not np.isfinite(center).all() or center[2] <= radius:
        return None
    cu = k[0, 2] + k[0, 0]*center[0]/center[2]
    cv = k[1, 2] + k[1, 1]*center[1]/center[2]
    if (not 0 <= cu < depth.shape[1] or not 0 <= cv < depth.shape[0]
            or abs(cu-u) > .7*width or abs(cv-v) > .7*height):
        return None
    return float(cu), float(cv), *map(float, center)
