import math
from typing import Optional, Tuple

def clamp(x: float, lo: float, hi: float) -> float:
    return lo if x < lo else hi if x > hi else x

def norm3(x: float, y: float, z: float) -> float:
    return math.sqrt(x*x + y*y + z*z)

def unit3(x: float, y: float, z: float, eps: float = 1e-12) -> Optional[Tuple[float, float, float]]:
    n = norm3(x, y, z)
    if n < eps:
        return None
    return (x/n, y/n, z/n)

def dot(a, b) -> float:
    return a[0]*b[0] + a[1]*b[1] + a[2]*b[2]

def cross(a, b):
    ax, ay, az = a
    bx, by, bz = b
    return (ay*bz - az*by, az*bx - ax*bz, ax*by - ay*bx)

def mean(vals) -> float:
    return sum(vals)/len(vals) if vals else 0.0

def std(vals) -> float:
    if len(vals) < 2:
        return 0.0
    m = mean(vals)
    return math.sqrt(sum((v-m)*(v-m) for v in vals)/(len(vals)-1))
