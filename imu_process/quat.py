import math
from math3d import clamp

# quaternion format: (x,y,z,w) with scalar w

def q_norm(q) -> float:
    x, y, z, w = q
    return math.sqrt(x*x + y*y + z*z + w*w)

def q_normalize(q, eps: float = 1e-12):
    n = q_norm(q)
    if n < eps:
        return (0.0, 0.0, 0.0, 1.0)
    x, y, z, w = q
    return (x/n, y/n, z/n, w/n)

def q_conj(q):
    x, y, z, w = q
    return (-x, -y, -z, w)

def q_mul(q1, q2):
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    x = w1*x2 + x1*w2 + y1*z2 - z1*y2
    y = w1*y2 - x1*z2 + y1*w2 + z1*x2
    z = w1*z2 + x1*y2 - y1*x2 + z1*w2
    w = w1*w2 - x1*x2 - y1*y2 - z1*z2
    return (x, y, z, w)

def q_rotate_world_to_body(q, v_world):
    vx, vy, vz = v_world
    vq = (vx, vy, vz, 0.0)
    return q_mul(q_mul(q, vq), q_conj(q))[:3]

def q_from_axis_angle(axis, angle_rad: float):
    ax, ay, az = axis
    half = 0.5 * angle_rad
    s = math.sin(half)
    c = math.cos(half)
    return q_normalize((ax*s, ay*s, az*s, c))

def q_slerp(q0, q1, t: float):
    x0, y0, z0, w0 = q0
    x1, y1, z1, w1 = q1
    cosom = x0*x1 + y0*y1 + z0*z1 + w0*w1

    if cosom < 0.0:  # shortest path
        cosom = -cosom
        x1, y1, z1, w1 = -x1, -y1, -z1, -w1

    if cosom > 0.9995:  # lerp
        x = x0 + t*(x1 - x0)
        y = y0 + t*(y1 - y0)
        z = z0 + t*(z1 - z0)
        w = w0 + t*(w1 - w0)
        return q_normalize((x, y, z, w))

    omega = math.acos(clamp(cosom, -1.0, 1.0))
    sinom = math.sin(omega)
    a = math.sin((1.0 - t) * omega) / sinom
    b = math.sin(t * omega) / sinom
    return (a*x0 + b*x1, a*y0 + b*y1, a*z0 + b*z1, a*w0 + b*w1)

def q_to_euler_deg(q):
    x, y, z, w = q
    # ZYX intrinsic
    siny = 2.0*(w*z + x*y)
    cosy = 1.0 - 2.0*(y*y + z*z)
    yaw = math.atan2(siny, cosy)

    sinp = 2.0*(w*y - z*x)
    sinp = clamp(sinp, -1.0, 1.0)
    pitch = math.asin(sinp)

    sinr = 2.0*(w*x + y*z)
    cosr = 1.0 - 2.0*(x*x + y*y)
    roll = math.atan2(sinr, cosr)

    rad2deg = 57.29577951308232
    return (roll*rad2deg, pitch*rad2deg, yaw*rad2deg)

def q_dot(q, omega_rad_s):
    # qdot = 0.5 * q ⊗ (ω,0)
    ox, oy, oz = omega_rad_s
    omega_q = (ox, oy, oz, 0.0)
    x, y, z, w = q_mul(q, omega_q)
    return (0.5*x, 0.5*y, 0.5*z, 0.5*w)

def q_integrate_midpoint(q, omega_prev, omega_now, dt: float):
    # RK2 midpoint for quaternion kinematics
    omega_mid = ((omega_prev[0] + omega_now[0]) * 0.5,
                 (omega_prev[1] + omega_now[1]) * 0.5,
                 (omega_prev[2] + omega_now[2]) * 0.5)
    qd = q_dot(q, omega_mid)
    q_pred = (q[0] + qd[0]*dt,
              q[1] + qd[1]*dt,
              q[2] + qd[2]*dt,
              q[3] + qd[3]*dt)
    return q_normalize(q_pred)
