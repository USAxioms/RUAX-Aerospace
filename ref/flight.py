"""
CAI-AERO-01 flight kinematic sentinel: canonical WAD18 integer-only reference.
No floats. No true division. No raise/assert. Imports: hashlib, math only.
Total on well-formed 17-tuple of signed integers (all WAD-scaled).
ALL THRESHOLDS AND PHYSICAL CONSTANTS ARE PLACEHOLDERS.
NOT A FLIGHT COMPUTER. NOT SAFETY-CERTIFIED. NOT DO-178C.
"""
import hashlib
from math import isqrt

WAD       = 10 ** 18
CAP       = 10 ** 30
TWO_WAD   = 2 * WAD

# Physical constant: g = 9.80665 m/s² as integer (9.80665 × WAD, truncated)
G_ACCEL   = 9806650000000000000

# Envelope thresholds — PLACEHOLDERS; replace with certified limits before any use
G_MAX_WAD    = 9 * WAD               # 9 g structural limit
SPEED_MAX    = 7900 * WAD            # 7900 m/s LEO-class placeholder
AOA_COS_MIN  = 600000000000000000    # cos(≈53.1°, arctan(4/3))×WAD; F_AOA when cos < this
QNORM_TOL    = 10000                 # quaternion norm tolerance from WAD (integer ulps)
MAX_ITER     = 64                    # max R3 refinement passes

# Telemetry indices (17-tuple)
I_DT                     = 0
I_AX, I_AY, I_AZ        = 1, 2, 3
I_WX, I_WY, I_WZ        = 4, 5, 6
I_QW, I_QX, I_QY, I_QZ  = 7, 8, 9, 10
I_PX, I_PY, I_PZ         = 11, 12, 13
I_VX, I_VY, I_VZ         = 14, 15, 16
ARITY = 17

# Envelope flag bits
F_GLOAD  = 1
F_SPEED  = 2
F_AOA    = 4
F_QNORM  = 8

# Actions
ACT_NOMINAL = 0
ACT_BREACH  = 1
ACT_FAULT   = 2

GENESIS    = bytes(32)
TRACE_KEYS = ("qw","qx","qy","qz","px","py","pz","vx","vy","vz",
              "g_load","speed_sq","aoa_cos","qnorm_err",
              "flags","refine_iters","conv","action")


def iabs(a):
    return a if a >= 0 else -a


def smul(a, b):
    """WAD signed multiply, truncates toward zero."""
    q = (iabs(a) * iabs(b)) // WAD
    return -q if (a < 0) != (b < 0) else q


def sdiv(a, b):
    """WAD signed divide, truncates toward zero; 0 for b == 0."""
    if b == 0:
        return 0
    q = (iabs(a) * WAD) // iabs(b)
    return -q if (a < 0) != (b < 0) else q


def half_smul(x, y):
    """x * y / (2 * WAD) truncating toward zero (single floor, minimum error)."""
    p = x * y
    return p // TWO_WAD if p >= 0 else -((-p) // TWO_WAD)


def clampv(x, lo, hi):
    if x < lo:
        return lo
    if x > hi:
        return hi
    return x


def vec3_norm(a, b, c):
    return isqrt(a * a + b * b + c * c)


def q4_norm(w, x, y, z):
    return isqrt(w * w + x * x + y * y + z * z)


def body_x(qw, qx, qy, qz):
    """World-frame body +x axis from quaternion (first column of rotation matrix)."""
    bx = WAD - 2 * smul(qy, qy) - 2 * smul(qz, qz)
    by = 2 * (smul(qx, qy) + smul(qw, qz))
    bz = 2 * (smul(qx, qz) - smul(qw, qy))
    return bx, by, bz


def qupdate(qw, qx, qy, qz, wx, wy, wz, dt):
    """Euler: q_new = q + 0.5 * (q ⊗ Ω) * dt (un-normalized)."""
    dw = -(smul(qx, wx) + smul(qy, wy) + smul(qz, wz))
    dx =   smul(qw, wx) + smul(qy, wz) - smul(qz, wy)
    dy =   smul(qw, wy) - smul(qx, wz) + smul(qz, wx)
    dz =   smul(qw, wz) + smul(qx, wy) - smul(qy, wx)
    return (qw + half_smul(dw, dt),
            qx + half_smul(dx, dt),
            qy + half_smul(dy, dt),
            qz + half_smul(dz, dt))


def refine_qnorm(w, x, y, z, tol, max_iter):
    """R3 refinement: renormalize until |norm(q) - WAD| <= tol or max_iter hit."""
    iters = 0
    for _ in range(max_iter):
        n = q4_norm(w, x, y, z)
        if iabs(n - WAD) <= tol:
            break
        if n == 0:
            w, x, y, z = WAD, 0, 0, 0
            break
        w = (w * WAD) // n
        x = (x * WAD) // n
        y = (y * WAD) // n
        z = (z * WAD) // n
        iters += 1
    conv = 1 if iabs(q4_norm(w, x, y, z) - WAD) <= tol else 0
    return w, x, y, z, iters, conv


def clamp_inputs(tel):
    return tuple(clampv(v, -CAP, CAP) for v in tel)


def audit(tel):
    t = clamp_inputs(tel)
    dt                      = t[I_DT]
    ax, ay, az              = t[I_AX], t[I_AY], t[I_AZ]
    wx, wy, wz              = t[I_WX], t[I_WY], t[I_WZ]
    qw, qx, qy, qz         = t[I_QW], t[I_QX], t[I_QY], t[I_QZ]
    px, py, pz              = t[I_PX], t[I_PY], t[I_PZ]
    vx, vy, vz              = t[I_VX], t[I_VY], t[I_VZ]

    # Diagnostic: pre-update quaternion norm error
    qnorm_err = iabs(q4_norm(qw, qx, qy, qz) - WAD)

    # Attitude update with R3 renormalization
    nw, nx, ny, nz = qupdate(qw, qx, qy, qz, wx, wy, wz, dt)
    nw, nx, ny, nz, r_iters, conv = refine_qnorm(nw, nx, ny, nz, QNORM_TOL, MAX_ITER)

    # Euler kinematic integration (position uses pre-update velocity)
    vx_n = vx + smul(ax, dt)
    vy_n = vy + smul(ay, dt)
    vz_n = vz + smul(az, dt)
    px_n = px + smul(vx, dt)
    py_n = py + smul(vy, dt)
    pz_n = pz + smul(vz, dt)

    # G-load: |a| / g_accel, in WAD (units of g)
    a_mag  = vec3_norm(ax, ay, az)
    g_load = sdiv(a_mag, G_ACCEL)

    # Speed squared: |v_new|² / WAD (reduces scale; in units of WAD per m²/s²)
    speed_sq     = (vx_n * vx_n + vy_n * vy_n + vz_n * vz_n) // WAD
    speed_max_sq = (SPEED_MAX * SPEED_MAX) // WAD

    # AoA: cosine of angle between velocity and body +x axis (from updated quaternion)
    bx_w, bx_x, bx_z = body_x(nw, nx, ny, nz)
    v_mag = vec3_norm(vx_n, vy_n, vz_n)
    if v_mag > 0:
        dot     = smul(vx_n, bx_w) + smul(vy_n, bx_x) + smul(vz_n, bx_z)
        aoa_cos = sdiv(dot, v_mag)
    else:
        aoa_cos = WAD   # zero velocity: AoA undefined; return cos(0°)=1

    # Envelope flags
    flags = 0
    if g_load > G_MAX_WAD:          flags += F_GLOAD
    if speed_sq > speed_max_sq:     flags += F_SPEED
    if aoa_cos < AOA_COS_MIN:       flags += F_AOA
    if conv == 0:                   flags += F_QNORM

    if conv == 0:
        action = ACT_FAULT
    elif flags != 0:
        action = ACT_BREACH
    else:
        action = ACT_NOMINAL

    return {"qw": nw, "qx": nx, "qy": ny, "qz": nz,
            "px": px_n, "py": py_n, "pz": pz_n,
            "vx": vx_n, "vy": vy_n, "vz": vz_n,
            "g_load": g_load, "speed_sq": speed_sq, "aoa_cos": aoa_cos,
            "qnorm_err": qnorm_err,
            "flags": flags, "refine_iters": r_iters, "conv": conv, "action": action}


def _w(v):
    return (v % (2 ** 256)).to_bytes(32, "big")


def next_hash(prev, n, tel, trace):
    data = prev + _w(n)
    for v in clamp_inputs(tel):
        data += _w(v)
    for k in TRACE_KEYS:
        data += _w(trace[k])
    return hashlib.sha256(data).digest()


def run(seq):
    h = GENESIS
    out = []
    for n, tel in enumerate(seq):
        tr = audit(tel)
        h = next_hash(h, n, tel, tr)
        out.append((tr, h.hex()))
    return out
