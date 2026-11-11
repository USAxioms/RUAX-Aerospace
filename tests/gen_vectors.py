"""Deterministic vector generation: integer LCG + exhaustive boundary sweeps. No floats."""
import json, sys, os
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "ref"))
import flight as f

W=f.WAD; G=f.G_ACCEL; SM=f.SPEED_MAX; CAP=f.CAP
H = 707106781186547524   # floor(WAD/sqrt(2)), near-45deg quaternion component

def mk17(dt=W,ax=0,ay=0,az=0,wx=0,wy=0,wz=0,qw=W,qx=0,qy=0,qz=0,px=0,py=0,pz=0,vx=0,vy=0,vz=0):
    return (dt,ax,ay,az,wx,wy,wz,qw,qx,qy,qz,px,py,pz,vx,vy,vz)

EDGE = [
    # Anchors
    mk17(),                                              # identity nominal
    mk17(az=-G),                                         # 1g downward
    mk17(vx=W),                                          # forward flight 1 m/s
    mk17(vx=3*W, vy=4*W),                               # 3-4-5: aoa_cos==AOA_COS_MIN, not flagged
    mk17(vx=3*W-1, vy=4*W),                             # aoa_cos < AOA_COS_MIN, flagged
    # G-load boundary (at 9g, 9g+9, 9g+10)
    mk17(az=9*G),                                        # exactly 9g: not flagged
    mk17(az=9*G+9),                                      # still not flagged (rounding)
    mk17(az=9*G+10),                                     # flagged (g_load = 9W+1)
    mk17(az=-(9*G+10)),                                  # negative az same magnitude
    mk17(ax=5*G, ay=5*G, az=5*G),                        # diagonal 3D 9g-ish
    mk17(az=20*G),                                       # well over limit
    # Speed boundary
    mk17(vx=SM),                                         # exactly SPEED_MAX: not flagged
    mk17(vx=SM+1),                                       # flagged
    mk17(vx=SM, vy=1),                                   # diagonal just over
    mk17(vx=-SM),                                        # negative direction
    # Quaternion degenerate and boundary
    mk17(qw=0,qx=0,qy=0,qz=0),                          # all-zero q: refine→identity
    mk17(qw=0,qx=0,qy=0,qz=W),                          # 180° about z
    mk17(qw=H,qx=0,qy=0,qz=H),                          # ~90° about z
    mk17(qw=H,qx=0,qy=H,qz=0),                          # ~90° about y
    mk17(qw=H,qx=H,qy=0,qz=0),                          # ~90° about x
    mk17(qw=0,qx=W,qy=0,qz=0),                          # 180° about x
    mk17(qw=0,qx=0,qy=W,qz=0),                          # 180° about y
    mk17(qw=2*W,qx=0,qy=0,qz=0),                        # over-normalized q
    mk17(qw=W//2,qx=0,qy=0,qz=0),                       # under-normalized q
    mk17(qw=-W,qx=0,qy=0,qz=0),                         # negative w (valid unit quat)
    # Angular rate excitation with identity q
    mk17(wx=W//10),                                      # small roll
    mk17(wy=W//10),                                      # small pitch
    mk17(wz=W//10),                                      # small yaw
    mk17(wx=W, wy=W, wz=W),                              # large rates
    mk17(wx=-W//10, wz=W//20),                           # mixed rates
    # dt boundaries
    mk17(dt=0),                                          # zero timestep: no update
    mk17(dt=W//100, az=-G),                              # 10ms timestep
    mk17(dt=W//1000, wx=W//100),                         # 1ms with rotation
    mk17(dt=10*W),                                       # large timestep
    mk17(dt=-W),                                         # negative dt (clamped → -CAP)
    # AoA with non-identity quaternion (90° about z: body x→world y)
    mk17(qw=H,qx=0,qy=0,qz=H,vx=0,vy=W),               # body-forward=world-y, v=world-y → AoA≈0
    mk17(qw=H,qx=0,qy=0,qz=H,vx=W,vy=0),               # body-forward=world-y, v=world-x → AoA=90°
    # Combined accel + velocity
    mk17(az=-G, vz=-100*W),                              # falling, with downward velocity
    mk17(ax=G, vx=100*W, px=1000*W),                     # horizontal acceleration in flight
    # Large and cap values
    mk17(ax=CAP, ay=CAP, az=CAP, vx=CAP, vy=CAP, vz=CAP),
    mk17(ax=-CAP, ay=-CAP, az=-CAP, vx=-CAP, vy=-CAP, vz=-CAP),
    mk17(*([2*CAP]*17)),                                  # over-cap: all clamped
    mk17(*([-(2*CAP)]*17)),
    mk17(qw=CAP, qx=CAP, qy=CAP, qz=CAP),
    mk17(qw=-CAP, qx=-CAP, qy=-CAP, qz=-CAP),
    mk17(px=CAP, py=CAP, pz=CAP, vx=CAP, vy=CAP, vz=CAP),
    # Zero cases
    mk17(*([0]*17)),                                      # all zero (q degenerate→identity)
    # State chain: sequential motion sequence
    mk17(dt=W//100, ax=G//10, az=-G, qw=W),
    mk17(dt=W//100, ax=G//10, az=-G, qw=W, vx=G//1000, vz=-G//1000),
    mk17(dt=W//100, ax=0, az=-G, wx=W//50, qw=W, vx=G//500),
]

def lcg(seed):
    x = seed
    while True:
        x = (x * 6364136223846793005 + 1442695040888963407) % (2**64)
        yield x >> 33

def signed_rnd(g, scale):
    v = next(g) % (2 * scale + 1)
    return v - scale

def generate(n=400):
    rows = list(EDGE)
    g = lcg(20261104)
    for _ in range(n):
        dt = W // 100 + next(g) % (W // 10)
        a = [signed_rnd(g, 15*G) for _ in range(3)]
        w = [signed_rnd(g, 2*W) for _ in range(3)]
        qn = [signed_rnd(g, W) for _ in range(4)]; qn[0] = iabs(qn[0]) or W  # keep qw non-negative usually
        p = [signed_rnd(g, 10**6 * W) for _ in range(3)]
        v = [signed_rnd(g, SM + SM//4) for _ in range(3)]
        rows.append(tuple([dt] + a + w + qn + p + v))
    return rows

def iabs(a): return a if a >= 0 else -a

if __name__ == "__main__":
    rows = generate()
    res = f.run(rows)
    out = [{"tel": list(r), "trace": t, "hash": h} for r, (t, h) in zip(rows, res)]
    with open(os.path.join(HERE, "vectors.json"), "w") as fh:
        json.dump(out, fh, sort_keys=True)
    print("vectors", len(out), "final", out[-1]["hash"])
