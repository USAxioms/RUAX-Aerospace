"""Lightweight suite for mutation testing: anchors + first 55 edge vectors only. No full chain."""
import ast, json, os
HERE = os.path.dirname(os.path.abspath(__file__))
W = 10**18; G = 9806650000000000000; SM = 7900 * W
ALLOWED_IMPORTS = {"hashlib","math"}

def lint(src):
    bad = []
    for n in ast.walk(ast.parse(src)):
        if isinstance(n, ast.Constant) and isinstance(n.value, float): bad.append("float_literal")
        if isinstance(n, ast.Div): bad.append("true_division")
        if isinstance(n, ast.Call) and getattr(n.func,"id","") == "float": bad.append("float_call")
        if isinstance(n, (ast.Raise, ast.Assert)): bad.append("raise_or_assert")
    return bad

def run_suite(m, src):
    f = ["LINT:"+b for b in lint(src)]
    try:
        def mk(**kw):
            b=dict(dt=W,ax=0,ay=0,az=0,wx=0,wy=0,wz=0,qw=W,qx=0,qy=0,qz=0,px=0,py=0,pz=0,vx=0,vy=0,vz=0)
            b.update(kw); return tuple(b.values())
        # Anchor: identity
        ai = m.audit(mk())
        if ai["qw"]!=W or ai["g_load"]!=0 or ai["aoa_cos"]!=W or ai["flags"]!=0 or ai["action"]!=0: f.append("ANCHOR_IDENTITY")
        # 1g
        if m.audit(mk(az=-G))["g_load"] != W: f.append("ANCHOR_1G")
        # Forward AoA
        if m.audit(mk(vx=W))["aoa_cos"] != W: f.append("ANCHOR_AOA_FWD")
        # G-load boundary
        if m.audit(mk(az=9*G))["flags"] & m.F_GLOAD:         f.append("GLOAD_9G_NOFLAG")
        if m.audit(mk(az=9*G))["g_load"] != 9*W:             f.append("GLOAD_9G_VALUE")
        if not m.audit(mk(az=9*G+10))["flags"] & m.F_GLOAD:  f.append("GLOAD_BOUNDARY_FLAG")
        if m.audit(mk(az=9*G+10))["g_load"] != 9*W+1:        f.append("GLOAD_BOUNDARY_VAL")
        if m.audit(mk(az=9*G+9))["flags"] & m.F_GLOAD:       f.append("GLOAD_9G9_NOFLAG")
        # Speed boundary
        if m.audit(mk(vx=SM))["flags"] & m.F_SPEED:          f.append("SPEED_MAX_NOFLAG")
        if not m.audit(mk(vx=SM+1))["flags"] & m.F_SPEED:    f.append("SPEED_MAX1_FLAG")
        # AoA boundary
        if m.audit(mk(vx=3*W,vy=4*W))["flags"] & m.F_AOA:    f.append("AOA_3_4_NOFLAG")
        if not m.audit(mk(vx=3*W-1,vy=4*W))["flags"] & m.F_AOA: f.append("AOA_3M1_4_FLAG")
        # body_x exact
        if m.body_x(W,0,0,0)!=(W,0,0):  f.append("BODY_X_IDENTITY")
        if m.body_x(0,0,0,W)!=(-W,0,0): f.append("BODY_X_180Z")
        if m.body_x(0,0,W,0)!=(-W,0,0): f.append("BODY_X_180Y")
        # Signed ops
        if m.smul(-3*W,2*W)!=-6*W:          f.append("SMUL_NEG")
        if m.sdiv(-7*W,2*W)!=-35*10**17:    f.append("SDIV_NEG")
        if m.sdiv(W,0)!=0:                   f.append("SDIV_ZERO")
        if m.half_smul(W,W)!=W//2:           f.append("HALF_SMUL")
        if m.half_smul(-W,W)!=-(W//2):       f.append("HALF_SMUL_NEG")
        # Clamp
        if m.clamp_inputs((-10**31,)*17)!=(-10**30,)*17: f.append("CLAMP")
        # Degenerate q
        dq = m.audit(mk(qw=0,qx=0,qy=0,qz=0))
        if dq["conv"]!=1 or dq["qw"]!=W: f.append("DEGEN_Q")
        # dt=0
        a0 = m.audit(mk(dt=0,ax=G,vx=W))
        if a0["vx"]!=W or a0["px"]!=0: f.append("DT_ZERO")
        # Constant pins (kill AOA_COS_MIN+1, QNORM_TOL+1, MAX_ITER+1, ARITY+1, ACT_FAULT+1 etc.)
        if m.AOA_COS_MIN  != 600000000000000000:  f.append("PIN_AOA_COS_MIN")
        if m.QNORM_TOL    != 10000:               f.append("PIN_QNORM_TOL")
        if m.MAX_ITER     != 64:                  f.append("PIN_MAX_ITER")
        if m.ARITY        != 17:                  f.append("PIN_ARITY")
        if m.ACT_FAULT    != 2:                   f.append("PIN_ACT_FAULT")
        if m.ACT_BREACH   != 1:                   f.append("PIN_ACT_BREACH")
        if m.ACT_NOMINAL  != 0:                   f.append("PIN_ACT_NOMINAL")
        if m.GENESIS      != bytes(32):            f.append("PIN_GENESIS")
        if m.TWO_WAD      != 2 * W:               f.append("PIN_TWO_WAD")
        # Exact refine_qnorm tol boundary: q=(WAD+QNORM_TOL, 0,0,0) → norm error == tol → NOT refined further
        q_tol = m.refine_qnorm(W + m.QNORM_TOL, 0, 0, 0, m.QNORM_TOL, m.MAX_ITER)
        if q_tol[4] != 0: f.append("REFINE_TOL_NO_STEP")   # already within tol: 0 iters
        if q_tol[5] != 1: f.append("REFINE_TOL_CONV")
        # q one ulp over tol: needs exactly 1 step
        q_tol1 = m.refine_qnorm(W + m.QNORM_TOL + 1, 0, 0, 0, m.QNORM_TOL, m.MAX_ITER)
        if q_tol1[4] != 1: f.append("REFINE_TOL1_STEP")    # one step needed
        if q_tol1[5] != 1: f.append("REFINE_TOL1_CONV")
        # Mini hash-chain: 2 steps from GENESIS must match deterministic value
        import hashlib
        tel0 = mk()
        t0   = m.audit(tel0)
        h0   = hashlib.sha256(m.GENESIS + (0 % 2**256).to_bytes(32,"big")
               + b"".join((x % 2**256).to_bytes(32,"big") for x in m.clamp_inputs(tel0))
               + b"".join((t0[k] % 2**256).to_bytes(32,"big") for k in m.TRACE_KEYS)).digest()
        chain = m.run([tel0])
        if chain[0][1] != h0.hex(): f.append("HASH_CHAIN_STEP0")
        # AoA exact boundary: 3-4-5 Pythagorean → aoa_cos == AOA_COS_MIN exactly (not flagged)
        if m.audit(mk(vx=3*W, vy=4*W))["aoa_cos"] != 600000000000000000: f.append("AOA_345_VALUE")
        if m.audit(mk(vx=3*W-1, vy=4*W))["aoa_cos"] != 599999999999999999: f.append("AOA_345M1_VALUE")
        # Monotonicity
        if not m.audit(mk(az=2*G))["g_load"] > m.audit(mk(az=G))["g_load"]: f.append("MONO_GLOAD")
        # 55 edge vectors for trace consistency + flag checks
        vecs = json.load(open(os.path.join(HERE,"vectors.json")))[:55]
        for i, v in enumerate(vecs):
            t = m.audit(tuple(v["tel"]))
            if t != v["trace"]: f.append("TRACE@%d"%i)
            # Flag-value consistency
            if bool(t["flags"]&m.F_GLOAD) != bool(t["g_load"] > m.G_MAX_WAD):         f.append("FC_G@%d"%i)
            if bool(t["flags"]&m.F_SPEED) != bool(t["speed_sq"] > (m.SPEED_MAX*m.SPEED_MAX)//m.WAD): f.append("FC_S@%d"%i)
            if bool(t["flags"]&m.F_AOA)   != bool(t["aoa_cos"] < m.AOA_COS_MIN):       f.append("FC_A@%d"%i)
            if bool(t["flags"]&m.F_QNORM) != bool(t["conv"]==0):                        f.append("FC_Q@%d"%i)
            if t["action"] not in (0,1,2): f.append("ACT_RANGE@%d"%i)
            if t["g_load"] < 0:            f.append("G_NEG@%d"%i)
    except Exception as ex:
        f.append("EXC:%s"%type(ex).__name__)
    return f
