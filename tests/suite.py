"""Verification suite. run_suite(module, source) -> list of failures (empty = pass)."""
import ast, json, os
HERE = os.path.dirname(os.path.abspath(__file__))
W = 10**18
G = 9806650000000000000
SM = 7900 * W
ALLOWED_IMPORTS = {"hashlib","math"}

def lint(src):
    bad = []
    for n in ast.walk(ast.parse(src)):
        if isinstance(n, ast.Constant) and isinstance(n.value, float): bad.append("float_literal")
        if isinstance(n, ast.Div): bad.append("true_division")
        if isinstance(n, ast.Call) and getattr(n.func,"id","") == "float": bad.append("float_call")
        if isinstance(n, (ast.Raise, ast.Assert)): bad.append("raise_or_assert")
        if isinstance(n, ast.Import) and any(a.name not in ALLOWED_IMPORTS for a in n.names): bad.append("bad_import")
        if isinstance(n, ast.ImportFrom) and n.module not in ALLOWED_IMPORTS: bad.append("bad_import_from")
    return bad

def run_suite(m, src):
    f = ["LINT:" + b for b in lint(src)]
    try:
        # Profile pin
        prof = json.load(open(os.path.join(HERE, "profile.json")))
        cur  = {k: v for k, v in vars(m).items() if k.isupper() and isinstance(v, int)}
        if cur != prof: f.append("PROFILE_PIN")

        # --- Anchors ---
        def mk(**kw):
            base = dict(dt=W,ax=0,ay=0,az=0,wx=0,wy=0,wz=0,qw=W,qx=0,qy=0,qz=0,px=0,py=0,pz=0,vx=0,vy=0,vz=0)
            base.update(kw); return tuple(base.values())

        ai = m.audit(mk())
        if ai["qw"]!=W or ai["qx"]!=0 or ai["qy"]!=0 or ai["qz"]!=0: f.append("ANCHOR_Q_IDENTITY")
        if ai["g_load"]!=0:   f.append("ANCHOR_G_ZERO")
        if ai["aoa_cos"]!=W:  f.append("ANCHOR_AOA_ZERO_V")
        if ai["flags"]!=0:    f.append("ANCHOR_FLAGS")
        if ai["action"]!=0:   f.append("ANCHOR_ACTION")
        if ai["conv"]!=1:     f.append("ANCHOR_CONV")
        if ai["px"]!=0 or ai["py"]!=0 or ai["pz"]!=0: f.append("ANCHOR_POS")

        a1g = m.audit(mk(az=-G))
        if a1g["g_load"] != W: f.append("ANCHOR_1G")

        afwd = m.audit(mk(vx=W))
        if afwd["aoa_cos"] != W: f.append("ANCHOR_AOA_FWD")

        # --- G-load boundary ---
        def tg(az): return mk(az=az)
        a9  = m.audit(tg(9*G));   a9x = m.audit(tg(9*G+10))
        if a9["g_load"]  != 9*W:                      f.append("GLOAD_9G_VALUE")
        if a9["flags"] & m.F_GLOAD:                   f.append("GLOAD_9G_NOFLAG")
        if not (a9x["flags"] & m.F_GLOAD):            f.append("GLOAD_BOUNDARY_FLAG")
        if a9x["g_load"] != 9*W + 1:                  f.append("GLOAD_BOUNDARY_VAL")
        if m.audit(tg(9*G+9))["flags"] & m.F_GLOAD:  f.append("GLOAD_9G9_NOFLAG")

        # --- Speed boundary ---
        def tv(vx): return mk(vx=vx)
        if m.audit(tv(SM))["flags"] & m.F_SPEED:     f.append("SPEED_MAX_NOFLAG")
        if not m.audit(tv(SM+1))["flags"] & m.F_SPEED: f.append("SPEED_MAX1_FLAG")

        # --- AoA boundary ---
        if m.audit(mk(vx=3*W, vy=4*W))["flags"] & m.F_AOA:    f.append("AOA_3_4_NOFLAG")
        if not m.audit(mk(vx=3*W-1, vy=4*W))["flags"] & m.F_AOA: f.append("AOA_3M1_4_FLAG")
        if m.audit(mk(vx=W))["flags"] & m.F_AOA:                 f.append("AOA_FWD_NOFLAG")

        # --- body_x exact cases ---
        if m.body_x(W,0,0,0)  != (W,0,0):   f.append("BODY_X_IDENTITY")
        if m.body_x(0,0,0,W)  != (-W,0,0):  f.append("BODY_X_180Z")
        if m.body_x(0,W,0,0)  != (W,0,0):   f.append("BODY_X_180X")  # 180° about x: x-axis unchanged
        if m.body_x(0,0,W,0)  != (-W,0,0):  f.append("BODY_X_180Y")  # 180° about y: x flips

        # --- Signed ops ---
        if m.smul(-3*W, 2*W)  != -6*W:            f.append("SMUL_NEG")
        if m.smul(0, W)        != 0:               f.append("SMUL_ZERO")
        if m.sdiv(-7*W, 2*W)  != -35*10**17:      f.append("SDIV_NEG")
        if m.sdiv(W, 0)        != 0:               f.append("SDIV_ZERO")
        if m.half_smul(W, W)   != W//2:            f.append("HALF_SMUL")
        if m.half_smul(-W, W)  != -(W//2):         f.append("HALF_SMUL_NEG")
        if m.half_smul(-W, -W) != W//2:            f.append("HALF_SMUL_NEG2")

        # --- Clamp ---
        if m.clamp_inputs((-10**31,)*17) != (-10**30,)*17: f.append("CLAMP_NEG")
        if m.clamp_inputs((10**31,)*17)  != (10**30,)*17:  f.append("CLAMP_POS")

        # --- Degenerate q→identity ---
        adq = m.audit(mk(qw=0,qx=0,qy=0,qz=0))
        if adq["conv"]   != 1: f.append("DEGEN_Q_CONV")
        if adq["qw"]     != W: f.append("DEGEN_Q_W")

        # --- dt=0: no kinematic change ---
        a0 = m.audit(mk(dt=0,ax=G,vx=W))
        if a0["vx"] != W or a0["px"] != 0: f.append("DT_ZERO")

        # --- Quaternion norm after refine always within tol ---
        vecs = json.load(open(os.path.join(HERE, "vectors.json")))
        for i, v in enumerate(vecs):
            t = v["trace"]
            norm = m.q4_norm(t["qw"],t["qx"],t["qy"],t["qz"])
            if abs(norm - W) > m.QNORM_TOL:   f.append("QNORM@%d"%i)
            if t["action"] not in (0,1,2):    f.append("ACTION_RANGE@%d"%i)
            if t["g_load"]  < 0:              f.append("GLOAD_NEG@%d"%i)
            if t["speed_sq"] < 0:             f.append("SPEED_NEG@%d"%i)
            # Flag-value consistency
            if bool(t["flags"] & m.F_GLOAD)  != bool(t["g_load"]  > m.G_MAX_WAD): f.append("FLAG_GLOAD_CONS@%d"%i)
            if bool(t["flags"] & m.F_SPEED)  != bool(t["speed_sq"] > (m.SPEED_MAX*m.SPEED_MAX)//m.WAD): f.append("FLAG_SPEED_CONS@%d"%i)
            if bool(t["flags"] & m.F_AOA)    != bool(t["aoa_cos"]  < m.AOA_COS_MIN): f.append("FLAG_AOA_CONS@%d"%i)
            if bool(t["flags"] & m.F_QNORM)  != bool(t["conv"]     == 0): f.append("FLAG_QNORM_CONS@%d"%i)
            if t["action"]==m.ACT_FAULT   and t["conv"]!=0:     f.append("ACTION_FAULT_CONV@%d"%i)
            if t["action"]==m.ACT_NOMINAL and t["flags"]!=0:    f.append("ACTION_NOM_FLAGS@%d"%i)
            if t["action"]==m.ACT_BREACH  and t["flags"]==0:    f.append("ACTION_BREACH_FLAGS@%d"%i)

        # --- Monotonicity: higher |a| → higher g_load ---
        gl1 = m.audit(mk(az=G))["g_load"];  gl2 = m.audit(mk(az=2*G))["g_load"]
        if not gl2 > gl1: f.append("MONO_GLOAD")

        # --- Full chain: trace and hash must match golden vectors ---
        res = m.run([tuple(v["tel"]) for v in vecs])
        for i, (v, (t, h)) in enumerate(zip(vecs, res)):
            if t != v["trace"]: f.append("TRACE@%d"%i)
            if h != v["hash"]:  f.append("HASH@%d"%i)

    except Exception as ex:
        f.append("EXC:%s:%s" % (type(ex).__name__, str(ex)[:40]))
    return f
