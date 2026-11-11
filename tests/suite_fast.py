"""Fast suite for mutation testing: audit-only checks (no full run() chain)."""
import ast, json, os
HERE = os.path.dirname(os.path.abspath(__file__))
W = 10**18; G = 9806650000000000000
ALLOWED_IMPORTS = {"hashlib", "math"}

def tel(**kw):
    b=[W,0,0,0,0,0,0,W,0,0,0,0,0,0,0,0,0]
    idx={"dt":0,"ax":1,"ay":2,"az":3,"wx":4,"wy":5,"wz":6,"qw":7,"qx":8,"qy":9,"qz":10,"px":11,"py":12,"pz":13,"vx":14,"vy":15,"vz":16}
    for k,v in kw.items(): b[idx[k]]=v
    return tuple(b)

def lint(src):
    bad = []
    for n in ast.walk(ast.parse(src)):
        if isinstance(n, ast.Constant) and isinstance(n.value, float): bad.append("float")
        if isinstance(n, ast.Div): bad.append("true_div")
        if isinstance(n, ast.Call) and getattr(n.func,"id","") == "float": bad.append("float_call")
        if isinstance(n, (ast.Raise, ast.Assert)): bad.append("raise")
        if isinstance(n, ast.Import) and any(a.name not in ALLOWED_IMPORTS for a in n.names): bad.append("import")
        if isinstance(n, ast.ImportFrom) and n.module not in ALLOWED_IMPORTS: bad.append("import")
    return bad

_VECS = None
def _vecs():
    global _VECS
    if _VECS is None:
        _VECS = json.load(open(os.path.join(HERE, "vectors.json")))
    return _VECS

def run_suite(m, src):
    f = ["LINT:" + b for b in lint(src)]
    try:
        prof = json.load(open(os.path.join(HERE, "profile.json")))
        cur  = {k:v for k,v in vars(m).items() if k.isupper() and isinstance(v,int)}
        if cur != prof: f.append("PROFILE_PIN")

        SM = m.SPEED_MAX
        a1 = m.audit(tel())
        if a1["qw"]!=W or a1["qx"]!=0 or a1["qy"]!=0 or a1["qz"]!=0: f.append("A1_Q")
        if a1["g_load"]!=0: f.append("A1_GLOAD")
        if a1["aoa_cos"]!=W: f.append("A1_AOA")
        if a1["flags"]!=0 or a1["action"]!=0: f.append("A1_FLAGS")
        if a1["conv"]!=1: f.append("A1_CONV")
        if a1["px"]!=0 or a1["vy"]!=0: f.append("A1_KIN")
        if m.audit(tel(az=-G))["g_load"] != W: f.append("A2_1G")
        if m.audit(tel(vx=W))["aoa_cos"] != W: f.append("A3_AOA")
        a9  = m.audit(tel(az=9*G))
        a9p = m.audit(tel(az=9*G+10))
        if a9["g_load"]  != 9*W:        f.append("GLOAD_9G")
        if a9["flags"] & m.F_GLOAD:     f.append("GLOAD_9G_NOFLAG")
        if not (a9p["flags"] & m.F_GLOAD): f.append("GLOAD_9Gp10")
        if a9p["g_load"] != 9*W+1:      f.append("GLOAD_9Gp10_VAL")
        if m.audit(tel(az=-9*G))["g_load"]    != 9*W:   f.append("GLOAD_NEG")
        if m.audit(tel(az=-9*G-10))["g_load"] != 9*W+1: f.append("GLOAD_NEG_B")
        if m.audit(tel(vx=SM))["flags"] & m.F_SPEED: f.append("SPEED_MAX")
        if not (m.audit(tel(vx=SM+1))["flags"] & m.F_SPEED): f.append("SPEED_MAX1")
        if not (m.audit(tel(vx=-SM-1))["flags"] & m.F_SPEED): f.append("SPEED_NEG")
        if m.audit(tel(vx=100*W,vy=32*W))["flags"] & m.F_AOA: f.append("AOA_32")
        if not (m.audit(tel(vx=100*W,vy=33*W))["flags"] & m.F_AOA): f.append("AOA_33")
        if m.body_x(W,0,0,0)!=(W,0,0):   f.append("BX_ID")
        if m.body_x(0,0,0,W)!=(-W,0,0):  f.append("BX_180Z")
        if m.half_smul(W,W)!=W//2:       f.append("HSMUL_PP")
        if m.half_smul(-W,W)!=-(W//2):   f.append("HSMUL_NP")
        if m.half_smul(W,-W)!=-(W//2):   f.append("HSMUL_PN")
        if m.half_smul(-W,-W)!=W//2:     f.append("HSMUL_NN")
        if m.half_smul(0,W)!=0:          f.append("HSMUL_0")
        if (m.smul(-3*W,2*W),m.sdiv(-7*W,2*W),m.sdiv(W,0)) != (-6*W,-35*10**17,0): f.append("SIGN_OPS")
        a0 = m.audit(tel(dt=0,az=-G))
        if a0["vz"]!=0 or a0["pz"]!=0:   f.append("DT_ZERO")
        if m.audit(tel(ax=W))["vx"]!=W:  f.append("EULER_VX")
        if not (m.audit(tel(az=2*G))["g_load"] > m.audit(tel(az=G))["g_load"]): f.append("MONO")
        if m.clamp_inputs((-10**31,10**31))!=(-10**30,10**30): f.append("CLAMP")
        # AoA and g-load flag consistency over ALL audit traces (fast: no run chain)
        for i,v in enumerate(_vecs()):
            t = m.audit(tuple(v["tel"]))
            if bool(t["aoa_cos"] < m.AOA_COS_MIN) != bool(t["flags"] & m.F_AOA): f.append("AOA_C@%d"%i)
            if bool(t["g_load"]  > m.G_MAX_WAD)   != bool(t["flags"] & m.F_GLOAD): f.append("GL_C@%d"%i)
            if t["action"] not in (0,1,2):   f.append("ACT@%d"%i)
            if t["conv"]   not in (0,1):     f.append("CONV@%d"%i)
            if t["g_load"]  < 0:             f.append("GNEG@%d"%i)
            if t["speed_sq"]< 0:             f.append("SNEG@%d"%i)
            qn = m.q4_norm(t["qw"],t["qx"],t["qy"],t["qz"])
            if abs(qn-m.WAD) > m.QNORM_TOL: f.append("QNORM@%d"%i)
        # Hash chain: spot-check first 30 vectors
        res = m.run([tuple(v["tel"]) for v in _vecs()[:30]])
        for i,(v,(tr,h)) in enumerate(zip(_vecs()[:30],res)):
            if tr!=v["trace"]: f.append("TR30@%d"%i)
            if h !=v["hash"]:  f.append("H30@%d"%i)
    except Exception as ex:
        f.append("EXC:%s:%s"%(type(ex).__name__,str(ex)[:30]))
    return f
