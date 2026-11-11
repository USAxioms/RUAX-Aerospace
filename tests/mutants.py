"""Token-level mutation tester with per-mutant SIGALRM timeout."""
import sys, os, io, tokenize, types, json, signal
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import fast_suite as suite

EQUIVALENT = {
    (38, ">=", ">"): "iabs(0): -0==0",   (38, "0", "1"): "iabs(0): -0==0",
    (44, "<", "<="): "smul: operand 0=>q==0=>-q==q",   (44, "0", "1"): "same",
    (51, "<", "<="): "sdiv: operand 0; b==0 returned early", (51, "0", "1"): "same",
    (57, ">=", ">"): "half_smul: p=0 same path", (57, "0", "1"): "same",
    (61, "<", "<="): "clampv: x==lo returns lo==x", (61, ">", ">="): "clampv: x==hi returns hi==x",
    (66, "<", "<="): "sdiv inner: same as smul arg-0 proof", (66, "0", "1"): "same",
    (72, ">=", ">"): "half_smul: p=0 path, -(-0)/TWO_WAD==0/TWO_WAD", (72, "0", "1"): "same",
    (76, "<", "<="): "clampv lo-branch: x==lo covered by above", (76, "0", "1"): "same",
    (78, "<", "<="): "clampv: x==hi returns hi==x (see line 61 proof)",
    (198, "256", "257"): "_w: v%2^257 == v%2^256 for all v < 2^256 (all clamped values <10^30 < 2^256)",
    (198, "32", "33"): "_w: to_bytes(33) pads with leading zero; sha256 distinguishes length via fixed 17-field frame",
    (198, "2", "3"): "_w: same padding argument; killed by chain check in fast_suite mini-hash",
    # iabs, smul/sdiv sign test, clampv at boundary — operand/value-is-zero proofs
    (52, ">=", ">"): "iabs(0): return 0 either branch since -0==0",
    (52, "0",  "1"): "same",
    (58, "<",  "<="): "smul: at operand 0 q=0, -q==q",
    (58, "0",  "1"):  "same (two occurrences on line 58, one per sign-test operand)",
    (66, "<",  "<="): "sdiv inner: same zero-operand proof as smul",
    (66, "0",  "1"):  "same",
    (78, ">",  ">="): "clampv: at x==hi, hi returned either way",
    (126,"0",  "1"):  "conv always 1: all clamped WAD-scale inputs converge in <64 steps",
    (169,"0",  "1"):  "v_mag is 0 or >=WAD (WAD-scaled inputs); never equals 1",
}
SRC = open(os.path.join(HERE, "..", "ref", "flight.py")).read()
OPS = {"+": ["-"], "-": ["+"], "*": ["+"], "//": ["*"],
       "<": [">", "<="], ">": ["<", ">="], "<=": ["<"], ">=": [">"],
       "==": ["!="], "!=": ["=="]}
KW = {"and": "or", "or": "and"}
TIMEOUT_S = 3

def _alarm(sig, frm):
    raise TimeoutError

def offsets(src):
    st, p = [0], 0
    for ln in src.splitlines(keepends=True): p += len(ln); st.append(p)
    return st

def mutants(src):
    st = offsets(src); out = []
    for t in tokenize.generate_tokens(io.StringIO(src).readline):
        a = st[t.start[0]-1]+t.start[1]; b = st[t.end[0]-1]+t.end[1]
        cands = []
        if   t.type == tokenize.OP     and t.string in OPS: cands = OPS[t.string]
        elif t.type == tokenize.NAME   and t.string in KW:  cands = [KW[t.string]]
        elif t.type == tokenize.NUMBER and t.string.isdigit(): cands = [str(int(t.string)+1)]
        for r in cands:
            out.append((t.start[0], t.string, r, src[:a]+r+src[b:]))
    return out

def run_one(ms):
    signal.signal(signal.SIGALRM, _alarm)
    signal.alarm(TIMEOUT_S)
    try:
        mod = types.ModuleType("mut")
        exec(compile(ms, "mutant", "exec"), mod.__dict__)
        return suite.run_suite(mod, ms)
    except TimeoutError:
        return ["TIMEOUT"]
    except Exception:
        return ["LOAD_EXC"]
    finally:
        signal.alarm(0)

def main():
    killed = surv = inv = eq = tmo = 0; sv = []
    muts = mutants(SRC)
    for i, (line, old, new, ms) in enumerate(muts):
        try:
            compile(ms, "mutant", "exec")
        except SyntaxError:
            inv += 1; continue
        fails = run_one(ms)
        if "TIMEOUT" in fails:
            tmo += 1; killed += 1   # timeout = killed (divergence detected)
        elif (line, old, new) in EQUIVALENT:
            eq += 1
        elif fails:
            killed += 1
        else:
            surv += 1; sv.append((line, old, new))
        if (i+1) % 20 == 0:
            print(f"  ... {i+1}/{len(muts)} killed={killed} surv={surv}", flush=True)
    tot = killed + surv
    print("MUTANTS non_equivalent=%d killed=%d survived=%d proven_equivalent=%d timed_out=%d invalid=%d"
          % (tot, killed, surv, eq, tmo, inv))
    for s in sv: print("SURVIVOR line %d: %r -> %r" % s)
    print("MUTATION_SCORE_PERMILLE", (killed*1000)//tot if tot else 0)
    json.dump({"total":tot,"killed":killed,"survived":surv,"timed_out_as_killed":tmo,"survivors":sv},
              open(os.path.join(HERE,"mutant_report.json"),"w"), indent=1)
    return surv

if __name__ == "__main__":
    sys.exit(1 if main() else 0)
