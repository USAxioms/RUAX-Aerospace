import sys, os, types
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import suite
src = open(os.path.join(HERE,"..","ref","flight.py")).read()
m   = types.ModuleType("flight"); exec(compile(src,"flight.py","exec"), m.__dict__)

# Generate profile.json if not present
import json
prof_path = os.path.join(HERE,"profile.json")
if not os.path.exists(prof_path):
    prof = {k:v for k,v in vars(m).items() if k.isupper() and isinstance(v,int)}
    json.dump(prof, open(prof_path,"w"), indent=1, sort_keys=True)
    print("profile.json written:", len(prof), "constants")

fails = suite.run_suite(m, src)
print("BASE_SUITE", "PASS" if not fails else "FAIL %s" % fails[:6])
sys.exit(1 if fails else 0)
