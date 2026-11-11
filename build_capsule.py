"""Deterministic ZIP builder: sorted entries, fixed timestamps, fixed permissions. Writes MANIFEST.sha256 first."""
import hashlib, os, zipfile, sys
ROOT = os.path.dirname(os.path.abspath(__file__))
OUT = sys.argv[1] if len(sys.argv) > 1 else os.path.join(ROOT, "..", "wad18-aero-flight-capsule-v0.1.0.zip")
files = []
for d, dirs, fs in os.walk(ROOT):
    dirs[:] = sorted(x for x in dirs if x != "__pycache__")
    for f in sorted(fs):
        if f in ("MANIFEST.sha256",) or f.endswith(".pyc") or f.endswith(".zip"):
            continue
        files.append(os.path.relpath(os.path.join(d, f), ROOT))
files.sort()
manifest = "".join("%s  %s\n" % (hashlib.sha256(open(os.path.join(ROOT, p), "rb").read()).hexdigest(), p) for p in files)
open(os.path.join(ROOT, "MANIFEST.sha256"), "w").write(manifest)
with zipfile.ZipFile(OUT, "w", zipfile.ZIP_DEFLATED) as z:
    for p in files + ["MANIFEST.sha256"]:
        zi = zipfile.ZipInfo(p, (1980, 1, 1, 0, 0, 0)); zi.external_attr = 0o644 << 16
        zi.compress_type = zipfile.ZIP_DEFLATED
        z.writestr(zi, open(os.path.join(ROOT, p), "rb").read())
print("built", len(files) + 1, "files")
