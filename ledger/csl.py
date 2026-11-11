"""Cognitive State Ledger: append-only, SHA-256 hash-chained JSONL. No floats, no wall clock."""
import hashlib, json, sys, os
PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "CSL.jsonl")
GENESIS = "0" * 64

def _last():
    if not os.path.exists(PATH):
        return 0, GENESIS
    last = None
    with open(PATH, encoding="utf-8") as f:
        for line in f:
            if line.strip():
                last = json.loads(line)
    return (last["seq"], last["hash"]) if last else (0, GENESIS)

def append(kind, date, summary, details):
    seq, prev = _last()
    body = {"seq": seq + 1, "date": date, "kind": kind, "summary": summary,
            "details": details, "prev": prev}
    blob = json.dumps(body, sort_keys=True, separators=(",", ":")).encode()
    body["hash"] = hashlib.sha256(blob).hexdigest()
    with open(PATH, "a", encoding="utf-8") as f:
        f.write(json.dumps(body, sort_keys=True, separators=(",", ":")) + "\n")
    return body["seq"], body["hash"]

def verify():
    prev = GENESIS
    n = 0
    with open(PATH, encoding="utf-8") as f:
        for line in f:
            if not line.strip():
                continue
            e = json.loads(line)
            h = e.pop("hash")
            if e["prev"] != prev:
                return False, n
            blob = json.dumps(e, sort_keys=True, separators=(",", ":")).encode()
            if hashlib.sha256(blob).hexdigest() != h:
                return False, n
            prev = h
            n += 1
    return True, n

if __name__ == "__main__":
    ok, n = verify()
    print("LEDGER_OK" if ok else "LEDGER_BROKEN", n)
    sys.exit(0 if ok else 1)
