#!/bin/sh
set -e
python3 ledger/csl.py
python3 tests/gen_vectors.py
python3 tests/run_tests.py
python3 tests/mutants.py
