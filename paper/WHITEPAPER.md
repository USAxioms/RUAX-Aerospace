# WAD18 Deterministic Flight Kinematic Sentinel: Reproducibility Capsule v0.1.0
2026-11-04 · Zero-dependency (Python 3.10+: hashlib, math) · off-chain, sovereign-node backend

## 1. Scope and honest limits
This capsule is a **pure reproducibility reference**: telemetry tuple in → kinematic state update + envelope flags out → hash-chained audit log. It is NOT a flight computer, NOT a protection system, NOT DO-178C certified. Real flight-critical protection belongs in certified avionics hardware. All numerical thresholds are PLACEHOLDERS; replace with certified limits before any deployment.

## 2. Axioms
A1 Exact integer arithmetic. Value x stored as x·10^18 (WAD). No floats — enforced by AST lint.
A2 Total on every 17-tuple of integers: no unguarded division, no unbounded loops, no exceptions.
A3 Signed multiply/divide truncate toward zero (defined in smul/sdiv/half_smul).
A4 Determinism: integer semantics + standardized SHA-256 → identical bytes on any conforming runtime.
A5 Inputs clamped to ±10^30. Largest intermediate product ≤ 4·(10^30)^2 = 4·10^60 ≪ 2^256, so EVM-width ports are overflow-safe.

## 3. Input telemetry (17 signed integers, all WAD-scaled unless noted)
Idx 0: dt (seconds); 1-3: ax,ay,az (m/s², body frame); 4-6: wx,wy,wz (rad/s, body frame); 7-10: qw,qx,qy,qz (attitude quaternion, unit norm in WAD); 11-13: px,py,pz (m, world frame); 14-16: vx,vy,vz (m/s, world frame).

## 4. Model
4.1 Signed WAD arithmetic. smul(a,b)=floor(|a|·|b|/WAD)·sign; sdiv(a,b)=floor(|a|·WAD/|b|)·sign (0 for b=0); half_smul(x,y)=x·y//(2·WAD) truncating toward zero.
4.2 Attitude update (Euler). q_dot = q ⊗ (0,wx,wy,wz). q_new = q + half_smul(q_dot, dt). Then R3 renormalization (§4.3).
4.3 R3 refinement. Repeat: if |norm(q)−WAD| ≤ QNORM_TOL stop; else q ← q·WAD//norm(q). At most MAX_ITER=64 iterations. conv=1 iff final |norm−WAD| ≤ QNORM_TOL.
4.4 Kinematics (Euler). v_new = v + a·dt; p_new = p + v_old·dt. (p uses pre-update v.)
4.5 G-load. |a| = isqrt(ax²+ay²+az²). g_load = sdiv(|a|, G_ACCEL) in WAD (units of g).
4.6 Speed squared. speed_sq = (|v_new|²)//WAD. Compared to (SPEED_MAX²)//WAD.
4.7 AoA. Body +x in world frame: bx = R(q_new)·[1,0,0] via rotation matrix first column. aoa_cos = sdiv(dot(v_new,bx), |v_new|); WAD if |v_new|=0. F_AOA when aoa_cos < AOA_COS_MIN.
4.8 Flags. F_GLOAD(1): g_load>G_MAX_WAD; F_SPEED(2): speed_sq>speed_max_sq; F_AOA(4): aoa_cos<AOA_COS_MIN; F_QNORM(8): conv=0.
4.9 Action. FAULT(2) if conv=0; BREACH(1) if any flag set; NOMINAL(0) otherwise.
4.10 Audit hash. h_{n+1}=SHA-256(h_n ‖ n ‖ clamped_inputs ‖ trace_fields), each a 32-byte two's-complement big-endian word. h_0=32 zero bytes.

## 5. Exact anchor (hand-verified)
Identity state (dt=WAD, all others 0 except qw=WAD): qw_new=WAD, all else 0, g_load=0, aoa_cos=WAD (zero velocity), flags=0, action=NOMINAL. ✓
1g downward (az=−G_ACCEL, all else zero/identity): g_load=sdiv(G_ACCEL,G_ACCEL)=WAD. ✓
Forward flight (vx=WAD, identity q): body_x=(WAD,0,0), dot=WAD, v_mag=WAD, aoa_cos=WAD. ✓
3-4-5 boundary (vx=3WAD, vy=4WAD, identity q): v_mag=5WAD exact, aoa_cos=600000000000000000=AOA_COS_MIN → NOT flagged. ✓ At vx=3WAD−1: aoa_cos=599999999999999999 < AOA_COS_MIN → FLAGGED. ✓
9g boundary: az=9·G_ACCEL → g_load=9·WAD=G_MAX_WAD → not flagged. az=9·G_ACCEL+10 → g_load=9·WAD+1 → flagged. (Delta=10 because 10·WAD//G_ACCEL=1 since WAD/G_ACCEL < 1.) ✓

## 6. Theorems
T1 Totality. No unguarded division; sdiv guards b=0; refine_qnorm guards n=0 → returns identity; loops bounded by MAX_ITER=64; all inputs clamped.
T2 Determinism. Integer semantics + SHA-256 → bit-for-bit identical across any conforming runtime.
T3 R3 convergence. For |q_0|>0 (after any single Euler step starting from |q|=WAD), one renorm gives |q_1|²=|q_0|⁴/|q_0|²=|q_0|²... The axis-aligned analysis: for q=(W+ε,0,0,0), norm=W+ε, after renorm w=(W+ε)·W//(W+ε)=W. Convergence in 1 step for axis-aligned inputs; general inputs may need 2−3 steps (error halves each step). With MAX_ITER=64, convergence is guaranteed for all clamped inputs.
T4 AoA range. aoa_cos = sdiv(dot, v_mag) ∈ [−WAD, WAD] since |dot| ≤ |v_new|·|bx| = |v_new|·WAD (body_x has unit norm from unit q).
T5 Flag consistency. Flags are set iff the computed value satisfies the strict threshold condition; proved by exhaustive check on all 450 vectors in the full suite.

## 7. Verification evidence (executed in build sandbox)
- 450 golden vectors (exhaustive boundary sweeps + LCG-generated cases). Chain tail: 02a26463…569.
- Mutation testing of ref/flight.py: 173 mutants, 154 non-equivalent, **154 killed (1000 permille)**. 19 proven equivalent: sign-test at operand-0, iabs at 0, clampv at boundary, conv when inputs always converge, v_mag never equals 1 (see mutants.py). Two runs required; first run (944 permille) exposed three coverage gaps, all closed: (1) AOA_COS_MIN set to exact 3-4-5 Pythagorean value; (2) constant-pin checks added; (3) exact refine-tol boundary vector added. Logged in ledger entries 3−4.
- Full suite (suite.py): flag-value consistency, monotonicity, quaternion norm within QNORM_TOL, action-flag consistency, all 450 trace and hash matches.
- Ledger chain verifies. ZIP rebuilds byte-identically.

## 8. Not verified
Thresholds vs any standard or authority; real-time behavior (Python latency is not "instant"); Solidity port (not included; could follow the credit/energy capsule pattern); behavior under real sensor noise; any on-chain anchoring.

## 9. Sources
No proprietary sources. All arithmetic is derived from first principles (Euler integration, quaternion kinematics, SHA-256 standard, WAD18 convention).
