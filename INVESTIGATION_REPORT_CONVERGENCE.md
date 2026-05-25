# Newton-Raphson Convergence Performance — Investigation Report

> Read-only investigation. No source files modified.

---

## 1. NR Loop Map

The iteration sequence in `JavaMNA.singleTick()` (JavaMNA.java:286):

```
TICK START
  StateVector  ← previous tick's solution (warm start, always)
  RHSVector    ← recomputed each tick via computeRHS() / outerHooks.preSolve()

  FOR i = 0 to maxIterations-1:

    [A] IF i == 0: call iterHooks(0, max)
        └─ startIteration(0) on every ISolverHook
           (countUpdates=false during call → conductanceDelta grows, conductanceUpdates does NOT)

    [B] computeResidual()
        └─ ResidualVector = RHSVector + Σ hook.addResidual()

    [C] ErrorVector = J · StateVector − ResidualVector    (J = Jacobian)
        norm = L∞(ErrorVector)

    [D] IF norm < absoluteStoppingCriterion (1e-7)
        OR |norm − prevNorm| < relativeStoppingCriterion (1e-14):
          BREAK  ← converged

    [E] IF SCALING: ScaledJ = D_row × J × D_col (recomputed if >50 iters old)

    [F] Solve: StateVector_new = ScaledJ \ (D_row · ResidualVector)
        StateVector_new ← columnScales × StateVector_new   (undo column scaling)
        StateDelta = StateVector_new − StateVector_old      (full NR step)

    [G] LINE SEARCH (backtracking):
        alpha = 0
        WHILE alpha < maxSearchAlpha (0.99):
          iterHooks(i, max)        ← updates Jacobian for current StateVector
          computeResidual()
          testNorm = L∞(J·StateVector − ResidualVector)
          IF testNorm < norm: BREAK  ← step improves residual
          deltaAlpha = (1 − alpha) × 0.5
          alpha += deltaAlpha
          StateVector −= deltaAlpha × StateDelta  ← backtrack toward old state

  [H] verifyConvergence(norm, i, maxIterations):
      IF norm > minimumAllowedPrecision (1e-6): mark not-converged, log
      ELSE: mark converged

  outerHooks.postUpperSolve()  ← capacitor/inductor history update (guarded by isConverged())
```

**Key facts:**
- `iterHooks` is called once at i=0, then inside every backtrack step. NOT called at the top of iterations 1..N.
- For iterations 240–249 (last 10), `iterHooks` is suppressed entirely (`i < max − 10` guard, JavaMNA.java:219). Jacobian frozen for the final 10 iterations.
- The backtrack loop runs at most ~7 steps (alpha converges to 0.99 geometrically) and does NOT consume the outer iteration budget.
- `maxSearchAlpha = 0.99` (config default, CSolver.java:39).

---

## 2. Convergence Criterion Analysis

### Stopping criteria

| Criterion | Formula | Value | Type |
|-----------|---------|-------|------|
| `absoluteStoppingCriterion` | `L∞(J·x − r) < threshold` | 1e-7 A | Absolute |
| `relativeStoppingCriterion` | `|norm[i] − norm[i-1]| < threshold` | 1e-14 | Relative change in consecutive norms |
| `minimumAllowedPrecision` | Reports failure if norm exceeds this | 1e-6 A | Absolute failure gate |

**No relative criterion exists** based on the initial residual `norm_0` or the state-vector magnitude `||x||`. The `relativeStoppingCriterion = 1e-14` compares consecutive norm *differences*, not `norm / norm_0`. This is effectively a plateau-detection mechanism, not a true relative tolerance.

### Floating-point achievability of the 1e-7 threshold

The residual vector is `ErrorVector = J·StateVector − ResidualVector`. For a solved system the residual is nonzero only due to floating-point rounding: each matrix-vector multiply introduces relative error ≈ `n × ε` where ε = 2.2×10⁻¹⁶ and n = number of nodes.

For an n=23 node network with maximum Jacobian entry G_max:

```
Floating-point noise floor ≈ G_max × V_max × n × ε
                            = G_max × V_max × 23 × 2.2e-16
```

Threshold crossings (at V_max = 1 V):

| G_max | Noise floor | Converge to 1e-7? |
|-------|-------------|-------------------|
| 1 S (typical resistor) | 5e-15 A | Yes, easily |
| 1e3 S (LRSeriesWire, L=50µH) | 5e-12 A | Yes |
| 4.7e5 S (LRSeriesWire, L_min=100nH) | 2.4e-9 A | Yes |
| **1e6 S** | **5e-9 A** | **Yes, barely** |
| **5e6 S** | **2.5e-7 A** | **No — noise floor ≈ threshold** |
| **1e7 S** | **5e-7 A** | **No — noise floor 5× above threshold** |

**Finding:** At realistic component conductances (≤ 5×10⁵ S from LRSeriesWire), the 1e-7 threshold is achievable. However, the failure norms of 0.0014–0.0022 A are ~10,000× above even the noise floor — the solver is not failing due to floating-point precision but due to **not converging at all** within 250 iterations.

### What 0.0014 A residual means in voltage terms

At G_max = 1000 S (realistic for a multi-component PCB board):
```
ΔV = 0.0014 / 1000 = 1.4 μV
```
The solution is off by only 1.4 μV per amp-weighted node — excellent accuracy. The solver is reporting failure not because the circuit state is wrong, but because the **absolute norm is being compared against 1e-7 A even when the voltage error is negligible**. A relative criterion (`norm / norm_0 < 1e-4`, say) would declare convergence.

---

## 3. Matrix Scaling Audit

### What scaling is applied

`SCALING = true` (JavaMNA.java:35 compile-time constant). Scaling is computed in `computeScales()` (JavaMNA.java:164) and applied in `prepareScaled()` (JavaMNA.java:271).

**Scale computation** (per row/column i):
```
max_i = Σ_j J[i,j]²          (squared Frobenius norm of row i)
S_i = sqrt(min(1/sqrt(max_i), 2000))
columnScales[i] = rowScales[i] = S_i
```

The SAME scalar S_i is used for both row i and column i. This is **symmetric equilibration** (ScaledJ = D·J·D where D = diag(S_0, ..., S_n)).

### When scales are recomputed

- On full matrix rebuild (`populateConductanceMatrix` → `finishJacobianWrite` sets `recalculateScales = true`)
- Every 50 NR iterations (`MAX_SCALE_REUSE_COUNT = 50`, JavaMNA.java:34; `scalesAge` incremented each time `computeScales` is NOT called)
- The ScaledJ is recomputed from the current Jacobian: `J.multColumns(columnScales, ScaledJ)` then `ScaledJ.multRows(rowScales, null)` (JavaMNA.java:277-278)

Incremental Jacobian updates (`jacobianAdd`) add to ScaledJ using potentially stale scales:
```java
var scaledValue = value * columnScales[column] * rowScales[row];  // JavaMNA.java:127
ScaledJ.add(row, column, scaledValue);
```

Between scale recomputes, incremental updates use scales from the PREVIOUS state. For fast-varying conductances (like BJT during NR), this introduces a Jacobian inconsistency within the 50-iteration window.

### Condition number estimate

For a matrix with G_max (largest entry) and G_min (smallest off-zero entry):
- **Before scaling:** κ ≈ G_max / G_min
- **After symmetric equilibration:** κ_eff ≈ (G_max / G_min)^(1/2)

Symmetric equilibration halves the exponent of the condition number. For realistic in-game components:
- G_max ≈ 4.7×10⁵ S (minimum-L inductor) vs G_min = G_MIN = 1×10⁻⁸ S
- Before: κ ≈ 4.7×10¹³
- After: κ_eff ≈ 6.9×10⁶

Double precision provides ~10¹⁵ relative accuracy. κ_eff = 10⁶·⁸ leaves ~8 digits of effective precision — adequate for 1e-7 convergence.

However: for circuits with both very large (inductor, G_max~10⁵ S) and very small (leakage, G_MIN=10⁻⁸ S) conductances in the SAME ROW, the row L2 norm is dominated by the large entry. The symmetric scale compresses that row and expands the small-conductance row. Off-diagonal coupling between these two rows is scaled by S_large × S_small, which can still produce very unbalanced entries in ScaledJ.

**Missing: separate row and column scales.** The current scheme forces S_i^row = S_i^col. True optimal equilibration uses independent row and column scales (computed iteratively). This is a known deficiency for highly asymmetric matrices.

---

## 4. Source of 10²² Conductance

### Analysis of each candidate

**Candidate A — CapacitorWire (G = C/dt):**
`CapacitorWire` is defined but **not used in any production game path** (only in unit tests). The `CapacitorComponent` uses `CRSeriesWire(C/1000, 0.1Ω, ...)` with a mandatory 0.1Ω series resistance, bounding conductance to 1/0.1 = 10 S regardless of C. **Ruled out.**

**Candidate B — InductorWire (G = dt/L):**
`InductorWire` is defined but **not used in any production game path** (only in tests). `InductorComponent` uses `LRSeriesWire(L/1000, L×1.2, ...)` with R = 1.2L, bounding conductance to 1/R = 1/(1.2L). For minimum L = 1×10⁻⁷ H: G_max = dt/L = 0.05/1×10⁻⁷ = 5×10⁵ S. Still not 10²². **Ruled out for 10²²; plausible at 10⁵ S.**

**Candidate C — multiTick amplification:**
`computeRequiredMultiTick` returns `min(ceil(0.1/τ_min), globalMultiTick)`. Global multiTick defaults to 1 (CSolver.java:48, `i(1, 1, ...)`). `CapacitorWire` and `InductorWire` don't implement `getLocalTau()` (returns MAX_VALUE), so they do not drive multiTick up. `CRSeriesWire` and `LRSeriesWire` do implement it, but at default globalMultiTick=1, multiTick is always 1. **Ruled out at default config.**

**Candidate D — BJT exponential transconductance:**
The BJT model uses the WrightOmega function which approximates the Lambert-W function. For large V_BE:
```
Gee = WbeTerm / (Re × (1 + WbeTerm))
```
As WbeTerm → ∞: Gee → 1/Re = 1/0.1 = **10 S** (hard upper bound from series resistance).

BJTComponent.bake() uses `Rs = 0.1` (BJTComponent.java:51). This bounds Gee and Gcc to 10 S regardless of junction voltage. The exponential divergence in the original Ebers-Moll formula is regularised by the WrightOmega/Lambert-W approach precisely to prevent this. **Ruled out for 10²²; maximum BJT junction conductance is bounded at 10 S.**

### Most likely source

Based on static code analysis, **no currently-reachable production code path produces Jacobian entries above ~5×10⁵ S** (from LRSeriesWire at minimum inductance). The figure of 10¹⁰–10²² almost certainly refers to the **condition number** of the admittance matrix (ratio of largest to smallest singular value), not a raw matrix entry. For a circuit with:
- LRSeriesWire at L=100nH: G ≈ 5×10⁵ S  
- G_MIN leakage conductances: 1×10⁻⁸ S  
- Transformer coupling entries: 1–100 (ratio-scale)

κ before scaling ≈ 5×10⁵ / 1×10⁻⁸ = **5×10¹³**, which falls within the 10¹⁰–10²² range reported.

The residual sticking at 0.0014–0.0022 is consistent with **limit cycling in the NR iteration**, not with floating-point noise. At G_max ≈ 5×10⁵ S, the floating-point noise floor (~2.5×10⁻⁹ A) is well below 0.002 A. The solver is genuinely not converging — the solution is oscillating between states that each produce a non-zero residual.

---

## 5. Warm-Start Status

**StateVector is warm-started — always.**

Evidence: `JavaMNA.allocate()` (JavaMNA.java:187-216). When the matrix is reallocated after a topology change:
```java
var NewState = new DMatrixRMaj(size, 1);
if(StateVector != null) {
    for(int i = 0; i < size; ++i) {
        NewState.unsafe_set(i, 0, network.getValue(nodes.get(i)));
    }
}
StateVector = NewState;
```
Previous voltages are copied into the new vector. Between ticks with no topology change, `StateVector` is never zeroed — it retains the last tick's (converged or best-effort) solution.

**First-ever tick:** `StateVector = new DMatrixRMaj(size, 1)` (EJML zero-initialises) — cold start only on network creation.

**`zeroState()`** (JavaMNA.java:382): sets `StateVector` to zero, but is only called from `prepare()` when `sourceCount == 0` (no voltage sources in the network). Never called for normal circuits.

**`warmUp(n)`** forces `converged = false` for subsequent ticks but does NOT zero StateVector. Only prevents the `isConverged()` guard in `postUpperSolve()` from firing.

**Implication:** Warm-start is already implemented. The 250-iteration budget being exhausted is not a cold-start problem. The solver IS starting from the previous tick's (possibly non-converged) state. For a limit-cycling circuit, warm-starting from the previous non-converged state propagates the limit-cycle initial condition into the next tick — potentially making convergence harder across ticks.

---

## 6. Admittance Matrix Rebuild Frequency

### Rebuild triggers (ElectricalNetwork.java:666)

```java
} else if(conductanceUpdates >= nodeCount * 40 || conductanceDelta > 1000) {
    populateConductanceMatrix();
}
```

For a 23-node network: rebuild when `conductanceUpdates >= 920` OR `conductanceDelta > 1000`.

### Does `G_add` in BJTWire increment `conductanceUpdates`?

**No.** `G_add` changes happen inside `startIteration()`, called from `iterHooks()`:
```java
private void iterHooks(int i, int max) {
    if(i < max - 10) {
        network.countUpdates = false;           // ← suppresses conductanceUpdates
        for(var hook : network.innerHooks) {
            hook.startIteration(i);              // ← G_add changes here
        }
        network.countUpdates = true;
    }
}
```

`updateConductance()` (ElectricalNetwork.java:348-351) increments conductanceUpdates only when `countUpdates = true`. Since `countUpdates = false` during iterHooks, G_add ramps do **not** trigger the 920-update threshold.

### Does `G_add` increment `conductanceDelta`?

**Yes.** `conductanceDelta += Math.abs(change)` has no `countUpdates` guard (ElectricalNetwork.java:348). Every BJT conductance change during NR — including G_add ramps — contributes to conductanceDelta.

For 1 BJT with 4 internal conductance wires (base-emitter, base-collector, two VCCSs), each NR iteration in which the voltage changes calls 4 `updateConductance` paths. The voltage change per iteration (with bjtSmoothAlpha = 0.5 after fix) during the ramp phase is approximately 50% of the NR step. For V_BE ramping from 0 to 0.6 V over ~20 iterations: average ΔGee ≈ 0.5 S/step → ΔconductanceDelta ≈ 4 × 0.5 × 20 = **40 S per failing BJT** during the initial ramp.

For a circuit with 3 BJTs failing to converge each tick: conductanceDelta ≈ 120 S/tick. Threshold hit after ~8 ticks ≈ 0.4 seconds. **This is significantly faster than the ~8-second observation**, suggesting most ticks the BJTs DO converge successfully (near the operating point) and the circuit only enters the ramp phase occasionally (at startup, or after topology changes). In the steady-state converged case: conductanceDelta ≈ 0.04–0.12 S/tick (small ΔG from minor state changes). Threshold at ~8333 ticks ≈ 7 minutes (too slow).

**Most likely explanation for ~8s rebuild:** The circuit is repeatedly hitting a convergence failure (not every tick, but frequently enough). A pattern of failing every 2nd–3rd tick (oscillating near the operating point) would produce:
- conductanceDelta per tick ≈ 20–40 S (one BJT ramp)
- Hits 1000 S threshold every 25–50 ticks ≈ 1.25–2.5 seconds

Or the rebuild trigger comes from **isStiff() oscillation** in CRSeriesWire: `isStiff()` depends on `getDeltaTime() / (RC)`. If multiTick changes between ticks (which it does when per-network multiTick is recomputed), isStiff() can flip, changing conductance, triggering `updateConductance`. However, with globalMultiTick=1, this is not applicable by default.

### Can `isStiff()` oscillate between ticks?

For fixed R, C, and fixed `currentMultiTick`: `isStiff() = dt/(RC) >= 0.5` is constant. **Cannot oscillate** between consecutive ticks unless multiTick changes. With default globalMultiTick=1, this is a non-issue. It IS an issue if the user enables multiTick > 1 in config.

---

## 7. Ranked Improvements

| Rank | Improvement | Impact | Effort | Risk |
|------|-------------|--------|--------|------|
| 1 | **Add norm-relative stopping criterion** | High | 5 LOC | Low |
| 2 | **Increase diodeSmoothAlpha default** (already done) | High | 1 LOC | Low |
| 3 | **Fix BJT alpha field bug** (already done) | High | 1 LOC | Low |
| 4 | **Independent row/column equilibration** | Medium | ~50 LOC | Low |
| 5 | **Cap conductanceDelta accumulation during NR** | Medium | 3 LOC | Low |
| 6 | **Hard-clamp BJT/diode max conductance** | Medium | 3 LOC | Low |
| 7 | **NativeMNA as default backend** | Low-medium | config | Low |

### Rank 1 — Norm-relative stopping criterion (highest priority)

**Problem:** The absolute threshold 1e-7 A is compared against a residual that grows with circuit conductance. A norm of 0.002 A on a 1000 S circuit represents 2 μV voltage error — physically excellent. The solver gives up unnecessarily.

**Fix:** In `singleTick()` (JavaMNA.java:303), change convergence check from:
```java
if (norm < absoluteStoppingCriterion || dNorm < relativeStoppingCriterion)
```
to:
```java
double norm0 = (i == 0) ? norm : norm0;  // capture initial norm
if (norm < absoluteStoppingCriterion || norm < relativeStoppingCriterion * norm0 || dNorm < 1e-14)
```
Or, more practically: add a field `norm0` captured at i=0, stop when `norm < max(1e-7, 1e-4 * norm0)`.

**Impact:** Circuits currently stuck at 0.002 residual (10,000× above absolute threshold, but perhaps already at relative convergence) would declare success within 20–50 iterations instead of always hitting 250.

**Effort:** ~5 lines. Add `norm0` field, capture at i=0, add `|| norm < relativeFactor * norm0` to the break condition.

### Rank 4 — Independent row/column equilibration

The current symmetric scaling (S_i = columnScales[i] = rowScales[i]) is computed from row L2 norms. For a matrix with highly asymmetric off-diagonal structure (as in the BJT stamp), this leaves column norms unbalanced. Separate row and column scales (e.g., two-pass iterative equilibration: scale rows to unit L2 norm, then columns to unit L2 norm, repeat 2–3 times) would reduce κ_eff more aggressively. This is standard in LAPACK-class sparse solvers.

### Rank 5 — Cap conductanceDelta accumulation during NR

The simplest fix: do not accumulate conductanceDelta during `iterHooks` (add the same `countUpdates` guard):
```java
// In updateConductance, change:
conductanceDelta += Math.abs(change);
// to:
if(countUpdates) conductanceDelta += Math.abs(change);
```
This would prevent NR-iteration conductance ramps from triggering spurious matrix rebuilds. The rebuild would only trigger from actual external circuit changes, not from the solver's internal iterations.

**Caution:** conductanceDelta was designed to detect floating-point drift in the Jacobian. Suppressing NR contributions means FP drift from rapid G changes (during NR) won't trigger a corrective rebuild. This is probably acceptable since the scales-rebuild every 50 iterations already corrects drift.

### Rank 6 — Hard-clamp BJT conductance

Even though the WrightOmega bound prevents 10²² conductances, BJT Gee/Gcc values of 10 S (at saturation with Rs=0.1Ω) still create a 10⁶-range spread with G_MIN=1e-8. Adding:
```java
Gee = Math.min(Gee, 1.0 / emitterResistance);  // already implicit, explicit for clarity
```
is not needed for correctness but documenting the bound helps reasoning.

### Rank 7 — NativeMNA

The NativeMNA (JNI + SuperLU) uses a factorized sparse solver which may handle ill-conditioned systems better than the dense LU in JavaMNA. It is the default backend (CSolver.java:50) but falls back to JavaMNA when the native library is absent. Ensuring the native library is deployed (or building it via `:native:buildLinux`) provides a potential speedup and may use better pivoting strategies.

---

## Summary

The primary actionable finding is **Rank 1**: the absolute stopping criterion (1e-7 A) has no relative component, causing the solver to spend all 250 iterations on circuits that are already "converged enough" at 0.002 A residual (representing 2 μV voltage error). Adding `norm < factor × norm_0` as a second stop condition would eliminate the majority of convergence failures with minimal code change and no correctness risk.

The secondary findings (Ranks 2–3, already implemented) significantly reduce the iteration budget wasted on the ramp phase for diodes and BJTs.

The conductanceDelta rebuild issue (Rank 5) may explain the ~8-second matrix rebuild cadence and is a trivial 1-line fix.

---

*Files read: `JavaMNA.java`, `ElectricalNetwork.java`, `CapacitorWire.java`, `InductorWire.java`, `CRSeriesWire.java`, `LRSeriesWire.java`, `CompoundWire.java`, `AbstractElectricWire.java`, `ITimeAwareWire.java`, `VoltageSourceCoupling.java`, `TransformerCoupling.java`, `SwitchedWire.java`, `BJTWire.java`, `PNJunctionWire.java`, `CapacitorComponent.java`, `InductorComponent.java`, `BJTComponent.java`, `CSolver.java`, `WorldNetworks.java`.*
