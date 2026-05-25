# Task: Investigate Newton-Raphson Convergence Performance

**Purpose:** The PowerGrid solver is failing to converge within 250 iterations on
circuits with ~23 nodes and conductance values of ~10^10–10^22. This causes 34% TPS
deficit and chronic server lag. The final residual norms (~0.0014–0.0022) are
consistently 4 orders of magnitude above the stopping criterion (1e-7), indicating the
solver is not even close to converging when it gives up. This task is to identify every
mechanism in the solver that affects convergence rate and quality, and to assess which
are tunable, buggy, or missing.

**Files to modify:** None. Read-only investigation.
**Files NOT to modify:** All source files.

**Output:** Write `INVESTIGATION_REPORT_CONVERGENCE.md` in the repo root.

---

## Overview / Context

### Key Numbers from the Profiling Report

- **23 nodes** in the problem circuit
- **Conductance scale: ~10^10 to 10^22** — a range of 12 orders of magnitude
- **Residual norm at failure: ~0.0014–0.0022** — about 10,000× above the 1e-7 threshold
- **Admittance matrix rebuilt every ~8s** — something is continuously changing
- **250 iterations never sufficient** — systematic, not occasional

The 10^22 conductance figure is the most alarming. For reference, a 1Ω resistor has
conductance 1.0, and a 20kΩ resistor has conductance 5×10^-5. A conductance of 10^22
suggests either a near-short-circuit element, a numerical artifact, or a unit mismatch
somewhere. An ill-conditioned matrix of this scale will defeat any iterative solver
regardless of iteration count.

### Why High Conductance Destroys NR Convergence

Newton-Raphson solves `J·Δx = -r` at each iteration where J is the Jacobian (the
admittance matrix for linear networks) and r is the residual vector. The condition
number of J determines how much error amplification occurs per solve. For condition
number κ, a residual of 0.002 could correspond to a true solution error of
`0.002 × κ`. If κ ~ 10^15 (plausible given 10^22 conductance entries), the solver is
essentially working with random numbers.

The residual norm used is `elementMaxAbs(J·x - r)` — an infinity norm in **amps**.
For a high-conductance network, even a tiny voltage error produces large current
residuals. The 1e-7 threshold in amps may be orders of magnitude stricter than
necessary for voltage accuracy.

---

## Step 1 — Map the Full NR Loop

Locate `JavaMNA.singleTick()` and `JavaMNA.computeResidual()`. For each iteration of
the NR loop, record the exact sequence of operations:

1. How is the initial state vector `StateVector` populated at the start of each tick?
   Is it warm-started from the previous tick's solution, or reset to zero?
2. How is the Jacobian built? Is it rebuilt every iteration or only when conductances
   change?
3. What linear solve method is used? (LU factorization, iterative, etc.)
4. How is the step `StateDelta` applied to `StateVector`?
5. What backtracking or line search is applied? Reference the `slowdown` flag
   introduced in commit `bd98af47c` — what does it do exactly, and when does it fire?

Record the complete iteration map as pseudocode.

---

## Step 2 — Audit the Convergence Criterion

Locate `verifyConvergence()` in `JavaMNA.java` and record:

- The exact formula for `norm` — what quantity is computed, what norm is used
- The `absoluteStoppingCriterion` value (expected: 1e-7)
- The `relativeStoppingCriterion` value if present
- The `minimumAllowedPrecision` value (expected: 1e-6)
- Whether any **relative** stopping criterion exists (e.g. norm / ||x|| or norm / ||r_0||)

The profiling shows norms of 0.0014–0.0022 at failure. Compute: at what voltage error
magnitude does a 23-node network with conductance G = 10^10 S produce a current
residual of 0.002 A? The answer is `ΔV = 0.002 / 10^10 = 2×10^-13 V`. This is below
double-precision floating point resolution at voltages above ~1V. **The solver may be
asking for accuracy that is physically impossible to represent in floating point.**

Record whether any relative criterion exists that would prevent this. If none exists,
this is a significant finding.

---

## Step 3 — Audit Matrix Scaling

Locate any matrix scaling or preconditioning applied before or during the linear solve.
The `SCALING` flag seen in previous code review applies column scaling. Record:

- Is row scaling applied? (Should balance the rows of J to similar magnitude)
- Is column scaling applied? What are the `columnScales` and how are they computed?
- Is any diagonal preconditioner applied?
- Are the scaling vectors recomputed every iteration or only on matrix rebuild?

For a matrix with entries ranging from 1e-5 to 10^22, proper row/column scaling is
essential. Without it, double-precision arithmetic loses ~17 orders of magnitude of
relative accuracy, far more than the dynamic range of the matrix.

Compute the effective condition number estimate if scaling information is available:
`κ_eff ≈ max(diagonal) / min(diagonal)` after scaling.

---

## Step 4 — Identify the Source of 10^22 Conductance

A conductance of 10^22 is physically impossible for any real component. Identify what
in the codebase could produce it:

**Candidate A — BE companion model for stiff capacitors**

The Backward Euler companion conductance for a capacitor is `G = C/dt`. The adaptive
BE/BDF2 fix implemented earlier uses BE when dt/τ >= 0.5. For the stiff 1µF circuit
with R_series = 0.1Ω:

```
τ = R_series × C = 0.1 × 1e-6 = 100 ns
G_BE = C/dt = 1e-6 / 0.05 = 2×10^-5 S
```

That's only 2×10^-5 S — not problematic.

But: does the in-game capacitor component (via `CapacitorWire`) have `R_series = 0`?
If so and if `CapacitorWire.conductance()` returns `C/dt` with dt computed from
`currentMultiTick`, and if something drives `currentMultiTick` to a very high value,
`G_BE = C / (dt/multiTick)` could become enormous.

**Candidate B — InductorWire BE companion for small inductances**

BE companion conductance for an inductor is `G = dt/L`. For a very small inductance
(e.g. L = 1µH) at default dt = 0.05s:

```
G_BE = dt/L = 0.05 / 1e-6 = 50,000 S
```

For L = 1nH: G_BE = 5×10^7 S. Still not 10^22, but the trend is clear.

**Candidate C — Per-network multiTick amplification**

`computeRequiredMultiTick()` computes `ceil(0.1 / τ_min)`. For `τ_min = 100ns`:
`required = 1,000,000`. The config cap defaults to 1, but if the config has been
changed to a higher value (or if the cap was accidentally removed), the effective
`dt = 0.05 / 1,000,000 = 5×10^-8 s` and:

```
G_BE = C/dt_effective = 1e-6 / 5e-8 = 20 S   (still not 10^22)
```

**Candidate D — BJT exponential transconductance under extreme voltages**

The BJT stamps exponential transconductances. Under extreme node voltages from a
previous non-converged solution used as initial state, `exp(V_BE / V_T)` with
V_BE = 1V gives `exp(40) ≈ 2.4×10^17`. Even modest overshoots could produce
astronomically large Jacobian entries. With `V_T = 0.025V`:

```
g_m = I_S/V_T * exp(V_BE/V_T)
For V_BE = 1.3V: g_m = 1e-14/0.025 * exp(52) ≈ 4×10^-13 * 3.8×10^22 ≈ 10^10 S
For V_BE = 1.5V: g_m ≈ 10^22 S
```

This is the most likely source. Record whether `pnLim()` (the voltage limiter added
in the BJT fix) successfully prevents V_BE from reaching 1.5V, or whether there is a
path where the limiter is bypassed or ineffective for this conductance scale.

For each candidate: check whether it could plausibly produce the 10^22 figure given
the mod's component value ranges. Record the most likely source with the formula and
parameter values that produce it.

---

## Step 5 — Audit the Warm-Start Strategy

Locate how `StateVector` is initialised at the start of each tick. Options:

- **Zero initialisation**: StateVector = 0 every tick. Wastes all prior computation.
- **Warm start from previous solution**: StateVector retains the previous tick's
  converged (or best-effort) solution. This is the most important single improvement
  for slow-varying circuits.
- **DC operating point**: StateVector is set to the DC solution for the current
  sources. Good initial guess but requires an extra solve.

If the solver uses zero initialisation, warm-starting alone could reduce the number of
NR iterations from 250 to 2–5 for slowly-varying circuits.

Record exactly what value `StateVector` contains at the start of `singleTick()` on
the first iteration. Does it retain the previous solution or is it reset?

---

## Step 6 — Audit the Admittance Matrix Rebuild Trigger

The profiling shows the admittance matrix is rebuilt every ~8 seconds. Locate the
trigger conditions in `prepareMatrices()`:

```java
conductanceUpdates >= nodeCount*40 || conductanceDelta > 1000
```

Record the exact values of `conductanceUpdates` and `conductanceDelta` thresholds.
For a 23-node network: `nodeCount*40 = 920`. If conductances are being updated
frequently (e.g. from the adaptive BE/BDF2 switching its G value each tick), this
counter could be advancing rapidly and triggering unnecessary full rebuilds.

Specifically: after the adaptive BE/BDF2 changes, does `isStiff()` in `CRSeriesWire`
ever change value between ticks? If the circuit oscillates across the dt/τ = 0.5
threshold, `conductance()` returns different values on consecutive ticks, each
calling `updateConductance()` and incrementing `conductanceUpdates`. Record whether
this is possible given `isStiff()` only depends on fixed parameters (R, C, dt).

Also check: does `G_add` in `BJTWire` call `updateConductance()` as it ramps up
across iterations? If so, every BJT iteration that changes `G_add` increments the
counter, and a circuit with multiple BJTs failing to converge could hit the 920
threshold in a single tick.

---

## Step 7 — Identify Quick Wins

Based on the above findings, identify and rank improvements by expected impact.
For each candidate improvement, assess:
- **Impact**: how much would it reduce iterations or improve convergence?
- **Effort**: lines of code to implement
- **Risk**: could it break existing correct behavior?

Candidates to evaluate (add others found during investigation):

1. **Relative stopping criterion** — stop when `norm / norm_0 < 1e-6` instead of
   `norm < 1e-7`. Prevents demanding sub-floating-point accuracy on high-conductance
   networks.
2. **Warm-start StateVector** — retain previous solution instead of zeroing.
3. **Improved matrix scaling** — balanced row/column scaling to reduce condition
   number before the linear solve.
4. **Clamp BJT exponential** — hard-limit `exp(V_BE/V_T)` to prevent 10^22 Jacobian
   entries from entering the matrix regardless of `pnLim()` state.
5. **Stabilise `isStiff()` result** — prevent `conductance()` from changing value
   between ticks, eliminating spurious matrix rebuilds.
6. **NativeMNA availability** — is the native (SuperLU-based) solver available and
   faster than JavaMNA? Under what conditions does the code switch between them?
7. **`conductanceDelta` threshold tuning** — is 1000 an appropriate threshold for
   the observed conductance scale of 10^10–10^22?

---

## Deliverable

Write `INVESTIGATION_REPORT_CONVERGENCE.md` with these sections:

### 1. NR Loop Map
Complete pseudocode of the iteration sequence including warm-start, backtracking,
and convergence check.

### 2. Convergence Criterion Analysis
Exact stopping criterion formula. Whether a relative criterion exists. Computation
of what the 1e-7 absolute threshold means in voltage terms for this network's
conductance scale. Whether the threshold is achievable in floating point.

### 3. Matrix Scaling Audit
What scaling is applied, when, and whether it is sufficient for the observed
conductance range. Estimated condition number before and after scaling.

### 4. Source of 10^22 Conductance
Most likely candidate identified with supporting formula and parameter values. For
the BJT exponential candidate, what V_BE value produces 10^22 and whether `pnLim()`
prevents it.

### 5. Warm-Start Status
Whether `StateVector` is warm-started. If not, estimated iteration reduction from
adding warm-start.

### 6. Admittance Matrix Rebuild Frequency
What triggers the ~8s rebuild. Whether the adaptive BE/BDF2 `isStiff()` switch or
`G_add` ramp could be contributing via `conductanceUpdates`.

### 7. Ranked Improvements
Table of candidate improvements ranked by expected impact:

| Rank | Improvement | Impact | Effort | Risk |
|------|-------------|--------|--------|------|
| 1 | ... | ... | ... | ... |

---

## Acceptance Criteria

- [ ] `INVESTIGATION_REPORT_CONVERGENCE.md` exists in repo root
- [ ] Section 4 identifies the source of 10^22 conductance with a specific formula
  and plausible parameter values, or explicitly rules out all four candidates
- [ ] Section 2 states whether any relative stopping criterion exists and computes
  the floating-point achievability of the 1e-7 threshold at this conductance scale
- [ ] Section 5 states definitively whether StateVector is warm-started or zero-init
- [ ] Section 6 determines whether `G_add` increments `conductanceUpdates`
- [ ] Section 7 contains at least 5 ranked improvements with effort/impact/risk
- [ ] No source files were modified
