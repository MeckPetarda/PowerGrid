# Task: Fix NR Convergence — Relative Stopping Criterion and conductanceDelta Guard

**Purpose:** The PowerGrid solver reports convergence failures on every tick for
circuits with ~23 nodes, causing 34% TPS deficit. Investigation confirmed two root
causes:

1. The absolute stopping criterion (1e-7 A) has no relative component, causing the
   solver to iterate all 250 times on circuits already converged to 2 µV accuracy.
   At G = 1000 S, a residual of 0.002 A represents a 2 µV voltage error — excellent
   accuracy — but the solver doesn't know it and keeps going.

2. `conductanceDelta` accumulates contributions from BJT conductance ramps during NR
   iterations (which are not guarded by `countUpdates`), triggering spurious full
   admittance matrix rebuilds every ~8 seconds instead of only on real topology
   changes.

Both fixes are small and well-isolated. Combined they should eliminate the majority
of convergence failure log spam and the associated tick time overrun.

**Files to modify:**
- `src/main/java/org/patryk3211/powergrid/electricity/sim/solver/JavaMNA.java`
- `src/main/java/org/patryk3211/powergrid/electricity/sim/ElectricalNetwork.java`

**Files NOT to modify:** All other files. No config changes, no interface changes,
no changes to reactive components or BJTWire.

**Reference:** `INVESTIGATION_REPORT_CONVERGENCE.md` in the repo root.

---

## Part A — Relative Stopping Criterion (JavaMNA.java)

### Background

The NR loop in `singleTick()` currently stops when:

```java
if (norm < absoluteStoppingCriterion || dNorm < relativeStoppingCriterion)
    break;
```

Where:
- `absoluteStoppingCriterion = 1e-7` — fixed threshold in amps
- `relativeStoppingCriterion = 1e-14` — detects plateau in consecutive norm difference

The plateau detector (`dNorm < 1e-14`) fires when the solver stops improving, not
when it has reached sufficient accuracy. For a limit-cycling solver, consecutive
norms oscillate slightly and `dNorm` never drops to 1e-14.

The missing criterion is: **has the residual dropped sufficiently relative to where
it started?** A circuit that begins a tick at norm_0 = 0.5 A and reaches norm = 5e-5 A
has improved by 4 orders of magnitude and is converged to engineering accuracy,
regardless of whether 5e-5 > 1e-7.

### Step A.1 — Add norm0 field

Add `private double norm0 = 0.0` to the fields section of `JavaMNA` alongside
`absoluteStoppingCriterion` and `relativeStoppingCriterion`.

### Step A.2 — Add relativeFactor field

Add `private double relativeFactor = 1e-4` alongside the other criterion fields.
This is the relative convergence factor: stop when `norm < relativeFactor * norm0`.

A value of 1e-4 means "stop when the residual has dropped by 4 orders of magnitude
from the first iteration of this tick." This is conservative enough to ensure the
solution has genuinely converged relative to the starting point, while being loose
enough to stop early on circuits that were already near their solution.

### Step A.3 — Capture norm0 on first iteration

In `singleTick()`, locate where `norm` is first computed (after the first `L∞`
calculation). Add capture on iteration 0:

```java
if (i == 0) {
    norm0 = norm;
}
```

This must be placed AFTER `norm` is first set and BEFORE the convergence check. Read
the existing loop structure carefully to find the exact insertion point.

### Step A.4 — Add relative check to the break condition

Change the existing convergence break from:

```java
if (norm < absoluteStoppingCriterion || dNorm < relativeStoppingCriterion)
    break;
```

To:

```java
if (norm < absoluteStoppingCriterion
        || norm < relativeFactor * norm0
        || dNorm < relativeStoppingCriterion)
    break;
```

Do not remove or modify the existing `absoluteStoppingCriterion` or `dNorm` checks —
they handle the cases where the circuit is small enough that absolute accuracy is
achievable.

### Step A.5 — Reset norm0 on state reset

Locate `zeroState()` or any other method that resets solver state between ticks.
Add `norm0 = 0.0` there to prevent a stale norm0 from a previous tick influencing
the next tick's relative criterion.

Also reset `norm0 = 0.0` at the top of `singleTick()` before the iteration loop
begins, as a safety measure, to ensure a fresh capture on iteration 0 each tick.

---

## Part B — conductanceDelta Guard (ElectricalNetwork.java)

### Background

`updateConductance()` increments two counters:

```java
conductanceUpdates++;             // guarded by countUpdates
conductanceDelta += Math.abs(change);   // NOT guarded — always increments
```

`countUpdates` is set to `false` during `iterHooks()` calls, which is where BJT
conductance ramps (`G_add` escalation, junction conductance updates) happen. This
means `conductanceUpdates` correctly ignores NR-internal changes, but
`conductanceDelta` accumulates them all.

The rebuild threshold `conductanceDelta > 1000` is therefore hit by BJT NR iterations,
not just by real circuit changes. For a circuit with converging BJTs this adds ~0.04–0.12 S
per tick; for failing BJTs during the ramp phase it adds ~20–40 S per tick, hitting
the 1000 S threshold every few seconds and triggering a full `populateConductanceMatrix()`
rebuild unnecessarily.

### Step B.1 — Add the guard

In `updateConductance()` in `ElectricalNetwork.java`, locate:

```java
conductanceDelta += Math.abs(change);
```

Change to:

```java
if (countUpdates) conductanceDelta += Math.abs(change);
```

This is a one-line change. Do not change the `conductanceUpdates++` line (it already
has the guard implicitly through the call structure — verify this is still true by
reading the surrounding code before changing).

### Step B.2 — Verify countUpdates is reset correctly

Confirm that `countUpdates` is always set back to `true` after `iterHooks()` completes.
If there is any code path where `countUpdates` could be left as `false` after the
method returns (e.g. via exception or early return), that would incorrectly suppress
all conductanceDelta accumulation. Read the code around the `iterHooks()` call and
confirm the try/finally pattern or equivalent exists. If it does not, add it.

---

## Acceptance Criteria

### Part A
- [ ] `JavaMNA.java` compiles without warnings
- [ ] `norm0` field exists and is initialised to 0.0
- [ ] `relativeFactor` field exists with value 1e-4
- [ ] `norm0` is captured on iteration 0 inside `singleTick()`
- [ ] Break condition has three clauses: absolute, relative, and plateau
- [ ] `norm0` is reset to 0.0 at the start of each `singleTick()` call
- [ ] **Convergence log test:** run the previously-laggy circuit. The
  "could not converge in 250 iterations" log messages should be significantly
  reduced or absent
- [ ] **Basic RC test** (R=20kΩ, C=100µF, τ=2.0s): τ_eff ≈ 2.0s ± 5% — no
  regression from the convergence criterion change
- [ ] **Charge pump test** (R1=4.7kΩ, R2=22kΩ, R3=50kΩ, C1=4.7µF, C2=100µF,
  Vcc=60V, 1Hz): steady-state C2 ≈ 7.5–10V — no regression

### Part B
- [ ] `ElectricalNetwork.java` compiles without warnings
- [ ] The `conductanceDelta +=` line has the `if (countUpdates)` guard
- [ ] `countUpdates` is confirmed to be reset to `true` after `iterHooks()` in all
  code paths
- [ ] Matrix rebuild frequency is reduced — the ~8 second cadence of
  "populateConductanceMatrix" log messages (if any) should be less frequent
  under normal operation

### Combined
- [ ] TPS is measurably improved on the previously-laggy circuit (target: above
  18 TPS, was 13.21 TPS)
