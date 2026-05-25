# Task: Investigate Newton-Raphson Convergence Failures and Performance

**Purpose:** The simulator is logging convergence failures ("could not converge in 250
iterations") during normal gameplay, causing microlags. This task is to map the full
NR implementation, identify why it fails to converge within the iteration budget, and
catalogue the available improvement strategies with enough codebase context to write
targeted fix tasks.

**Files to modify:** None. Read-only investigation.
**Output:** Write `INVESTIGATION_REPORT_CONVERGENCE.md` in the repo root.

---

## Background

Newton-Raphson convergence failure in circuit simulation has several distinct root
causes, each requiring a different fix:

**A — Starting point too far from solution**
The initial guess for the NR iteration is far from the true solution. Each iteration
makes progress but 250 steps isn't enough to get within tolerance.

**B — Jacobian accuracy**
The Jacobian is wrong or poorly conditioned. NR steps overshoot or oscillate around
the solution without converging.

**C — Tolerance too tight**
`absoluteStoppingCriterion = 1e-7` may be unachievable for circuits with large node
voltages or currents (floating point noise floor rises with signal magnitude).

**D — Limit cycling**
The iteration is stuck in a loop — two or more states that each point to the other.
This is a qualitative failure, not just a slow convergence.

**E — Genuinely ill-conditioned circuit**
The circuit has near-degenerate topology (floating nodes, nearly-parallel voltage
sources) that makes the Jacobian singular or near-singular.

**F — Interaction with the new BE/BDF2 adaptive scheme**
The companion model conductance changes between BE and BDF2 modes (at dt/τ = 0.5
threshold). If a component oscillates across this threshold between ticks, the
admittance matrix is rebuilt with a different G, potentially destabilising the NR
starting point.

---

## Step 1 — Map the Full NR Loop

Read `JavaMNA.singleTick()` in full. For each of the following, record the exact
line number and current value or formula:

- Maximum iteration count (`maxIterations`) — default and config override
- `absoluteStoppingCriterion` — the convergence threshold
- `minimumAllowedPrecision` — the "failed to converge" threshold
- The norm used to measure convergence — what quantity, what norm (L1, L2, L∞)?
- What happens when the iteration limit is reached without convergence — is the
  last computed state used, or is some fallback applied?
- The `slowdown` mechanism (backtracking step) — when does it trigger, what does
  it do, and does it count against the iteration budget?
- `G_add` — how it is initialised per tick, how it grows across iterations, and
  whether it is reset between ticks

---

## Step 2 — Map the Initial Guess Strategy

NR starts each tick from some initial guess for node voltages. Identify:

- What is `StateVector` initialised to at the start of each `singleTick()` call?
- Is it the solution from the previous tick (warm start), or reset to zero (cold
  start)?
- For multi-tick networks (per-network adaptive multiTick from Part C), is the
  state carried between sub-ticks or reset each sub-tick?
- Does the order in which components call `preSolve()` / `iterHooks()` affect the
  starting point?

A warm start (previous tick's solution as initial guess) is standard in SPICE and
dramatically reduces iterations for slowly-changing circuits. A cold start wastes
most of the iteration budget on the approach phase.

---

## Step 3 — Analyse the Jacobian Construction

Read the Jacobian stamp for each nonlinear component:

**BJTWire:**
- Where is the Jacobian (conductance matrix update) computed for the BJT?
- Is it recomputed every NR iteration or only on certain iterations?
- What is the formula for `Gee`, `Gcc`, `G_add`? How do these relate to the
  standard Ebers-Moll transconductance `g_m = I_C / V_T`?
- Is there a `diodeSmoothAlpha` equivalent for the BJT's Jacobian, or is it
  the full Newton step?

**PNJunctionWire (diode):**
- Same questions — where is the Jacobian computed, what is the formula, is
  `diodeSmoothAlpha` applied to the Jacobian update or only the voltage limiting?

**Note:** All resistors and linear elements have exact Jacobians by definition.
Only nonlinear components matter here.

---

## Step 4 — Profile the Iteration Budget

For a circuit that triggers convergence failures, estimate how the 250-iteration
budget is actually spent. Without runtime instrumentation, use static analysis:

- In the `slowdown` backtracking path: each backtrack costs an extra iteration
  without advancing the solution. How many backtracks can occur before the budget
  is exhausted? Is there a limit on consecutive backtracks?
- `G_add` growth: at what iteration does `G_add` first materially change the
  Jacobian (i.e. become comparable to the smallest circuit conductance)? Does the
  `G_add` ramp help convergence or hurt it by distorting the true Jacobian?
- After convergence, `postUpperSolve()` runs for all components. For the new
  BE/BDF2 adaptive scheme: can `isStiff()` change between the pre-solve and
  post-solve state? If yes, the admittance matrix used during NR would be
  inconsistent with the history term written in `postUpperSolve()`.

---

## Step 5 — Identify Circuit Topologies That Reliably Fail

Without runtime data, reason about which circuit types are most likely to hit the
250-iteration limit:

- **Diode in forward conduction**: exponential I-V with large dynamic conductance.
  What is the NR step size for a typical forward-biased diode? Does `diodeSmoothAlpha`
  adequately limit it?
- **BJT in or near saturation**: both junctions conducting, highly nonlinear. Does
  the solver have any saturation detection?
- **Switched capacitor (BE mode)**: at the first tick after a switch closes, the
  capacitor stamp changes from its pre-switch value. Does the NR starting point
  account for this?
- **Mixed stiff/non-stiff network**: after the adaptive BE/BDF2 change, a network
  with components on both sides of the dt/τ = 0.5 threshold will have mixed
  conductance stamps. Does this create any particular NR difficulty?

---

## Step 6 — Survey Known Convergence Improvement Techniques

For each technique below, check whether it is already present in the codebase,
partially present, or absent:

**Source stepping:** Ramp voltage/current sources from 0 to their target value
across multiple NR iterations when cold-starting. Prevents large initial residuals.
Look for any ramping of source values in `addStaticResidual()` of source components.

**Continuation methods:** Solve a simpler version of the circuit first (e.g. with
all nonlinear elements linearised), then use that solution to initialise the full
nonlinear solve. Look for any `homotopy`, `continuation`, or `operating point` code.

**Adaptive damping:** Rather than the current fixed `0.5` backtrack step, use a
line search to find the optimal step length. Look for any line search or Armijo
condition in `singleTick()`.

**Gmin stepping:** Temporarily add small conductances (`G_add` / `Gmin`) across
all nodes to improve Jacobian conditioning, then gradually reduce them. The current
`G_add` ramp is similar but check whether it is reduced back to zero after convergence
or stays elevated.

**Node voltage limiting:** For each NR step, clamp the maximum change in any node
voltage to a fixed value (e.g. 2V per iteration) to prevent large overshoots. Look
for any per-node clamping outside of the diode/BJT voltage limiters.

**DC operating point solve:** Before transient simulation begins, find the DC
solution with all capacitors open and inductors shorted (or vice versa). Use this
as the warm start for the first transient tick. Look for any DC init or `dcSolve`
call in `ElectricalNetwork` initialisation.

---

## Step 7 — Check the Convergence Failure Handler

When the iteration limit is reached:

- What log message is emitted? Is it per-tick (could fire 20 times/second) or
  rate-limited?
- Is the last non-converged state used as the result, or is the previous tick's
  state kept?
- Does convergence failure trigger any circuit-level recovery (e.g. reinitialising
  the network, forcing a cold start next tick)?
- Is there any mechanism to detect persistent failure (e.g. failing every tick for
  N consecutive ticks) and take corrective action?

If the last non-converged state is used as the tick result, estimate how wrong it
might be for a typical failure case — this determines whether the microlag is the
only symptom or whether simulation accuracy is also degraded during failure.

---

## Deliverable

Write `INVESTIGATION_REPORT_CONVERGENCE.md` with these sections:

### 1. NR Loop Map
Complete table of all parameters from Step 1 with line numbers.

### 2. Initial Guess Strategy
Warm vs cold start, with evidence from the code.

### 3. Jacobian Analysis
For each nonlinear component: formula, update frequency, damping mechanism.

### 4. Iteration Budget Analysis
How the 250 iterations are spent in a failure case, with estimates.

### 5. Failure-Prone Circuit Topologies
Which circuits are most likely to fail and why.

### 6. Technique Survey
Table of improvement techniques, presence in codebase, and estimated impact:

| Technique | Present? | Estimated impact | Implementation complexity |
|-----------|----------|-----------------|--------------------------|
| Source stepping | yes/partial/no | high/medium/low | high/medium/low |
| ... | | | |

### 7. Convergence Failure Handler
What happens on failure, whether accuracy is affected, and whether the log spam
can be rate-limited immediately as a stopgap.

### 8. Recommended Priority Order
Given everything found, rank the top 3 improvements by (impact / implementation
complexity) ratio — i.e. the changes that give the most convergence improvement
for the least code change.

---

## Acceptance Criteria

- [ ] `INVESTIGATION_REPORT_CONVERGENCE.md` exists in the repo root
- [ ] All seven steps are covered
- [ ] Every claim about the codebase cites a specific file and line number
- [ ] Section 8 gives a concrete ranked list of at most 3 recommendations with
  justification
- [ ] No source files were modified
