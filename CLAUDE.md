# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

Create: Power Grid is a Minecraft mod that adds physics-based electricity simulation to the Create mod. It targets **NeoForge** (primary) and Fabric (currently commented out in `settings.gradle`), built with the **Architectury** multiplatform framework on Minecraft 1.21.1.

## Build Commands

```bash
# Build NeoForge jar (output: forge/build/libs/)
./gradlew :forge:build

# Build Fabric jar (output: fabric/build/libs/)
./gradlew :fabric:build

# Run tests
./gradlew test

# Run a single test class
./gradlew test --tests "org.patryk3211.electricity.SolverTests"

# Build native acceleration library (requires cmake + C compiler)
./gradlew :native:buildLinux   # Linux
./gradlew :native:buildWindows # Windows (requires mingw64-cmake)
```

Tests use JUnit via `useJUnitPlatform()` and require the native library on the JVM path (configured automatically in root `build.gradle`).

## Architecture

### Architectury Multiplatform Layout

- `src/` — common code shared across platforms (bulk of the codebase)
- `forge/src/` — NeoForge-specific implementations (`@ExpectPlatform` fulfillments, platform entry point)
- `fabric/src/` — Fabric-specific implementations (currently excluded from `settings.gradle`)
- `native/` — C JNI library for accelerated MNA solving (cmake-based, uses OpenBLAS + SuperLU)

`@ExpectPlatform` methods in common code (e.g. `PowerGrid.createRegistrate()`) are fulfilled by platform-specific implementations. Architectury transforms calls at bytecode level.

### Electricity Simulation Core (`electricity/sim/`)

The heart of the mod. Implements Modified Nodal Analysis (MNA) to solve electrical circuits each game tick.

- **`ElectricalNetwork`** — the main circuit: holds wires, nodes, hooks, and drives the solver. Each `ServerLevel` has a `WorldNetworks` managed via `GlobalElectricNetworks`.
- **`GraphedElectricalNetwork`** — extends `ElectricalNetwork` with connectivity graph for split/merge on structural changes.
- **`node/`** — circuit node types: `ElectricNode` (voltage node), `CurrentSourceNode`, `FloatingNode`, `CouplingNode` (transformer coupling), `VoltageSourceCoupling`.
- **`solver/`** — MNA solver backends: `JavaMNA` (pure Java, always available), `NativeMNA` (JNI to native lib, optional), iterative solvers (`BiCGSTABSolver`, `GMRESSolver`).
- **`calculation/`** — stamping interfaces (`IStamped`) used by wires to contribute to the admittance matrix.
- **`special/`** — `TransmissionLine`, `TransmissionLinePort` for multi-tick delay elements.

### Electrical Blocks (`electricity/base/`)

Base classes for all electrical block entities:

- **`ElectricBehaviour`** — the main capability that wires connect to. Block entities hold one or more terminals.
- **`ElectricBlockEntity`** — base BE class; carries an `ElectricBehaviour`.
- **`terminals/`** — terminal placement helpers (`RotatedTerminalCollection`, etc.) for blocks with multiple connection points.
- **`ThermalBehaviour`** — optional thermal simulation attached to electrical devices.

### Devices (`electricity/`)

Each sub-package is one device type: `wire/`, `battery/`, `transformer/`, `heater/`, `light/`, `resistor/`, `fuse/`, `contactor/`, `sparkgap/`, `electromagnet/`, `gauge/`, `socket/`, `redstoneconverter/`, etc.

Wire connections use `WireEntity` (hanging entity) or `BlockWireEntity`. `GlobalElectricNetworks` resolves endpoints to `ElectricalNetwork` nodes when wires connect/disconnect.

### Kinetics (`kinetics/`)

Create-style kinetic devices that bridge electricity ↔ rotation: `generator/`, `motor/`, `servo/`, `variac/`, `rheostat/`, `plotter/`, `punchcard/`.

### Circuit Board System (`circuits/`)

An in-game circuit board editor allowing players to design PCB-style circuits:

- **`components/`** — individual component types (resistors, capacitors, inductors, BJTs, tubes, etc.) that map to `ElectricalNetwork` elements.
- **`editor/`** — the schematic editor logic.
- **`schematic/`** — schematic serialization.
- **`circuitboard/`** — the in-world circuit board block/BE.
- **`thermal/`** — thermal model for circuit board components.

### Registration (`collections/`)

All `Modded*` classes (`ModdedBlocks`, `ModdedItems`, `ModdedBlockEntities`, etc.) hold `DeferredRegister` instances and are called from `PowerGrid.register()`. `AbstractPowerGridRegistrate` wraps Create's Registrate for block/item registration with Create-style rendering.

### Native Acceleration (`native/`)

Optional JNI library (`libpowergridNative7.so` / `.dll`) wrapping OpenBLAS and SuperLU for fast sparse LU factorization. `NativeMNA.tryLoad()` attempts to load it at startup; falls back to `JavaMNA` on failure. Configure cmake path in `native/gradle.properties`.

## Key Patterns

- **Solver hooks**: `ISolverHook`, `IOuterHook`, `IMultiHooks` — wires implement these to participate in nonlinear iteration (Newton-Raphson via stamping).
- **`@ExpectPlatform`** — any method throwing `AssertionError` in common code has a platform counterpart in `forge/` or `fabric/`.
- **`SubstituteBlockEntityProvider`** — allows registering factory overrides for BEs before they are constructed (used for Sable compatibility).
- **Parchment mappings** — human-readable parameter names layered on top of Mojmap.

## Simulator Internals — Non-Obvious Gotchas

These facts are not apparent from reading the code and have caused implementation errors in the past.

### Companion model stamp timing

Reactive wires (`CapacitorWire`, `InductorWire`, `CRSeriesWire`, `LRSeriesWire`) use a Norton companion model split across two methods:

- **`conductance()`** → stamped into the admittance (Jacobian) matrix
- **`addStaticResidual()`** → stamped into the RHS as `Ieq = -G*V - Iprev`

`postUpperSolve()` runs at the end of tick T and updates both `Iprev`/`Vprev` AND the stored state `V`/`I`:

```
entry: V = V[T-1], Vcap/Inew = V[T] (just solved)
exit:  V = V[T] * 0.99999          ← stored field is NOW V[T]
```

When `addStaticResidual()` runs in tick T+1 it reads `V = V[T]`, **not V[T-1]**. Any formula for `Iprev` must account for this. Concretely:

- **BDF2** (2-step): `Iprev = C*(V[T] - V[T-1]) / (2dt)` — correct, both values available at write time
- **BE** (1-step): `Iprev = 0` — the `Ieq = -G*V` term alone gives `-(C/dt)*V[T]` ✓. Setting `Iprev = C*(Vcap-V)/dt` looks right at write time but is wrong when consumed next tick.

### conductance() is NOT called every tick

`conductance()` is called only in `populateConductanceMatrix()` (topology change / floating-point drift correction) and when `currentMultiTick` changes. `addStaticResidual()` IS called every tick.

Implication: if `conductance()` returns a value that depends on runtime state, the admittance matrix will not update automatically. The wire must call `network.updateConductance(this, newG - oldG)` explicitly (as `setCR`, `setLR`, etc. already do).

### BDF2 produces complex eigenvalues when dt/τ > 0.5

For the RC/RL decay ODE `y' = -y/τ`, BDF2 characteristic polynomial `(3+2z)λ² - 4λ + 1 = 0` (z = dt/τ) has discriminant `4 - 8z`. Roots go complex (overshoot/ringing) when **z > 0.5**. Backward Euler is monotonic for all z and should be used in the stiff regime.

### isConverged() guard + warmUp(1) interaction

`ElectricalNetwork.warmUp(1)` forces `converged = false` for the next tick. It is called on every topology change (switch open/close, wire add/remove). Any `if(isConverged())` guard in `postUpperSolve()` therefore fires — freezing `Iprev`/`V` — on every switching event, guaranteed, even in purely linear circuits that would otherwise converge in one NR iteration.

### Session memory

Non-obvious insights accumulated across sessions are indexed at:
`~/.claude/projects/-home-kuba-code-PowerGrid/memory/MEMORY.md`
