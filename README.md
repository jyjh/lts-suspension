# lts-suspension

Suspension component package for the FSAE transient lap-time simulation:
the `+Suspension` classes (`SuspensionManager`, `SimpleSuspension`,
`SuspensionGeometry`, anti-roll bars, force/travel elements), mounted into
the main repository at `src/+lts/+components/+Suspension`.

## Ownership

| | |
|---|---|
| Department | Suspension (Vehicle Dynamics) |
| Maintainer | *add GitHub handle* |
| Term | *e.g. 2026/27* |

## Running the tests

Requires MATLAB R2019b+ (CI pins R2026a) and the `lts-kit` submodule:

    git submodule update --init --recursive

Then in MATLAB, from the repository root: `run_tests`

The runner assembles a temporary `+lts` package sandbox in `build/`
(gitignored) — this repository's classes plus kit's `+util` — and runs
`tests/`. Nothing is installed into the main repository.

## Branch model and workflow

- `staging` — where PRs from forks land. `main` — stable, release-only; it advances only via the release cascade from the main `lts` repository.
- All development is done on forks; see [CONTRIBUTING.md](CONTRIBUTING.md).

## Contract with the main repository

- `SuspensionManager` reads only the vehicle geometry it is handed at
  construction (a struct with `wheelbase`, `trackWidth`, `cgHeight`,
  `staticFrontWeight`, optional `g` works); it never references the main
  repository's classes by name.
- The chassis↔suspension link is structural: the chassis probes for the
  corner-unit / roll-center / anti-geometry properties and
  `getAxleRollStiffness`; the suspension probes for
  `getFrontRollAngle`/`getRearRollAngle` on an optional linked chassis.
  Neither package requires the other on the MATLAB path.
- `tests/ConformanceTest.m` pins the `cfg.suspension` schema
  (`validateConfig`), the `SuspensionComponent` interface, and the
  `SuspensionState`/corner property names feeding the per-corner
  telemetry channels (`damperPos_FL`, `Fz_FL`, ...). Renaming any of
  them is a **contract change** — see "Changing the contract" on the
  [Contracts page](https://jyjh.github.io/lts/contracts/).
- Details: <https://jyjh.github.io/lts/repo-split/>

## Anti-roll-bar force and integration

The bar's differential wheel rate is `B = stiffness * motionRatio^2 /
leverArm^2`. Both wheel reactions use the same suspension-travel difference;
the equivalent axle roll stiffness is `B * trackWidth^2`. This is a linear,
symmetric, zero-preload installation; use an installed effective rate when
mount or link compliance matters.

ARB motion ratios may exceed one. An enabled bar requires a strictly positive
lever arm; a disabled bar may retain a zero placeholder. Nonfinite rates and
invalid enable values are rejected. Configuration names and telemetry fields
are unchanged.

Warm starts solve the shared bar force together with both tire/spring corner
equilibria, including the unilateral tire and bump-stop laws. Dynamic updates
advance both wheels on shared substeps and refresh endpoint bar reactions.
The optional `getMaxIntegrationStep()` capability bounds the anti-phase mode
including `2*B`; vehicle callers should subcycle the complete chassis/suspension
loop at this bound rather than advancing the chassis with frozen reactions.

The integration bound is memoized against every corner stiffness, effective
motion ratio, unsprung mass, integration cap, and axle bar rate. Changing any
input recomputes it. Chassis-state capabilities are cached by class, with live
reflection for dynamic properties; state values and bar forces remain live.
Private transient handle caches support the manager's value-class semantics
without retaining component handles or sharing physical state between copies.
