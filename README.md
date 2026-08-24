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

- `staging` — where PRs from forks land. `main` — stable, release-only.
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
- Details: <https://jyjh.github.io/lts/repo-split/>
