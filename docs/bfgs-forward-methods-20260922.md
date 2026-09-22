# Forward-method support and test reporting — audit stage 5

Date: 2026-09-22. **Status: done within the stated scope. The Houdini asset now
offers only the nonlinear methods PolyFEM's simulation can run, and runs each
of them; every unsupported choice is refused by name where it is used; the
BFGS tests are deterministic and say which strategy converged. The reporting
also exposed an ADAM defect, repaired here.**

Stage 5 of the [BFGS convergence audit](bfgs-convergence-audit-20260922.md)
asked to remove or explain the forward L-BFGS-B option (finding 5), add an HDA
JSON round-trip check and a clear forward validation message, keep
deterministic BFGS tests that fail on exceptions and record which strategy
converged, and label the intentionally permissive stress tests.

## Scope

PolySolve: clearer errors where box-constrained methods, dense BFGS and the
dense Newton strategies are refused; an ADAM repair; deterministic BFGS tests;
the permissive matrices report what they tolerate. PolyFEM: no source change,
only the pin and these records. Houdini: the nonlinear menu, BFGS's linear
solver, the importer's messages and the end-to-end test. No default, tolerance,
line search, contact law, CCD rule, trial cap or friction policy changed.

Starting checkouts were clean: PolyFEM `e099f2048`, PolySolve `fc62a679`, IPC
`482b9eab`. Evidence is in the parent workspace's
`outputs/bfgs-stage5/20260922T192123Z/`; the pre-change binary is
`baseline-PolyFEM_bin` (SHA-256 in `baseline-SHA256SUMS`), the final one's
hash is `final-SHA256`.

## What the asset offered, measured first

`probe_methods.py` (in the evidence) runs each method the asset's menu offered
through a forward solve of the public `quasistatic-semi` scene, one time step,
single-threaded, with the asset's default (sparse) linear solver and, for the
two dense methods, with `Eigen::LDLT` as well:

| method | before | after |
| --- | --- | --- |
| Newton | converges | unchanged |
| GradientDescent, StochasticGradientDescent | run, do not converge in one step | unchanged |
| ADAM, StochasticADAM | **ran as GradientDescent** (every direction NaN, see below) | run as ADAM; do not converge in one step |
| L-BFGS | completes the step | unchanged |
| BFGS | `BFGS linear solver must be dense, instead got Eigen::SimplicialLDLT` — **always, from the asset** | same refusal, now naming the fix; the asset exports `Eigen::LDLT`, and BFGS runs |
| BFGS, `Eigen::LDLT` | runs, does not converge | unchanged |
| DenseNewton | `Newton linear solver must be dense` | the same, plus: the dense strategies also need a dense Hessian; use Newton |
| DenseNewton, `Eigen::LDLT` | `Dense Hessian not implemented.` | `… by this problem: the dense Newton strategies … cannot run on it; use a sparse Newton strategy (Newton).` |
| L-BFGS-B, MMA | `Unrecognized solver type: L-BFGS-B` | `… is a box-constrained method: it runs only in BoxConstraintSolver (bounded optimization) …`, listing the unconstrained methods |

So finding 5 was one case of three: of the ten menu entries, **L-BFGS-B, MMA
and DenseNewton could never run**, and **BFGS could never run from the asset**,
whose linear-solver menu has no dense solver. No PolyFEM problem implements
`Problem::hessian(x, TMatrix&)`.

## Where the refusal lives

Every forward path — the elastic, thermoelastic, fluid, FSI, differentiable
and legacy solves, the AL and reduced stages — constructs its solver through
PolySolve's `nonlinear::Solver::create`, and the dense Newton strategies fail
at the problem's dense-Hessian call. The messages are made precise there, so
every caller inherits them.

An init-time refusal in PolyFEM was considered and **not** added: a linear
problem never builds a nonlinear solver, so refusing an unused
`solver/nonlinear` setting at `State::init` would stop inputs that run today,
and deciding which inputs will build one needs the assembled problem.
Constructing linear solvers at init to test density was also rejected: several
have process-wide side effects (MPI and Hypre initialization, Pardiso).

## The asset

- The menu offers **Newton, Gradient Descent, ADAM, Stochastic ADAM,
  Stochastic Gradient Descent, L-BFGS and BFGS**. Houdini saves ordinal menus
  by token (checked in a saved `.hip`), so removing entries shifts no other
  saved choice. A scene saved with a withdrawn method loads with Houdini's own
  warning — *Parameter value DenseNewton in solver_nl is invalid. Defaulting
  to 0.* — as Newton (checked with a scene saved by the previous asset). A
  `params.json` naming one imports as Newton, with the reason in the Import
  Report.
- With BFGS selected the asset exports `solver/linear/solver: "Eigen::LDLT"`
  and disables the Linear *Solver* menu, which keeps its choice for the other
  methods; the importer does not report the implied dense solver as foreign.
- The end-to-end test (`tests/test_polyfem_hda.py`) now exports **every**
  offered method, runs the real binary on it and requires that the strategy
  actually iterated (its log tag present, no refusal message), and imports the
  file back (same method, nothing reported); and for each withdrawn method it
  requires the Newton import with the reason and PolyFEM's refusal by name.
  All seven offered methods exit 0 on the test's one-step scene. All 13 HDA test
  files pass.

## ADAM never took an ADAM step

The permissive matrices' new escalation report showed every ADAM and
Stochastic ADAM solve escalating to GradientDescent with
`update_direction_failed`. PolySolve's ADAM counted its steps from 0, so its
first bias correction divided by `1 - beta^0 = 0`: the first direction was
NaN, the solver escalated, and each return to ADAM reset the counter and
repeated it. It also never kept its moment estimates. The step counter now
starts at 1 and the moments carry over (Kingma and Ba, Algorithm 1). The new
`adam-takes-its-own-steps` regression fails without the repair (*Update
direction could not be computed on last strategy*) and passes with it; ADAM
now runs as ADAM in PolyFEM. **Behaviour change:** a scene that selected ADAM
ran Gradient Descent until now.

## Tests

| test | what it states |
| --- | --- |
| `bfgs-deterministic-matrix` | BFGS and L-BFGS × Armijo, RobustArmijo, Backtracking, Wolfe × Quadratic, Sphere, Beale, Rosenbrock × two fixed starts each: the strategy alone, an exception fails, it converges on the configured gradient norm to the solution with no strategy transition — 64/64 |
| `bfgs-deterministic-chain-records-the-finishing-strategy` | the same 64 in the usual string form (GradientDescent appended as fallback): each converges, and the finishing strategy and every escalation are read back from the solver info — none escalates |
| `box-constrained-methods-are-named-by-the-unconstrained-solver` | L-BFGS-B and MMA are refused by name by `Solver::create` and still built by `BoxConstraintSolver` |
| `dense-method-requirements-are-named` | dense BFGS with a sparse linear solver, and DenseNewton on a sparse-only problem, fail naming their fix |
| `adam-takes-its-own-steps` | ADAM and Stochastic ADAM converge alone on the quadratic |
| `nonlinear-stress-permissive` (was `nonlinear`), tagged `[stress]` | unchanged tolerance; its `WARN` line reports converged / abandoned / escalated solves by combination |
| `nonlinear-easier` | unchanged tolerance; its `WARN` line lists every combination a fallback finished |

Measured by those reports on the final build: the stress matrix converges
533 solves, abandons 94 on an exception (permitted) and escalates 77; the easier
matrix converges 600, abandons none and escalates 141. Before the ADAM repair
they were 617 / 75 / 171 and 600 / 0 / 192: ADAM now does its own, slower work
where Gradient Descent used to take over, so more permissive solves reach the
1,000-iteration limit. Among the BFGS strategies only ResidualBacktracking on
Beale escalates (3 BFGS and 4 L-BFGS solves, from random starts); the
deterministic tests do not cover ResidualBacktracking, a residual-norm search.

| suite (final binary) | result |
| --- | --- |
| PolySolve `[bfgs],[solver]` | 45 cases / 2,591 assertions (stage 3: 40 / 2,083) |
| PolyFEM solver, form, observer and `[input_validation]` suites | 110 cases / 10,535 assertions |
| Five public Newton smokes, diagnostics off and on | frames byte-identical to stage 4; attempt streams pass the RB-04 checker |
| Houdini, all 13 test files | pass |

## Acceptance against the plan's item 5

| requirement | result |
| --- | --- |
| Remove or explain the unsupported forward L-BFGS-B option, keeping box-constrained optimization | removed from the asset with MMA and Dense Newton; `BoxConstraintSolver` still builds both (tested) |
| An HDA JSON round-trip check | every offered method, exported, run and re-imported; every withdrawn one imported with its reason |
| A clear forward validation message | at the point of use, for every forward caller: box-constrained methods, dense BFGS's linear solver, the dense Newton strategies |
| Deterministic BFGS tests that fail on exceptions and record which strategy converged | the two `bfgs-deterministic-*` cases, 128 solves |
| Permissive stress tests separately labelled | `nonlinear-stress-permissive` `[stress]`, reporting what it tolerated |
| Rebuild; affected PolySolve and PolyFEM suites, public smokes, HDA E2E | all pass on the final binary |

## Limits

- The method probe is one public scene, one time step; "runs" means the
  strategy was constructed and iterated, not that it converges. The
  first-order methods and dense BFGS do not converge there in one step.
- DenseNewton is refused because no PolyFEM problem assembles a dense Hessian
  today; a problem that did would make it usable again, and the asset would
  need its entry back.
- ADAM is not a descent method: under a line search its steps are sometimes
  rejected and the fallback takes over (93 of the easier matrix's 141
  escalations are ADAM's or Stochastic ADAM's). That is the method, not the
  repaired defect.
- The Wolfe line search of stage 3 is still not exposed in the asset.

## Published revisions

PolySolve [`c874cd59`](https://github.com/sdast9/polysolve/commit/c874cd59ca68afa6e7fc6b044581bd37bc2c536a),
branch `iteration-callback`. PolyFEM: the pin and this record. Houdini:
[`sdast9/houdini-plugins@99ba038`](https://github.com/sdast9/houdini-plugins/commit/99ba038). Evidence in the parent workspace's
`outputs/bfgs-stage5/20260922T192123Z/`.
