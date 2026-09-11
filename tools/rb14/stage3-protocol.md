# RB-14 stage 3 — physical neighborhoods, shared predictors and protection proposals

2026-09-10. Written before running the new comparisons. Preserve stage 2 sources,
protocol, cases and results. Reuse its assembled P1 Neo-Hookean selected-contact
fixture, actual IPC selector/extraction, implicit-Euler weights, explicit free
reduction, nonlinear solver/CCD/determinant checks and numerical tolerances.
The selected point/segment proxy does not represent the whole square surface.

Compare full reference; graph radius 1 (prior failure control); rest-space
Euclidean neighborhoods of radius .25, .5, 1 in the unit square, with exterior
increments fixed; and radius .5 compliance combined with the full sparse shared
predictor p=d0-J*solve(H,r). No radius tuning after inspecting results. Measure
K error and predictor error separately. Decompose local prediction error using
full z=solve(H,J^T) into missing exterior residual influence and interior relaxation
error. The decomposition uses reference data for diagnosis, not a cheap estimator.

Record sparse factor reuse for all bottom-node normal RHS together, and one
shared residual solve. This is a cost/reference calculation, not a coupled-contact
simulation. Check each RHS residual. Retain factor/matrix/RHS storage accounting;
no total peak-memory or production-throughput claim. Report the full solve cost
of the shared predictor separately from local extraction cost.

Protection proposals only: when the fresh full reference has positive target
demand, use its target coefficient. When it has zero target demand, compare
(a) freshly evaluated current assignment with trim=1 and (b) k=K_full/b''(.05),
matching barrier tangent to the fresh reference stiffness at d=.05. k has
force/length^3; b'' has length^2. This is an explicit response-scale candidate,
not a rigorous safety bound, new floor or selected physical law. Positive finite
SPD data and positive b'' are required; unavailable/indefinite/stale data cannot
produce either proposal. No target-upper-gap requirement for uncompressed cases.
Preserve CCD; never interpret zero demand as permission to remove active contact.
Measure both endpoint gap and force. Do not install either production policy.

Calibration is the same 15 stage-2 calibration configurations. Its three former
held-out cases are now labeled prior challenges, never new holdouts. Three new
untouched holdouts: (1) n=10, flipped diagonals, E=4,rho=.5,dt=.25,speed=.9,nu=.4;
(2) n=6,speed=1.2; (3) n=4,E=2,rho=.2,load=-5. The third is deliberately a
compressive RHS-sign control based on stage 2's measured sign convention.
All other settings retain the stage-2 defaults. No input or tolerance tuning.

Calibrate empirical K_true/K_est and (p_true-p_est)/dhat ranges separately for
physical-radius .5 and shared-predictor candidates on calibration only. Report
coverage on prior challenges and new holdouts separately. No safety padding or
enclosure claim. Compare nonlinear gaps and empirical interval existence without
silently choosing from empty/inactive intervals. Document nonlinear reference
error even when the shared predictor is exact for the frozen quadratic.

Use the unchanged screens: solve/identity residual 1e-8, FD 2e-5 with step 1e-6,
Newton 80 iterations, backtracking 40, Armijo 1e-4. A converged equilibrium is
not a target-accuracy proof. Preserve failed/incomplete runs and zero-demand
classifications. Confirm graph-radius failure control against stage 2; compile
only the isolated probe and run relevant mapping/elasticity regressions. No
private scene, Teseo, HDA, production coefficient or dependency change.
