# Free-DOF gradient balance tools

Record: [docs/gradient-balance-free-dofs-20260929.md](../../docs/gradient-balance-free-dofs-20260929.md).
Python 3.11+ with NumPy. They reuse the clamped-contact matrix driver
(`tools/clamped/sequence.py --balance-dofs all|free`, which states
`semi_implicit/gradient_balance_dofs` and prefixes the label) and EF-01's
reducer, and change no default or scene file.

```sh
W=/path/to/fable_polyfem; E=/fresh/evidence; B=$E/bin/PolyFEM_bin
python3 tools/clamped/sequence.py --workspace "$W" --out "$E/matrix" --binary "$B" --clamped exclude_statistics             # option absent
python3 tools/clamped/sequence.py --workspace "$W" --out "$E/matrix" --binary "$B" --clamped exclude_statistics --balance-dofs free
python3 tools/balance/synthetic.py --binary "$B" --out "$E/synthetic" --mode default --mode all --mode free
python3 tools/balance/reduce.py "$E"/matrix/runs/* --json "$E/characterization.json"
python3 tools/balance/compare.py --pair IT REF_RUN RUN [--pair ...] --json "$E/compare.json"
python3 tools/balance/gaps.py RUN [RUN ...]
```

* `reduce.py` reads the full trim-predictor records' `clamped.gradient_balance`
  block (the controller's balance, `all` or `excluding_fully`, and
  `free_dofs`), reconstructs the split into free and Dirichlet rows from
  the norms and cosines, and reports the factors of the full-DOF balance:
  `barrier_share` = ‖gB on free rows‖ / ‖gB‖ (the clamped half of partly
  clamped contacts), `energy_share` = the same for gE (Dirichlet reactions),
  `cross` = ⟨gB,gE⟩ / ⟨gB,gE⟩ on free rows. kappa_all = kappa_free · cross ·
  barrier_share², cos_all = cos_free · cross · barrier_share · energy_share.
  It also counts gate flips and the calibrating records at which each
  balance exceeds the trim in force, and reports kappa / trim.
* `synthetic.py` builds drops (transient, gravity) and presses
  (quasistatic) of the smoke's cube onto a coarse obstacle (the smoke's
  4-vertex slab), a fine grid obstacle, a clamped block, or an elastic
  support clamped at its bottom (control: no clamped half), in the modes
  `default` (key absent), `all` and `free`.
* `compare.py` compares pairs of runs: iterations, stall retunes, AL passes,
  trim path and calibration raises (with the count below 1 %), free-contact
  minimum gap, VTU byte identity and the relative L2 solution difference.
* `gaps.py` summarises the realized gaps per step (last iteration).

IT is trajectory-sensitive: deterministic runs that differ only in their
roundoff source differ by up to 23.5 % ([IT reproducibility](../../docs/it-reproducibility-20260928.md)).
Compare an IT candidate with an ensemble of realizations (the linear solver:
AccelerateLDLT, CHOLMOD, SimplicialLDLT), not with one run. R4 and BB are
multithreaded and not reproducible run to run; they need repeats.
