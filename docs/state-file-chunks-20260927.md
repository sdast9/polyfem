# State files sized to their data — 2026-09-27

## Problem

Each per-step restart state file of the ball-burst scene (217,480 nodes,
652,440 DOFs) was about 4.0 GB, although `gzip -1` shrank one to about 40 MB.
The Houdini asset's *Restart JSON* writes one `state_{:d}.hdf5` per step, so
a 200-step run would need about 800 GB.

Cause (inspected with h5py on `outputs/ef-07/20260927T231825Z/r0-fw-repro/out/state_3.hdf5`
in the parent workspace): `io::write_matrix(path, key, mat, replace)` called
h5pp's `writeDataset` with no layout. For anything over 512 kB, h5pp picks a
chunked layout, and it guesses square chunks (256 × 256 for a 2D array).
Uncompressed chunks are allocated in full, so each 652,440 × 1 column (`u`,
`v`, `a`) took 2,549 chunks × 512 KiB = 1.34 GB of storage for 5.2 MB of
data. The contact controller's tables, added in `c133948cf`, are small enough
that h5pp stores them compact or contiguous (≤ 64 kB each), so they were not
the problem. This is the same h5pp default that the VTU writer's patch
(`cmake/recipes/patches/paraviewo-hdf5-row-chunks.patch`) fixed for N × 3
fields.

## Change

`src/polyfem/io/MatrixIO.cpp`: when h5pp would store a dataset chunked (≥
512 kB), `write_matrix(path, key, mat, replace)` now passes chunk dimensions
made of whole rows of about 1 MiB (131,072 rows for a column). Nothing else
changes:

* smaller datasets keep h5pp's own compact or contiguous layout;
* chunked datasets keep unlimited maximum dimensions and h5pp's resize policy;
* when the dataset already exists, it keeps its own layout (h5pp applies
  creation options only when it creates a dataset);
* compression stays at the file's level, which is off by default. An
  `h5pp::Options` with no level would otherwise become 0, which attaches a
  deflate filter at level 0: it stores data uncompressed but still runs the
  filter;
* the reader is unchanged.

Every HDF5 matrix written through this function benefits: restart states
(`ImplicitTimeIntegrator::save_state`, contact/friction restart state), the
FSI state, and `NonlinearElasticVarForm`'s `u` export. No dependency patch is
needed.

## Validation

Evidence: `outputs/state-hdf5-chunks/20260927T233458Z/` in the parent
workspace. It holds the isolated worktree and build, the two binaries with
`binary.sha256`, `run.sh`, `matrix2.sh`, `compare.py` and the logs. The
baseline binary was built from the same tree with only `MatrixIO.cpp`
reverted to `de95980c9`. The resume matrix below ran on `de95980c9` plus the
first version of the change, before the compression line was added; that
version wrote deflate level 0, which is lossless. The published commit is
rebased onto `6a4e788bf` (trim band statistic weighting, PolySolve
`6a8c2cc9`) and was re-checked on its own build (`PolyFEM_bin-final`, last
section).

**Regression test.** `[restart][matrix_io]` "state file size stays near its
payload" writes a 652,440 × 1 and a 652,440 × 2 matrix (the ball-burst sizes
for Implicit Euler and BDF2) and a 779 × 6 table. It requires:

* the file to stay under 1.1 × payload + 1 MiB;
* chunk dimensions of {131072, 1} and {65536, 2};
* no filter on either large dataset;
* every value to read back exactly.

Negative controls: with `MatrixIO.cpp` at `HEAD` the file is 2,673,102,008
bytes for a 15.7 MB payload and the size check fails. Without the
compression line, both filter checks fail (one deflate filter each).

**Resume matrix.** Each case runs 6 steps straight through (A) and 3 steps
plus a resume from the written `restart_3.json`, launched from an unrelated
folder (B). Runs are single-threaded, on the semi-implicit transient smoke
geometry of the restart record. "r3" cases refine the cube three times
(35,937 nodes, above h5pp's 512 kB chunking threshold) and use
`Eigen::CholmodSupernodalLLT` for both binaries, because SimplicialLDLT took
about 3 minutes per solve.

| Case | A: baseline = fixed | max \|A − B\| baseline / fixed | state_3 bytes, baseline → fixed | u chunks, baseline → fixed |
| --- | --- | --- | --- | --- |
| IE no contact | byte-identical | 1.0e-15 / 1.0e-15 | 10,944 → 10,944 | contiguous (unchanged) |
| IE semi-implicit contact | byte-identical | 1.1e-16 / 1.1e-16 | 23,864 → 23,864 | unchanged |
| BDF2 contact | byte-identical | 1.0e-15 / 1.0e-15 | 33,152 → 33,152 | unchanged |
| Quasistatic contact | byte-identical | 1.0e-15 / 1.0e-15 | 23,864 → 23,864 | unchanged |
| IE contact + friction 0.3 | byte-identical | 1.0e-15 / 1.0e-15 | 24,800 → 24,800 | unchanged |
| BDF2 contact + friction 0.3 | byte-identical | 2.7e-15 / 2.7e-15 | 34,080 → 34,080 | unchanged |
| IE classic adaptive stiffness | byte-identical | 1.0e-15 / 1.0e-15 | 11,528 → 11,528 | unchanged |
| r3 IE no contact (107,811 DOFs) | byte-identical | 6.6e-14 / 6.6e-14 | 663,821,184 → 2,597,484 | (256, 256) → (107811, 1) |
| r3 IE contact + friction 0.3 (107,823 DOFs) | byte-identical | 3.2e-4 / 3.2e-4 | 664,076,880 → 2,853,468 | (256, 256) → (107823, 1) |

The refined states hold 3 × 0.86 MB of history; the new files are within
10 % of that (the contact and friction tables make up the rest).

**Read compatibility.** The fixed binary resumed from the *baseline's*
`restart_3.json`, whose state file has the old 256 × 256 chunk layout. Its
result is byte-identical to the baseline's own resume in both refined cases.
The restore path logs "restart state restored" for the contact and friction
forms. Existing state files therefore still resume.

**Pre-existing, not caused by this change:** the refined friction case
resumes 3.2e-4 away from the uninterrupted run, identically with both
binaries and in the cross-resume. The restart record measured roundoff
(1e-15) on the unrefined scene. On the refined scene the lagged friction does
not converge in any step: "Lagging failed to converge with 2 iteration(s)"
reports grad norms from 25 to 158 against a tolerance of 2e-3, and CHOLMOD
reports indefinite matrices. The cause of the resume difference was not
investigated; it is recorded here as an open observation.

**Published commit (rebased, compression pinned), `PolyFEM_bin-final`:**

* The refined no-contact state file is 2,597,256 bytes. `u` has chunks
  (107811, 1), no filter, and storage 862,488 bytes, exactly its payload.
* The solution is byte-identical to the first fixed binary, and the resume
  differs from the uninterrupted run by 6.6e-14.
* The resume from the baseline's old-layout state file is byte-identical to
  the baseline's resume.
* Small IE contact + friction resumes to 3.3e-15. Its solution differs from
  the pre-rebase runs, as expected from `6a4e788bf`'s change to the trim
  band statistic.
* `[matrix_io],[restart]` pass: 3 cases, 65 assertions on the rebased build
  before the compression line.
* The output/restart/contact selection
  (`unit-selection-tags.txt` = the restart record's selection plus
  `[matrix_io]`) passes: 96 cases, 11,288 assertions
  (`unit-selection-final.log`). Before the rebase it was 95 cases and 11,390
  assertions.
* All 13 Houdini asset test scripts pass against this binary
  (`hda-tests-final.log`) and against the pre-rebase fixed binary
  (`hda-tests.log`). They ran from copies pointed at the evidence binary,
  because the shared `polyfem/build` was running another session's full
  suite.

**Expected effect on the ball-burst scene:** its `u`, `v` and `a` go from
1.34 GB of storage each to about 5.2 MB (5 row chunks of 1 MiB). A state
file therefore goes from 4.0 GB to roughly 16 MB plus the contact tables.
That is about 3 GB for a 200-step *Restart JSON* run, instead of 800 GB. The
full scene was not rerun here (another session's run of it was using the
machine); the unit test writes its exact vector sizes.
