<h1 align="center">
<a href="https://polyfem.github.io/"><img alt="polyfem" src="https://polyfem.github.io/img/polyfem.png" width="60%"></a>
</h1><br>

[![Build](https://github.com/polyfem/polyfem/actions/workflows/continuous.yml/badge.svg?label=test)](https://github.com/polyfem/polyfem/actions/workflows/continuous.yml)
[![codecov](https://codecov.io/github/polyfem/polyfem/graph/badge.svg?token=ZU9KLLTTDT)](https://codecov.io/github/polyfem/polyfem)
[![Nightly](https://github.com/polyfem/polyfem/actions/workflows/nightly.yml/badge.svg)](https://github.com/polyfem/polyfem/actions/workflows/nightly.yml)
[![Docs](https://github.com/polyfem/polyfem/actions/workflows/docs.yml/badge.svg)](https://polyfem.github.io/polyfem)

PolyFEM is a polyvalent C++ FEM library.

### sdast9 fork

State as of **September 15, 2026** (`main`). The dated project state lives in
the parent workspace's README; this section keeps the repository's own claims
current.

- **Continuous integration** runs on this fork:
  [Build](https://github.com/sdast9/polyfem/actions/workflows/continuous.yml)
  (Linux GCC 13, macOS AppleClang 21, Windows MSVC 19.44; Debug and Release)
  and [pre-commit](https://github.com/sdast9/polyfem/actions/workflows/pre-commit.yml).
  The badges above are upstream's and say nothing about this fork. The
  required lanes are pre-commit, Linux Release and macOS Release (user decision
  2026-09-14). Measured at `fffc722b9` (RB-12 stage 3, run 34903453744):
  pre-commit green; Linux Debug, macOS Debug, Windows Debug and Windows
  Release green; Linux Release 340/344 and macOS Release 339/343, where the
  only failures are the four scene groups `standard`, `contact_2d`,
  `contact_3d` and `triangle_data` tracked as
  [CI-03–CI-06](docs/ci-portability-plan.md#4-scene-failures-requiring-distinct-treatment).
  Every `PolyFEM_bin` run writes a `run-manifest.json`
  ([schema](scenes/semi-implicit/README.md#run-manifest-rb-12)) and
  `PolyFEM_bin --build_info` prints the compiled-in build identity; CTest
  `cli_contract` checks both through the real executable on every lane.
- **Dependencies:** the recipes pin `sdast9/ipc-toolkit@c24d803e`
  (branch `semi-implicit-stiffness`) and `sdast9/polysolve@bce32a39` (branch
  `iteration-callback`); both forks' `Build` workflows run on those branches
  (PolySolve 6/6 lanes green; IPC Toolkit green on Linux/macOS, its two Windows
  lanes failing on upstream code — CI-07). A run's manifest records the
  effective sources next to the declared pins; verify local overrides before
  trusting a build. PolyFEM incorporates upstream through `6c9e7a390`
  (September 2); the September 5 `sync-upstream` promotion is historical.
- **Robustness work:** bounded sessions follow the
  **[RB robustness plan](docs/robustness-plan.md)**, whose status table is the
  authority on what is validated, characterized, closed or not started, and
  the [validation-record template](docs/robustness-record-template.md). The
  September 13 GitHub failure diagnosis and the remaining cross-platform work
  are the **[CI portability plan](docs/ci-portability-plan.md)** with its
  [evidence record](docs/ci-portability-evidence-2026-09-13.md). The
  [historical PF plan](docs/correctness-remediation-plan.md) retains the phase
  invariants and completion records: PF-01 and PF-03–PF-07 corrections are
  implemented and the constraint-floor barrier deletion/projection was removed
  on 2026-09-07 (old `constraint_floor` values are ignored compatibility data;
  positive values warn); CCD and the semi-implicit trial-displacement cap
  remain; PF-09 hard contact is deferred. See also
  [floor retirement](docs/pf-02-floor-removal.md),
  [semi-implicit usage](scenes/semi-implicit/README.md) and
  [VarForm architecture](docs/varform-design.md).
- **What is not claimed:** no full physical-accuracy or cross-platform
  certification. [PF-08](docs/pf-08-validation.md) records incomplete physical
  accounting, mesh sensitivity and intermittent solve failures; the RB records
  state their measured limits (for example RB-09's accuracy envelope and
  RB-12's threaded-friction repeatability of 1.7e-4). Packages, the Houdini
  launch paths on each OS and a cross-platform repeatability matrix are open
  CI-plan items.

Compilation
-----------

PolyFEM is tested on Windows, macOS, and Linux. A source build requires:

- CMake 3.25 or newer;
- a C++17 compiler;
- Git and an internet connection during the first CMake configuration, which downloads the C++ dependencies; and
- Python 3, including its development headers, unless Python expressions are disabled with `-DPOLYFEM_WITH_PYTHON=OFF`.

Ninja is optional, but recommended for consistent cross-platform build commands. Configure and build PolyFEM with:

    cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release
    cmake --build build

To build without Ninja, omit `-G Ninja`; CMake will select a generator available on the system.

On Linux, `zenity` is required for the file dialog window to work. On macOS and Windows, the native windows are used directly.

On macOS, the current CMake setup does not work with SuiteSparse installed via MacPorts. Please either use Homebrew or disable SPQR with `-DPOLYSOLVE_WITH_SPQR=OFF`.


### Optional

The formula for higher-order bases can be regenerated at CMake time using external Python scripts. Regeneration additionally requires:

- `numpy` and `sympy`
- `quadpy`

Usage
-----

The main executable, `./PolyFEM_bin`, can be called with a GUI or through a command-line interface. Simply run:

    ./PolyFEM_bin

A more detailed documentation can be found on the [website](https://polyfem.github.io/).

Documentation
-------------

The full documentation can be found at [https://polyfem.github.io/](https://polyfem.github.io/)

Community Projects
------------------

- [PolyFEM Blender Plugin](https://github.com/ETSim/PolyFEMBlenderPlugin) provides a community-maintained Blender interface for creating and running PolyFEM scenes.



License
-------

The code of PolyFEM itself is licensed under [MIT License](LICENSE). However, please be mindful of third-party libraries which are used by PolyFEM and may be available under a different license.

Citation
--------

If you use PolyFEM in your project, please consider citing our work:

```bibtex
@misc{polyfem,
  author = {Teseo Schneider and Jérémie Dumas and Xifeng Gao and Denis Zorin and Daniele Panozzo},
  title = {{Polyfem}},
  howpublished = "\url{https://polyfem.github.io/}",
  year = {2019},
}
```

```bibtex
@article{Schneider:2019:PFM,
  author = {Schneider, Teseo and Dumas, J{\'e}r{\'e}mie and Gao, Xifeng and Botsch, Mario and Panozzo, Daniele and Zorin, Denis},
  title = {Poly-Spline Finite-Element Method},
  journal = {ACM Trans. Graph.},
  volume = {38},
  number = {3},
  month = mar,
  year = {2019},
  url = {http://doi.acm.org/10.1145/3313797},
  publisher = {ACM}
}
```

```bibtex
@article{Schneider:2018:DSA,
    author = {Teseo Schneider and Yixin Hu and Jérémie Dumas and Xifeng Gao and Daniele Panozzo and Denis Zorin},
    journal = {ACM Transactions on Graphics},
    link = {},
    month = {10},
    number = {6},
    publisher = {Association for Computing Machinery (ACM)},
    title = {Decoupling Simulation Accuracy from Mesh Quality},
    volume = {37},
    year = {2018}
}
```

Acknowledgments & Funding
--------
The software is being developed in the [Geometric Computing Lab](https://cims.nyu.edu/gcl/index.html) at NYU Courant Institute of Mathematical Sciences and the University of Victoria, Canada.


This work was partially supported by:

* the NSF CAREER award 1652515
* NSF award 2053851, *Coordinated Advances in Reproductive Engineering for Health Research (CARE4HeR)*
* the NSF grant IIS-1320635
* the NSF grant DMS-1436591
* the NSF grant 1835712
* the SNSF grant P2TIP2_175859
* the NSERC grant RGPIN-2021-03707
* the NSERC grant DGECR-2021-00461
* Adobe Research
* nTopology
