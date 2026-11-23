# Analytical contact and elastic tangent checks

Run the Catch2 cases tagged `[physics_oracle]` through the normal unit-test
executable. Both cases build their small meshes in memory. They require no
reference-data checkout.

## Frozen semi-implicit contact

Two vertices have an initial relative vector of (0.3, 0.4) m and a barrier
support of 0.75 m. The driving Hessian has diagonal entries
(40, 60, 80, 100) N/m and inter-node couplings of -5 N/m in x and -7 N/m
in y. With the initial normal (0.6, 0.8), the normal Rayleigh quotient is
`[0.36(40+80+10) + 0.64(60+100+14)] / 2 = 79.08 N/m`.
Form weight 2.4 and trim 1.7 give the total energy coefficient
`1.7 × 79.08 / 0.75² = 238.997333333 N/m³`.

For relative position `r`, write `s = r·r`, `h = 0.75²`, and
`B(s) = -(s-h)² log(s/h)` inside the support. The test differentiates
this scalar directly. The relative gradient is `2 k B' r` and its
Jacobian is `2 k B' I + 4 k B'' r rᵀ`. The pair assembly follows from
the relative-coordinate Jacobian `[-I I]`. This predicts both force
components, all tangent entries, the zero net force and moment, and the
negative transverse eigenvalue. A second comparison clamps just that
eigenvalue to test the production PSD projection. The test also checks
the half share of energy assigned to each node.

The second active sample, (0.4, 0.25) m, changes the current contact
direction while retaining the original stiffness. The pair energies
are 18.926773359 J and 25.624061369 J. The first force on the second
vertex is (128.693809698, 171.591746264) N. A third sample lies outside
support and produces zero energy, force and tangent. The absolute
component tolerances range from 1e-12 for force and moment balance to
1e-9 for the tangent. They cover the double-precision arithmetic, including
the eigensolver used for PSD projection.

## Time-varying Hooke tangent on one ElasticForm

A unit tetrahedron has an apex x displacement of 0.01 m, so its
engineering XZ shear is 0.01. The material rotates about z by `pi*t/2`.
Even seconds use the ZX modulus of 12 kPa and odd seconds use the YZ
modulus of 5 kPa. The expected apex tangent is `G/6`, or
2,000 and 833.333333333 N/m. The full tangent action and gradient follow
from the nodal virtual work of the shear stress against the four known
shape gradients. The expected energies are 0.1 J and 0.041666666667 J.

The same form is evaluated at times 0, 1, 2, and 1 again. This checks the
tangent used by nonlinear solves directly. A force-driven endpoint alone
can converge with a stale tangent. The absolute tolerances are 1e-12 J,
1e-10 N and 1e-8 N/m.
