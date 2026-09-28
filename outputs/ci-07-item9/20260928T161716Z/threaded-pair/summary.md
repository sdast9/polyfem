# RB-12 repeatability matrix

binary `/home/user/polyfem/build-cloud/PolyFEM_bin` sha256 `67ae71090ecae5f3171b1385c9fb8e31e3b576a429df70ca5bdf72f31c205713`; repeats 2; roundoff rule 1e-12

| cell | completed | distinct histories | same-history max ‖du‖ | branch divergences (first differing step, max ‖du‖) | violations | wall s (min–max) | peak RSS MB (min–max) |
| --- | --- | --- | --- | --- | --- | --- | --- |
| quasistatic-semi--threads-default | 2/2 | 1 | 0.00e+00 | none | 0 | 9.5–10.2 | 180–196 |

## Discrete state spread across repeats (min–max per step)

| cell | step | active | continued | candidates (last) | iterations | trim |
| --- | --- | --- | --- | --- | --- | --- |
| quasistatic-semi--threads-default | 1 | 45–46 | 0 | 754 | 9 | 2 |
| quasistatic-semi--threads-default | 2 | 46–47 | 45–46 | 512 | 6 | 4 |
| quasistatic-semi--threads-default | 3 | 45–46 | 46–47 | 406 | 6 | 8 |
| quasistatic-semi--threads-default | 4 | 46–47 | 45–46 | 389 | 5 | 8 |

(only steps where some repeat differs are listed)

## Serial versus threaded

| fixture | pairs | same-history pairs | max ‖du‖ same history | max ‖du‖ different history |
| --- | --- | --- | --- | --- |
