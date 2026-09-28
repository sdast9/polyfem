# RB-12 repeatability matrix

binary `/home/user/polyfem/build-cloud/PolyFEM_bin` sha256 `67ae71090ecae5f3171b1385c9fb8e31e3b576a429df70ca5bdf72f31c205713`; repeats 3; roundoff rule 1e-12

| cell | completed | distinct histories | same-history max ‖du‖ | branch divergences (first differing step, max ‖du‖) | violations | wall s (min–max) | peak RSS MB (min–max) |
| --- | --- | --- | --- | --- | --- | --- | --- |
| quasistatic-semi--threads-1 | 3/3 | 1 | 0.00e+00 | none | 0 | 10.8–11.2 | 175–175 |

## Discrete state spread across repeats (min–max per step)

| cell | step | active | continued | candidates (last) | iterations | trim |
| --- | --- | --- | --- | --- | --- | --- |

(only steps where some repeat differs are listed)

## Serial versus threaded

| fixture | pairs | same-history pairs | max ‖du‖ same history | max ‖du‖ different history |
| --- | --- | --- | --- | --- |
