/* usage: dgetrf_probe [element offset 0..7 [general=1 uniform(-1,1); 2 sparse/rotation-like]]  (matrix placed at 8*offset bytes from a 64-byte boundary)
 * Counts how many 3x3 LU factorizations differ bitwise from a reference
 * (plain Doolittle with partial pivoting) and prints a hash of all results.
 * build: gcc -O1 dgetrf_probe.c -o dgetrf_probe -Wl,--start-group $MKL/libmkl_intel_lp64.a
 *        $MKL/libmkl_sequential.a $MKL/libmkl_core.a -Wl,--end-group -lpthread -lm -ldl
 * Compare the hash under different MKL_CBWR / qemu -cpu settings. */
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>
extern void dgetrf_(const int*, const int*, double*, const int*, int*, int*);
static uint64_t s = 88172645463325252ULL;
static double rnd(void){ s ^= s << 13; s ^= s >> 7; s ^= s << 17; return (double)(s >> 11) / 9007199254740992.0; }
/* rotation-like / sparse patterns: exact 0, +-1, tiny cos(90deg) and O(1) entries */
static double structured(int i){ double r = rnd(); if (r < 0.35) return 0.0; if (r < 0.5) return 1.0; if (r < 0.6) return -1.0; if (r < 0.7) return 6.123233995736766e-17 * (rnd() - 0.5); return 1.0 + 0.1 * (rnd() - 0.5); }
int main(int argc, char **argv){
  int off = argc > 1 ? atoi(argv[1]) : 0; int general = argc > 2 ? atoi(argv[2]) : 0; /* element offset (8 bytes each) inside a 64-byte aligned buffer */
  uint64_t H = 1469598103934665603ULL; long diff = 0, N = 200000;
  for (long it = 0; it < N; ++it) {
    static double buf[64] __attribute__((aligned(64)));
    double *A = buf + off, B[9];
    for (int i = 0; i < 9; ++i) A[i] = general == 2 ? structured(i) : general ? 2.0 * rnd() - 1.0 : (i % 4 == 0 ? 1.0 : 0.0) + 0.3 * (rnd() - 0.5);
    memcpy(B, A, sizeof A);
    int n = 3, ipiv[3], info;
    dgetrf_(&n, &n, A, &n, ipiv, &info);
    /* reference: column-major Doolittle with partial pivoting */
    int piv[3];
    for (int k = 0; k < 3; ++k) {
      int p = k; for (int i = k + 1; i < 3; ++i) if (fabs(B[i + 3*k]) > fabs(B[p + 3*k])) p = i;
      piv[k] = p;
      if (p != k) for (int j = 0; j < 3; ++j) { double t = B[k + 3*j]; B[k + 3*j] = B[p + 3*j]; B[p + 3*j] = t; }
      for (int i = k + 1; i < 3; ++i) B[i + 3*k] /= B[k + 3*k];
      for (int j = k + 1; j < 3; ++j) for (int i = k + 1; i < 3; ++i) B[i + 3*j] -= B[i + 3*k] * B[k + 3*j];
    }
    if (memcmp(A, B, sizeof A)) ++diff;
    for (int i = 0; i < 9; ++i) { uint64_t b; memcpy(&b, &A[i], 8); H = (H ^ b) * 1099511628211ULL; }
  }
  printf("offset_bytes=%d differs_from_plain_doolittle=%ld/%ld hash=%016llx\n", off*8, diff, N, (unsigned long long)H);
  return 0;
}
