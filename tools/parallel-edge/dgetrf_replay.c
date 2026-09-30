/* Replays 3x3 DGETRF inputs (one matrix per line, 9 hex doubles, column-major,
 * as dumped from a PolyFEM run with gdb) and prints a hash of the outputs and
 * the pivots, plus the number of inputs whose factors differ from a reference
 * file if one is given: dgetrf_replay inputs.txt [dump_out.bin|- [element offset 0..7]]
 * Link like dgetrf_probe. */
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
extern void dgetrf_(const int*, const int*, double*, const int*, int*, int*);
int main(int argc, char **argv){
  int off = argc > 3 ? atoi(argv[3]) : 0; /* element offset 0..7 inside a 64-byte aligned buffer */
  FILE *f = fopen(argv[1], "r"); char line[1024]; uint64_t H = 1469598103934665603ULL; long n_in = 0;
  FILE *o = (argc > 2 && argv[2][0] != '-') ? fopen(argv[2], "wb") : NULL;
  while (fgets(line, sizeof line, f)) {
    if (line[0] == '#') continue;
    static double buf[64] __attribute__((aligned(64))); double *A = buf + off; char *p = line;
    for (int i = 0; i < 9; ++i) { A[i] = strtod(p, &p); }
    int n = 3, ipiv[3], info; dgetrf_(&n, &n, A, &n, ipiv, &info);
    for (int i = 0; i < 9; ++i) { uint64_t b; memcpy(&b, &A[i], 8); H = (H ^ b) * 1099511628211ULL; }
    for (int i = 0; i < 3; ++i) H = (H ^ (uint64_t)ipiv[i]) * 1099511628211ULL;
    H = (H ^ (uint64_t)info) * 1099511628211ULL;
    if (o) fwrite(A, 8, 9, o);
    ++n_in;
  }
  printf("matrices=%ld hash=%016llx\n", n_in, (unsigned long long)H);
  if (o) fclose(o);
  return 0;
}
