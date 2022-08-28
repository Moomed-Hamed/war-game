#ifndef BT_GEN_RANDOM_H
#define BT_GEN_RANDOM_H

#include <stdlib.h>

#define GEN_RAND_MAX RAND_MAX

SIMD_FORCE_INLINE void GEN_srand(unsigned int seed) { srand(seed); }
SIMD_FORCE_INLINE unsigned int GEN_rand() { return rand(); }

#endif  //BT_GEN_RANDOM_H
