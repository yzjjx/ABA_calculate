#ifndef ABA_BATCH_H
#define ABA_BATCH_H

#include <ABA_HLS.h>

// Fixed-size deployment wrapper used to amortize control and DMA overhead for
// the 1000-state benchmark.  The single-state ABA() remains available as a top.
constexpr int ABA_BATCH_SIZE = 1000;

void ABA_batch(
    const data_t q[ABA_BATCH_SIZE][DOF],
    const data_t dq[ABA_BATCH_SIZE][DOF],
    const data_t tau[ABA_BATCH_SIZE][DOF],
    data_t ddq[ABA_BATCH_SIZE][DOF]
);

#endif
