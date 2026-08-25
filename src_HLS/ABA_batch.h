#ifndef ABA_BATCH_H
#define ABA_BATCH_H

#include "ABA_fixed.h"

// Fixed-size deployment wrapper used to amortize control and DMA overhead for
// the 1000-state benchmark.  The single-state ABA() remains available as a top.
void ABA_batch(
    const joint_pos_t q[ABA_BATCH_SIZE][DOF],
    const joint_vel_t dq[ABA_BATCH_SIZE][DOF],
    const joint_tau_t tau[ABA_BATCH_SIZE][DOF],
    data_t ddq[ABA_BATCH_SIZE][DOF]
);

#endif
