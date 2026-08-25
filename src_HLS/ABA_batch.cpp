#include "ABA_batch.h"

void ABA_batch(
    const joint_pos_t q[ABA_BATCH_SIZE][DOF],
    const joint_vel_t dq[ABA_BATCH_SIZE][DOF],
    const joint_tau_t tau[ABA_BATCH_SIZE][DOF],
    data_t ddq[ABA_BATCH_SIZE][DOF]
)
{
    // Four independent AXI masters allow q, dq, tau and ddq traffic to overlap.
    // depth is expressed in scalar 32-bit fixed-point elements:
    // 1000 states * 6 joints.
#pragma HLS INTERFACE m_axi port=q offset=slave bundle=gmem_q depth=6000 \
    max_read_burst_length=64 num_read_outstanding=16
#pragma HLS INTERFACE m_axi port=dq offset=slave bundle=gmem_dq depth=6000 \
    max_read_burst_length=64 num_read_outstanding=16
#pragma HLS INTERFACE m_axi port=tau offset=slave bundle=gmem_tau depth=6000 \
    max_read_burst_length=64 num_read_outstanding=16
#pragma HLS INTERFACE m_axi port=ddq offset=slave bundle=gmem_ddq depth=6000 \
    max_write_burst_length=64 num_write_outstanding=16

#pragma HLS INTERFACE s_axilite port=q bundle=control
#pragma HLS INTERFACE s_axilite port=dq bundle=control
#pragma HLS INTERFACE s_axilite port=tau bundle=control
#pragma HLS INTERFACE s_axilite port=ddq bundle=control
#pragma HLS INTERFACE s_axilite port=return bundle=control

batch_loop:
    for (int sample = 0; sample < ABA_BATCH_SIZE; ++sample)
    {
        // Match the resource-balanced ABA core.  Requesting II=1 here would
        // encourage another excessive replication attempt around the core.
#pragma HLS PIPELINE II=12 rewind

        joint_pos_t q_local[DOF];
        joint_vel_t dq_local[DOF];
        joint_tau_t tau_local[DOF];
        data_t ddq_local[DOF];

#pragma HLS ARRAY_PARTITION variable=q_local complete dim=1
#pragma HLS ARRAY_PARTITION variable=dq_local complete dim=1
#pragma HLS ARRAY_PARTITION variable=tau_local complete dim=1
#pragma HLS ARRAY_PARTITION variable=ddq_local complete dim=1

    load_joints:
        for (int joint = 0; joint < DOF; ++joint)
        {
#pragma HLS UNROLL
            q_local[joint] = q[sample][joint];
            dq_local[joint] = dq[sample][joint];
            tau_local[joint] = tau[sample][joint];
        }

        ABA(q_local, dq_local, tau_local, ddq_local);

    store_joints:
        for (int joint = 0; joint < DOF; ++joint)
        {
#pragma HLS UNROLL
            ddq[sample][joint] = ddq_local[joint];
        }
    }
}
