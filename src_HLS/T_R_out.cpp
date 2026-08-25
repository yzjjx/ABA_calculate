#include <hls_math.h>

#ifdef ABA_NATIVE_SIM
#include <cmath>
#endif

#include "ABA_fixed.h"

void cal_R_Trans(
    const joint_pos_t q[DOF],
    transform_t R_Trans[DOF][3][3])
{
#pragma HLS INLINE off
#pragma HLS ARRAY_PARTITION variable=q complete dim=1
#pragma HLS ARRAY_PARTITION variable=R_Trans complete dim=0

    for (int i = 0; i < DOF; i++)
    {
        // Roll the six joints through one fixed-point CORDIC sin/cos pipeline.
#pragma HLS PIPELINE II=2
        const angle_t q_angle = q[i];
        trig_t sq_fixed;
        trig_t cq_fixed;
#ifdef ABA_NATIVE_SIM
        // Native compiler fallback used only by fixed_accuracy_test. The
        // results are quantized to the exact CORDIC output type, so the rest
        // of the datapath is bit-accurate fixed point. HLS builds use the
        // fixed-point hls::sincos implementation below.
        sq_fixed = std::sin(q_angle.to_double());
        cq_fixed = std::cos(q_angle.to_double());
#else
        hls::sincos(q_angle, &sq_fixed, &cq_fixed);
#endif

        const transform_t cq = cq_fixed;
        const transform_t sq = sq_fixed;

        const transform_t ca = alpha_cos[i];
        const transform_t sa = alpha_sin[i];

        R_Trans[i][0][0] = cq;
        R_Trans[i][0][1] = sq * ca;
        R_Trans[i][0][2] = sq * sa;

        R_Trans[i][1][0] = -sq;
        R_Trans[i][1][1] = cq * ca;
        R_Trans[i][1][2] = cq * sa;

        R_Trans[i][2][0] = 0.0f;
        R_Trans[i][2][1] = -sa;
        R_Trans[i][2][2] = ca;
    }
}
