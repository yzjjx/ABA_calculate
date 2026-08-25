/**
 * @file test_v_ori.cpp
 * @brief 用于实现速度的前向递推计算，主要到书本的pass1的公式四
 *
 *
 * @author YZJ
 * @date 2026-05-18
 * @version t0.1
 */

#include <cmath>
#include <T_R_out.h>
#include <ABA_parms.h>
#include <v_ori.h>
#include <block_mat.h>

// 因为在test_T_R_out已经定义，下面不再定义
// typedef float data_t;
// const int DOF = 3;

void v_fina(
    const data_t q[DOF],
    const data_t dq[DOF],
    data_t E[DOF][3][3],
    data_t F[DOF][3][3],
    data_t v[DOF][6]
)
{
#pragma HLS INLINE
#pragma HLS ARRAY_PARTITION variable=q complete dim=1
#pragma HLS ARRAY_PARTITION variable=dq complete dim=1
#pragma HLS ARRAY_PARTITION variable=E complete dim=0
#pragma HLS ARRAY_PARTITION variable=F complete dim=0
#pragma HLS ARRAY_PARTITION variable=v complete dim=0

    // E = R^T
    cal_R_Trans(q, E);

    for (int i = 0; i < DOF; i++)
    {
#pragma HLS UNROLL
        // 当前关节的紧凑空间变换左下角：
        // F = -R^T * p_cross
        const data_t px = P_const[i][0];
        const data_t py = P_const[i][1];
        const data_t pz = P_const[i][2];

        // 直接计算F，不再构造p_cross
        for (int r = 0; r < 3; r++)
        {
#pragma HLS UNROLL
            const data_t e0 = E[i][r][0];
            const data_t e1 = E[i][r][1];
            const data_t e2 = E[i][r][2];

            F[i][r][0] = e2 * py - e1 * pz;
            F[i][r][1] = e0 * pz - e2 * px;
            F[i][r][2] = e1 * px - e0 * py;
        }

        /*
         * 速度前向递推：
         *
         * omega_new = E * omega_parent
         * linear_new = F * omega_parent + E * linear_parent
         */

        if (i == 0)
        {
            v[i][0] = 0.0f;
            v[i][1] = 0.0f;
            v[i][2] = dq[i];
            v[i][3] = 0.0f;
            v[i][4] = 0.0f;
            v[i][5] = 0.0f;
        }
        else
        {
            v_mat_cal(
                E[i],
                F[i],
                v[i - 1],
                v[i]
            );

            // 添加当前关节自身速度S*dq
            v[i][2] += dq[i];
        }
    }
}
