/**
 * @file pass3.cpp
 * @brief 用于实现书本pass3前向递推过程
 *
 *
 * @author YZJ
 * @date 2026-05-22
 * @version t0.1
 */

#include <cmath>
#include <v_ori.h>
#include <cal_p.h>
#include <pass3.h>
#include <block_mat.h>

void tip_pass3(
    const data_t E[DOF][3][3],
    const data_t F[DOF][3][3],
    const data_t c[DOF][6],
    const data_t inv_D[DOF],
    const data_t u[DOF],
    const data_t U[DOF][6],
    data_t ddq[DOF]
)
{
#pragma HLS INLINE
#pragma HLS ARRAY_PARTITION variable=E complete dim=0
#pragma HLS ARRAY_PARTITION variable=F complete dim=0
#pragma HLS ARRAY_PARTITION variable=c complete dim=0
#pragma HLS ARRAY_PARTITION variable=inv_D complete dim=1
#pragma HLS ARRAY_PARTITION variable=u complete dim=1
#pragma HLS ARRAY_PARTITION variable=U complete dim=0
#pragma HLS ARRAY_PARTITION variable=ddq complete dim=1

    data_t a_parent[6] = {0, 0, 0, 0, 0, 9.81};

    data_t a_s[6];

#pragma HLS ARRAY_PARTITION variable=a_parent complete dim=1
#pragma HLS ARRAY_PARTITION variable=a_s complete dim=1

    for(int i = 0;i<DOF;i++)
    {
#pragma HLS UNROLL
        v_mat_cal(E[i], F[i], a_parent, a_s);

        for (int k = 0; k < 6; k++)
        {
#pragma HLS UNROLL
            a_s[k] += c[i][k];
        }

        const data_t Ua =
            U[i][0] * a_s[0] +
            U[i][1] * a_s[1] +
            U[i][2] * a_s[2] +
            U[i][3] * a_s[3] +
            U[i][4] * a_s[4] +
            U[i][5] * a_s[5];

        ddq[i] = (u[i]-Ua)*inv_D[i];

        a_s[2] += ddq[i];

        for(int k = 0;k<6;k++)
        {
#pragma HLS UNROLL
            a_parent[k] = a_s[k];
        }
    }
}
