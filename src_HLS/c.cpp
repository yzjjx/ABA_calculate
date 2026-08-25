/**
 * @file test_c.cpp
 * @brief 用于实现书本的pass1的公式五
 *
 *
 * @author YZJ
 * @date 2026-05-18
 * @version t0.1
 */

#include <cmath>
#include <v_ori.h>
#include <c.h>

// 因为在test_T_R_out已经定义，下面不再定义
// typedef float data_t;
// const int DOF = 3;

// 定义静态数组空间向量叉乘方式
void space_cross(
    const data_t in_1[6],
    const data_t in_2[6],
    data_t out[6]
)
{
#pragma HLS INLINE
#pragma HLS ARRAY_PARTITION variable=in_1 complete dim=1
#pragma HLS ARRAY_PARTITION variable=in_2 complete dim=1
#pragma HLS ARRAY_PARTITION variable=out complete dim=1

    out[0] = in_1[1]*in_2[2]-in_1[2]*in_2[1];
    out[1] = in_1[2]*in_2[0]-in_1[0]*in_2[2];
    out[2] = in_1[0]*in_2[1]-in_1[1]*in_2[0];

    out[3] = in_1[1]*in_2[5]-in_1[2]*in_2[4]+ 
             in_1[4]*in_2[2]-in_1[5]*in_2[1];
    out[4] = in_1[2]*in_2[3]-in_1[0]*in_2[5]+ 
             in_1[5]*in_2[0]-in_1[3]*in_2[2];
    out[5] = in_1[0]*in_2[4]-in_1[1]*in_2[3]+ 
             in_1[3]*in_2[1]-in_1[4]*in_2[0];

}

// 因为是转动副，所以S圈i为0,因此c的计算只有后面的叉乘，为6*1矩阵叉乘
void c_fina(
    const data_t v[DOF][6],
    const data_t dq[DOF],
    data_t c[DOF][6])
{
#pragma HLS INLINE
#pragma HLS ARRAY_PARTITION variable=v complete dim=0
#pragma HLS ARRAY_PARTITION variable=dq complete dim=1
#pragma HLS ARRAY_PARTITION variable=c complete dim=0

    for (int i = 0; i < DOF; i++)
    {
#pragma HLS UNROLL
        const data_t wi = dq[i];

        c[i][0] =  v[i][1] * wi;
        c[i][1] = -v[i][0] * wi;
        c[i][2] =  0.0f;

        c[i][3] =  v[i][4] * wi;
        c[i][4] = -v[i][3] * wi;
        c[i][5] =  0.0f;
    }
}
