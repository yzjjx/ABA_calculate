#include <cmath>
#include <c.h>
#include <v_ori.h>
#include <cal_p.h>

// 因为在test_T_R_out已经定义，下面不再定义
// typedef float data_t;
// const int DOF = 3;

// 外力为0,后面项为0，仅计算前面的空间叉乘

// 空间动量(h)计算
// 计算空间惯量矩阵
void h_fina(
    const data_t I_spa[DOF][6][6],
    const data_t v[DOF][6],
    data_t h[DOF][6]
)
{
#pragma HLS INLINE
#pragma HLS ARRAY_PARTITION variable=I_spa complete dim=0
#pragma HLS ARRAY_PARTITION variable=v complete dim=0
#pragma HLS ARRAY_PARTITION variable=h complete dim=0

    // v_fina(alpha,a,d,q,dq,X_lam,v);

    for (int i = 0; i < DOF; i++)
    {
#pragma HLS UNROLL
        h[i][0] = I_spa[i][0][0]*v[i][0]+
                  I_spa[i][0][1]*v[i][1]+
                  I_spa[i][0][2]*v[i][2]+
                  I_spa[i][0][3]*v[i][3]+
                  I_spa[i][0][4]*v[i][4]+
                  I_spa[i][0][5]*v[i][5];

        h[i][1] = I_spa[i][1][0]*v[i][0]+
                  I_spa[i][1][1]*v[i][1]+
                  I_spa[i][1][2]*v[i][2]+
                  I_spa[i][1][3]*v[i][3]+
                  I_spa[i][1][4]*v[i][4]+
                  I_spa[i][1][5]*v[i][5];

        h[i][2] = I_spa[i][2][0]*v[i][0]+
                  I_spa[i][2][1]*v[i][1]+
                  I_spa[i][2][2]*v[i][2]+
                  I_spa[i][2][3]*v[i][3]+
                  I_spa[i][2][4]*v[i][4]+
                  I_spa[i][2][5]*v[i][5];

        h[i][3] = I_spa[i][3][0]*v[i][0]+
                  I_spa[i][3][1]*v[i][1]+
                  I_spa[i][3][2]*v[i][2]+
                  I_spa[i][3][3]*v[i][3]+
                  I_spa[i][3][4]*v[i][4]+
                  I_spa[i][3][5]*v[i][5];

        h[i][4] = I_spa[i][4][0]*v[i][0]+
                  I_spa[i][4][1]*v[i][1]+
                  I_spa[i][4][2]*v[i][2]+
                  I_spa[i][4][3]*v[i][3]+
                  I_spa[i][4][4]*v[i][4]+
                  I_spa[i][4][5]*v[i][5];

        h[i][5] = I_spa[i][5][0]*v[i][0]+
                  I_spa[i][5][1]*v[i][1]+
                  I_spa[i][5][2]*v[i][2]+
                  I_spa[i][5][3]*v[i][3]+
                  I_spa[i][5][4]*v[i][4]+
                  I_spa[i][5][5]*v[i][5];
    }
    
}

// 空间力叉乘算法
void f_space_cross(
    const data_t in_1[6],
    const data_t in_2[6],
    data_t out[6]
)
{
#pragma HLS INLINE
#pragma HLS ARRAY_PARTITION variable=in_1 complete dim=1
#pragma HLS ARRAY_PARTITION variable=in_2 complete dim=1
#pragma HLS ARRAY_PARTITION variable=out complete dim=1

    out[0] = in_1[1]*in_2[2]-in_1[2]*in_2[1]+
             in_1[4]*in_2[5]-in_1[5]*in_2[4];
    out[1] = in_1[2]*in_2[0]-in_1[0]*in_2[2]+
             in_1[5]*in_2[3]-in_1[3]*in_2[5];
    out[2] = in_1[0]*in_2[1]-in_1[1]*in_2[0]+
             in_1[3]*in_2[4]-in_1[4]*in_2[3];

    out[3] = in_1[1]*in_2[5]-in_1[2]*in_2[4];
    out[4] = in_1[2]*in_2[3]-in_1[0]*in_2[5];
    out[5] = in_1[0]*in_2[4]-in_1[1]*in_2[3];
}

// 空间叉乘计算
void p_fina(
    const data_t v[DOF][6],
    const data_t h[DOF][6],
    data_t p[DOF][6]
)
{
#pragma HLS INLINE
#pragma HLS ARRAY_PARTITION variable=v complete dim=0
#pragma HLS ARRAY_PARTITION variable=h complete dim=0
#pragma HLS ARRAY_PARTITION variable=p complete dim=0

    for(int i = 0;i<DOF;i++)
    {
#pragma HLS UNROLL
        f_space_cross(v[i],h[i],p[i]);
    }  
}
