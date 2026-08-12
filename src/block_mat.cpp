#include <block_mat.h>

// 用来进行（空间）速度矩阵的计算，使用了空间叉乘矩阵
void v_mat_cal(
    const data_t E[3][3],
    const data_t F[3][3],
    const data_t in[6],
    data_t out[6])
{
    for (int r = 0; r < 3; r++)
    {
        // 上半部分：E * 角速度
        out[r] =
            E[r][0] * in[0] +
            E[r][1] * in[1] +
            E[r][2] * in[2];

        // 下半部分：F * 角速度 + E * 线速度
        out[r + 3] =
            F[r][0] * in[0] +
            F[r][1] * in[1] +
            F[r][2] * in[2] +
            E[r][0] * in[3] +
            E[r][1] * in[4] +
            E[r][2] * in[5];
    }
}

// 用来进行空间力矩阵的块矩阵计算
void f_mat_cal(
    const data_t E[3][3],
    const data_t F[3][3],
    const data_t in[6],
    data_t out[6])
{
    for (int r = 0; r < 3; r++)
    {
        out[r] =
            E[0][r] * in[0] +
            E[1][r] * in[1] +
            E[2][r] * in[2] +
            F[0][r] * in[3] +
            F[1][r] * in[4] +
            F[2][r] * in[5];

        out[r + 3] =
            E[0][r] * in[3] +
            E[1][r] * in[4] +
            E[2][r] * in[5];
    }
}