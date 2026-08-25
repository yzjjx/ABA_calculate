//计算空间惯量矩阵，但是空间惯量矩阵为常数，因此这个最终不参加HLS综合
#include "ABA_fixed.h"

// 计算空间惯量矩阵
void I_space(
    const data_t mass[DOF],
    const data_t c_of_mass[DOF][3],
    const data_t I[DOF][9],//行展开矩阵
    inertia_t I_spa[DOF][6][6]
)
{
    // 计算质心叉乘矩阵
    data_t c_of_mass_cross[DOF][3][3];
    for(int i = 0;i < DOF;i++)
    {
        c_of_mass_cross[i][0][0] = 0;
        c_of_mass_cross[i][0][1] = -c_of_mass[i][2];
        c_of_mass_cross[i][0][2] = c_of_mass[i][1];

        c_of_mass_cross[i][1][0] = c_of_mass[i][2];
        c_of_mass_cross[i][1][1] = 0;
        c_of_mass_cross[i][1][2] = -c_of_mass[i][0];
    
        c_of_mass_cross[i][2][0] = -c_of_mass[i][1];
        c_of_mass_cross[i][2][1] = c_of_mass[i][0];
        c_of_mass_cross[i][2][2] = 0;
    }

    // 计算平行轴修正项（空间惯量矩阵左上角部分项）
    data_t corr_1_1[DOF][3][3];
    for(int i = 0;i < DOF;i++)
    {
        corr_1_1[i][0][0] = c_of_mass_cross[i][0][0]*c_of_mass_cross[i][0][0] + c_of_mass_cross[i][0][1]*c_of_mass_cross[i][0][1] + c_of_mass_cross[i][0][2]*c_of_mass_cross[i][0][2];
        corr_1_1[i][0][1] = c_of_mass_cross[i][0][0]*c_of_mass_cross[i][1][0] + c_of_mass_cross[i][0][1]*c_of_mass_cross[i][1][1] + c_of_mass_cross[i][0][2]*c_of_mass_cross[i][1][2];
        corr_1_1[i][0][2] = c_of_mass_cross[i][0][0]*c_of_mass_cross[i][2][0] + c_of_mass_cross[i][0][1]*c_of_mass_cross[i][2][1] + c_of_mass_cross[i][0][2]*c_of_mass_cross[i][2][2];

        corr_1_1[i][1][0] = c_of_mass_cross[i][1][0]*c_of_mass_cross[i][0][0] + c_of_mass_cross[i][1][1]*c_of_mass_cross[i][0][1] + c_of_mass_cross[i][1][2]*c_of_mass_cross[i][0][2];
        corr_1_1[i][1][1] = c_of_mass_cross[i][1][0]*c_of_mass_cross[i][1][0] + c_of_mass_cross[i][1][1]*c_of_mass_cross[i][1][1] + c_of_mass_cross[i][1][2]*c_of_mass_cross[i][1][2];
        corr_1_1[i][1][2] = c_of_mass_cross[i][1][0]*c_of_mass_cross[i][2][0] + c_of_mass_cross[i][1][1]*c_of_mass_cross[i][2][1] + c_of_mass_cross[i][1][2]*c_of_mass_cross[i][2][2];

        corr_1_1[i][2][0] = c_of_mass_cross[i][2][0]*c_of_mass_cross[i][0][0] + c_of_mass_cross[i][2][1]*c_of_mass_cross[i][0][1] + c_of_mass_cross[i][2][2]*c_of_mass_cross[i][0][2];
        corr_1_1[i][2][1] = c_of_mass_cross[i][2][0]*c_of_mass_cross[i][1][0] + c_of_mass_cross[i][2][1]*c_of_mass_cross[i][1][1] + c_of_mass_cross[i][2][2]*c_of_mass_cross[i][1][2];
        corr_1_1[i][2][2] = c_of_mass_cross[i][2][0]*c_of_mass_cross[i][2][0] + c_of_mass_cross[i][2][1]*c_of_mass_cross[i][2][1] + c_of_mass_cross[i][2][2]*c_of_mass_cross[i][2][2];


        corr_1_1[i][0][0] = mass[i]*corr_1_1[i][0][0];
        corr_1_1[i][0][1] = mass[i]*corr_1_1[i][0][1];
        corr_1_1[i][0][2] = mass[i]*corr_1_1[i][0][2];

        corr_1_1[i][1][0] = mass[i]*corr_1_1[i][1][0];
        corr_1_1[i][1][1] = mass[i]*corr_1_1[i][1][1];
        corr_1_1[i][1][2] = mass[i]*corr_1_1[i][1][2];

        corr_1_1[i][2][0] = mass[i]*corr_1_1[i][2][0];
        corr_1_1[i][2][1] = mass[i]*corr_1_1[i][2][1];
        corr_1_1[i][2][2] = mass[i]*corr_1_1[i][2][2];
    }

    // 计算空间惯量矩阵左上角
    data_t I_1_1[DOF][3][3];
    for(int i = 0;i < DOF;i++)
    {
        I_1_1[i][0][0] = I[i][0] + corr_1_1[i][0][0];
        I_1_1[i][0][1] = I[i][1] + corr_1_1[i][0][1];
        I_1_1[i][0][2] = I[i][2] + corr_1_1[i][0][2];

        I_1_1[i][1][0] = I[i][3] + corr_1_1[i][1][0];
        I_1_1[i][1][1] = I[i][4] + corr_1_1[i][1][1];
        I_1_1[i][1][2] = I[i][5] + corr_1_1[i][1][2];

        I_1_1[i][2][0] = I[i][6] + corr_1_1[i][2][0];
        I_1_1[i][2][1] = I[i][7] + corr_1_1[i][2][1];
        I_1_1[i][2][2] = I[i][8] + corr_1_1[i][2][2];
    }

    // 计算空间惯量矩阵右上角
    data_t I_1_2[DOF][3][3];
    for(int i = 0;i<DOF;i++)
    {
        I_1_2[i][0][0] = mass[i]*c_of_mass_cross[i][0][0];
        I_1_2[i][0][1] = mass[i]*c_of_mass_cross[i][0][1];
        I_1_2[i][0][2] = mass[i]*c_of_mass_cross[i][0][2];

        I_1_2[i][1][0] = mass[i]*c_of_mass_cross[i][1][0];
        I_1_2[i][1][1] = mass[i]*c_of_mass_cross[i][1][1];
        I_1_2[i][1][2] = mass[i]*c_of_mass_cross[i][1][2];

        I_1_2[i][2][0] = mass[i]*c_of_mass_cross[i][2][0];
        I_1_2[i][2][1] = mass[i]*c_of_mass_cross[i][2][1];
        I_1_2[i][2][2] = mass[i]*c_of_mass_cross[i][2][2];
    }

    // 计算空间惯量矩阵左下角
    data_t I_2_1[DOF][3][3];
    for(int i = 0;i<DOF;i++)
    {
        I_2_1[i][0][0] = mass[i]*c_of_mass_cross[i][0][0];
        I_2_1[i][0][1] = mass[i]*c_of_mass_cross[i][1][0];
        I_2_1[i][0][2] = mass[i]*c_of_mass_cross[i][2][0];

        I_2_1[i][1][0] = mass[i]*c_of_mass_cross[i][0][1];
        I_2_1[i][1][1] = mass[i]*c_of_mass_cross[i][1][1];
        I_2_1[i][1][2] = mass[i]*c_of_mass_cross[i][2][1];

        I_2_1[i][2][0] = mass[i]*c_of_mass_cross[i][0][2];
        I_2_1[i][2][1] = mass[i]*c_of_mass_cross[i][1][2];
        I_2_1[i][2][2] = mass[i]*c_of_mass_cross[i][2][2];
    }

    // 空间惯量矩阵生成
    for(int i = 0;i<DOF;i++)
    {
        I_spa[i][0][0] = I_1_1[i][0][0];
        I_spa[i][0][1] = I_1_1[i][0][1];
        I_spa[i][0][2] = I_1_1[i][0][2];
        I_spa[i][0][3] = I_1_2[i][0][0];
        I_spa[i][0][4] = I_1_2[i][0][1];
        I_spa[i][0][5] = I_1_2[i][0][2];

        I_spa[i][1][0] = I_1_1[i][1][0];
        I_spa[i][1][1] = I_1_1[i][1][1];
        I_spa[i][1][2] = I_1_1[i][1][2];
        I_spa[i][1][3] = I_1_2[i][1][0];
        I_spa[i][1][4] = I_1_2[i][1][1];
        I_spa[i][1][5] = I_1_2[i][1][2];

        I_spa[i][2][0] = I_1_1[i][2][0];
        I_spa[i][2][1] = I_1_1[i][2][1];
        I_spa[i][2][2] = I_1_1[i][2][2];
        I_spa[i][2][3] = I_1_2[i][2][0];
        I_spa[i][2][4] = I_1_2[i][2][1];
        I_spa[i][2][5] = I_1_2[i][2][2];

        I_spa[i][3][0] = I_2_1[i][0][0];
        I_spa[i][3][1] = I_2_1[i][0][1];
        I_spa[i][3][2] = I_2_1[i][0][2];
        I_spa[i][3][3] = mass[i];
        I_spa[i][3][4] = 0;
        I_spa[i][3][5] = 0;

        I_spa[i][4][0] = I_2_1[i][1][0];
        I_spa[i][4][1] = I_2_1[i][1][1];
        I_spa[i][4][2] = I_2_1[i][1][2];
        I_spa[i][4][3] = 0;
        I_spa[i][4][4] = mass[i];
        I_spa[i][4][5] = 0;

        I_spa[i][5][0] = I_2_1[i][2][0];
        I_spa[i][5][1] = I_2_1[i][2][1];
        I_spa[i][5][2] = I_2_1[i][2][2];
        I_spa[i][5][3] = 0;
        I_spa[i][5][4] = 0;
        I_spa[i][5][5] = mass[i];
    }
    
}
