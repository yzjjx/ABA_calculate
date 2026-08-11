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

void tip_pass3(
    const data_t X_lam[DOF][6][6],
    const data_t c[DOF][6],
    const data_t D[DOF],
    const data_t u[DOF],
    const data_t U[DOF][6],
    data_t ddq[DOF]
)
{
    data_t a_parent[6] = {0, 0, 0, 0, 0, 9.81};

    data_t a_s[6];

    for(int i = 0;i<DOF;i++)
    {
        for(int j = 0;j<6;j++)
        {
            a_s[j] =  X_lam[i][j][0]*a_parent[0]+
                      X_lam[i][j][1]*a_parent[1]+
                      X_lam[i][j][2]*a_parent[2]+
                      X_lam[i][j][3]*a_parent[3]+
                      X_lam[i][j][4]*a_parent[4]+
                      X_lam[i][j][5]*a_parent[5]+
                      c[i][j];
        }

        const data_t Ua =
            U[i][0] * a_s[0] +
            U[i][1] * a_s[1] +
            U[i][2] * a_s[2] +
            U[i][3] * a_s[3] +
            U[i][4] * a_s[4] +
            U[i][5] * a_s[5];

        data_t inv_D = 1.0f / D[i];

        ddq[i] = (u[i]-Ua)*inv_D;

        a_s[2] += ddq[i];

        for(int k = 0;k<6;k++)
        {
            a_parent[k] = a_s[k];
        }
    }
}