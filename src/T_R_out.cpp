#include <cmath>
#include "T_R_out.h"
#include "ABA_parms.h"

void cal_R_Trans(
    const data_t q[DOF],
    data_t R_Trans[DOF][3][3])
{
    for (int i = 0; i < DOF; i++)
    {
        const data_t cq = std::cos(q[i]);
        const data_t sq = std::sin(q[i]);

        const data_t ca = alpha_cos[i];
        const data_t sa = alpha_sin[i];

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