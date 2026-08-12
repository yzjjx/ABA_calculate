#ifndef BACK
#define BACK

#include <cmath>
#include <v_ori.h>
#include <cal_p.h>

// void mat_6x6(
//     const data_t in_1[6][6],
//     const data_t in_2[6][6],
//     data_t out[6][6]
// );

void back_pass(
    const data_t I_spa[DOF][6][6],
    const data_t p[DOF][6],
    const data_t tau[DOF],
    const data_t c[DOF][6],
    const data_t E[DOF][3][3],
    const data_t F[DOF][3][3],
    data_t U[DOF][6],
    data_t inv_D[DOF],
    data_t u[DOF]
);

#endif