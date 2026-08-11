#ifndef BACK
#define BACK

#include <cmath>
#include <v_ori.h>
#include <cal_p.h>

void mat_6x6(
    const data_t in_1[6][6],
    const data_t in_2[6][6],
    data_t out[6][6]
);

void back_pass(
    const data_t I_spa[DOF][6][6],
    const data_t p[DOF][6],
    const data_t tau[DOF],
    const data_t c[DOF][6],
    const data_t X_lam[DOF][6][6],
    data_t I_A[DOF][6][6],
    data_t p_A[DOF][6],
    data_t U[DOF][6],
    data_t D[DOF],
    data_t u[DOF],
    data_t I_a[DOF][6][6],
    data_t p_a[DOF][6]
);

#endif