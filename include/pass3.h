#ifndef PASS3
#define PASS3

#include <cmath>
#include <v_ori.h>
#include <cal_p.h>

void tip_pass3(
    const data_t E[DOF][3][3],
    const data_t F[DOF][3][3],
    const data_t c[DOF][6],
    const data_t inv_D[DOF],
    const data_t u[DOF],
    const data_t U[DOF][6],
    data_t ddq[DOF]
);

#endif