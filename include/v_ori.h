#ifndef V_ORI
#define V_ORI

#include <cmath>
#include <T_R_out.h>

void v_fina(
    const data_t q[DOF],
    const data_t dq[DOF],
    data_t E[DOF][3][3],
    data_t F[DOF][3][3],
    data_t v[DOF][6]
);

#endif