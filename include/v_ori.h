#ifndef V_ORI
#define V_ORI

#include <cmath>
#include <T_R_out.h>

void cal_v_J(
    const data_t dq[DOF],
    data_t v_J[6][DOF]
);

void v_fina(
    const data_t q[DOF],
    const data_t dq[DOF],
    data_t X_lam[DOF][6][6],
    data_t v[DOF][6]
);

#endif