#ifndef C
#define C

#include <cmath>
#include <v_ori.h>

void space_cross(
    const data_t in_1[6],
    const data_t in_2[6],
    data_t out[6]
);

void c_fina(
    const data_t v[DOF][6],
    const data_t dq[DOF],
    data_t c[DOF][6]
);

#endif