#ifndef CAL_P_H
#define CAL_P_H

#include <cmath>
#include <c.h>
#include <v_ori.h>

void h_fina(
    const data_t I_spa[DOF][6][6],
    const data_t v[DOF][6],
    data_t h[DOF][6]
);

void f_space_cross(
    const data_t in_1[6],
    const data_t in_2[6],
    data_t out[6]
);

// 空间叉乘计算
void p_fina(
    const data_t v[DOF][6],
    const data_t h[DOF][6],
    data_t p[DOF][6]
);

#endif