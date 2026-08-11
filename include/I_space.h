#ifndef I_SPACE_H
#define I_SPACE_H

#include <cmath>
#include "ABA_parms.h"

void I_space(
    const data_t mass[DOF],
    const data_t c_of_mass[DOF][3],
    const data_t I[DOF][9],//行展开矩阵
    data_t I_spa[DOF][6][6]
);

#endif