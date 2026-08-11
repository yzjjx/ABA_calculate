#ifndef T_R_OUT
#define T_R_OUT

#include <cmath>

typedef float data_t;
const int DOF = 6;

void cal_R_Trans(
    const data_t q[DOF],
    data_t R_Trans[DOF][3][3]
);

#endif