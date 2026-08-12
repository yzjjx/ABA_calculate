#ifndef BLOCK_MAT_H
#define BLOCK_MAT_H

#include <cmath>
#include <T_R_out.h>

void v_mat_cal(
    const data_t E[3][3],
    const data_t F[3][3],
    const data_t in[6],
    data_t out[6]);

void f_mat_cal(
    const data_t E[3][3],
    const data_t F[3][3],
    const data_t in[6],
    data_t out[6]);

static data_t dot3(
    const data_t a0,
    const data_t a1,
    const data_t a2,
    const data_t b0,
    const data_t b1,
    const data_t b2
);

void transform_inertia_ef(
    const data_t E[3][3],
    const data_t F[3][3],
    const data_t Ia[6][6],
    data_t result[6][6]
);

#endif